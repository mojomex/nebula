#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/ptc.hpp"

#include <iostream>
#include <iomanip>

namespace nebula
{
namespace drivers
{

PTCConnection::PTCConnection(const std::string & sensor_ip, uint16_t sensor_port, Logger logger)
: driver_(std::make_shared<boost::asio::io_context>(1)), logger_(logger)
{
  driver_.init_socket(sensor_ip, sensor_port);
  if (!driver_.open()) {
    driver_.closeSync();
    throw std::runtime_error("Could not open TCP socket");
  }
}

PTCConnection::ptc_cmd_result_t PTCConnection::executeCommand(uint8_t command_id, const std::vector<uint8_t> & payload)
{
  std::lock_guard lock(mtx_inflight_tcp_request_);

  uint32_t len = payload.size();

  std::vector<uint8_t> send_buf;
  send_buf.emplace_back(PTC_COMMAND_HEADER_HIGH);
  send_buf.emplace_back(PTC_COMMAND_HEADER_LOW);
  send_buf.emplace_back(command_id);
  send_buf.emplace_back(PTC_COMMAND_DUMMY_BYTE);
  send_buf.emplace_back((len >> 24) & 0xff);
  send_buf.emplace_back((len >> 16) & 0xff);
  send_buf.emplace_back((len >> 8) & 0xff);
  send_buf.emplace_back(len & 0xff);
  send_buf.insert(send_buf.end(), payload.begin(), payload.end());

  // These are shared_ptrs so that in case of request timeout, the callback (if ever called) can
  // access valid memory
  auto recv_buf = std::make_shared<std::vector<uint8_t>>();
  auto response_complete = std::make_shared<bool>(false);

  auto error_code = std::make_shared<ptc_error_t>();

  std::stringstream ss;
  ss << "0x" << std::setfill('0') << std::setw(2) << std::hex << static_cast<int>(command_id)
     << " (" << len << ") ";
  std::string log_tag = ss.str();

  logger_.debug(log_tag + "Entering lock");

  std::timed_mutex tm;
  tm.lock();

  if (driver_.GetIOContext()->stopped()) {
    logger_.debug(log_tag + "IOContext was stopped");
    driver_.GetIOContext()->restart();
  }

  logger_.debug(log_tag + "Sending payload");
  driver_.asyncSendReceiveHeaderPayload(
    send_buf,
    [this, log_tag, command_id, response_complete,
     error_code](const std::vector<uint8_t> & header_bytes) {
      error_code->ptc_error_code = header_bytes[3];

      size_t payload_len = (header_bytes[4] << 24) | (header_bytes[5] << 16) |
                           (header_bytes[6] << 8) | header_bytes[7];
      logger_.debug(
        log_tag + "Received header (expecting " + std::to_string(payload_len) + "B payload)");
      // If command_id in the response does not match, we got a response for another command (or
      // rubbish), probably as a result of too many simultaneous TCP connections to the sensor (e.g.
      // from GUI, Web UI, another nebula instance, etc.)
      if (header_bytes[2] != command_id) {
        error_code->error_flags |= TCP_ERROR_UNRELATED_RESPONSE;
      }
      if (payload_len == 0) {
        *response_complete = true;
      }
    },
    [this, log_tag, recv_buf, response_complete,
     error_code](const std::vector<uint8_t> & payload_bytes) {
      logger_.debug(log_tag + "Received payload");

      // Header had payload length 0 (thus, header callback processed request successfully already),
      // but we still received a payload: invalid state
      if (*response_complete == true) {
        error_code->error_flags |= TCP_ERROR_UNEXPECTED_PAYLOAD;
        return;
      }

      // Skip 8 header bytes
      recv_buf->insert(recv_buf->end(), std::next(payload_bytes.begin(), 8), payload_bytes.end());
      *response_complete = true;
    },
    [this, log_tag, &tm]() {
      logger_.debug(log_tag + "Unlocking mutex");
      tm.unlock();
      logger_.debug(log_tag + "Unlocked mutex");
    });

  driver_.GetIOContext()->run();

  if (!tm.try_lock_for(std::chrono::seconds(1))) {
    logger_.error(log_tag + "Request did not finish within 1s");
    error_code->error_flags |= TCP_ERROR_TIMEOUT;
    return *error_code;
  }

  if (!response_complete) {
    logger_.error(log_tag + "Did not receive response");
    error_code->error_flags |= TCP_ERROR_INCOMPLETE_RESPONSE;
    return *error_code;
  }

  if (!error_code->ok()) {
    return *error_code;
  }

  logger_.debug(log_tag + "Received response");

  return *recv_buf;
}

}  // namespace drivers
}  // namespace nebula
