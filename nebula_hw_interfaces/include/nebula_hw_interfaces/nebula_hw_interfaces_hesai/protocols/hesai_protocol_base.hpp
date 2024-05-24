#pragma once

#include "nebula_hw_interfaces/nebula_hw_interfaces_common/connections/udp.hpp"
#include "nebula_hw_interfaces/nebula_hw_interfaces_hesai/connections/ptc.hpp"

namespace nebula
{
namespace drivers
{

class PTCCommandGetLidarStatus
{
  static const uint8_t COMMAND_ID = 0x09;

#pragma pack(push, 1)
  struct ResponsePayload
  {
    int32_t system_uptime;
    int16_t motor_speed;
    uint32_t temp_tx1;
    uint32_t temp_tx2;
    uint32_t fpga_temp;
    uint32_t temp_rx1;
    uint32_t temp_rx2;
    uint32_t temp_mb1;
    uint32_t temp_mb2;
    uint32_t temp_pb;
    uint32_t temp_hot;
    int8_t gps_pps_lock;
    int8_t gps_gprmc_status;
    int32_t startup_times;
    int32_t total_operation_time;
    int8_t ptp_status;
    uint8_t reserved[5];
  };
#pragma pack(pop)

  std::vector<uint8_t> buildRequest();
  ResponsePayload parseResult(std::vector<uint8_t> & data);
}

class HesaiProtocolBase
{
private:
  UDPConnection cloud_connection_;
  PTCConnection ptc_connection_;
};

}  // namespace drivers
}  // namespace nebula
