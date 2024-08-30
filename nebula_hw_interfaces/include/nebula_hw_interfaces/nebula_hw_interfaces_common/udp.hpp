#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <iostream>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>
namespace nebula::drivers
{

class UdpSocket
{
public:
  using callback_t = std::function<void(std::vector<uint8_t> &, uint64_t)>;

  UdpSocket()
  {
    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd == -1) {
      throw std::runtime_error(strerror(errno));
    }

    int enable = 1;
    if (setsockopt(sock_fd, SOL_SOCKET, SO_TIMESTAMP, &enable, sizeof(enable)) < 0) {
      throw std::runtime_error(strerror(errno));
    }

    sock_fd_ = sock_fd;
  }

  void init(std::string sensor_ip, uint16_t sensor_port, std::string host_ip, uint16_t host_port)
  {
    init(host_ip, host_port);
    sender_.emplace(Endpoint{sensor_ip, sensor_port});
  }

  void init(std::string host_ip, uint16_t host_port) { host_ = {host_ip, host_port}; }

  void setBufferSize(size_t bytes) { buffer_size = bytes; }

  void open() {}

  void bind()
  {
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(host_.port);
    addr.sin_addr.s_addr = INADDR_ANY;

    if (::bind(sock_fd_, (struct sockaddr *)&addr, sizeof(addr)) == -1) {
      throw std::runtime_error(strerror(errno));
    }

    if (!sender_.has_value()) return;

    struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(host_.ip.c_str());  // Multicast group address
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);

    int result = setsockopt(sock_fd_, IPPROTO_IP, IP_ADD_MEMBERSHIP, (char *)&mreq, sizeof(mreq));
    if (result < 0) {
      throw std::runtime_error(strerror(errno));
    }
  }

  void asyncReceive(callback_t callback)
  {
    callback_ = std::move(callback);
    launch_receiver();
  }

  ~UdpSocket()
  {
    running_ = false;
    if (receive_thread_.joinable()) receive_thread_.join();
    close(sock_fd_);
  }

private:
  void launch_receiver()
  {
    running_ = true;
    receive_thread_ = std::thread([&]() {
      std::vector<uint8_t> buffer;
      while (running_) {
        buffer.resize(buffer_size);
        msghdr msg{};
        iovec iov{};
        char control[1024];

        struct sockaddr_in sender_addr;
        socklen_t sender_addr_len = sizeof(sender_addr);

        iov.iov_base = buffer.data();
        iov.iov_len = buffer.size();

        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control;
        msg.msg_controllen = sizeof(control);
        msg.msg_name = &sender_addr;
        msg.msg_namelen = sender_addr_len;

        ssize_t received = recvmsg(sock_fd_, &msg, 0);
        if (received < 0) {
          throw std::runtime_error(strerror(errno));
        }

        buffer.resize(received);

        cmsghdr * cmsg;
        timeval * tv = nullptr;

        for (cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr; cmsg = CMSG_NXTHDR(&msg, cmsg)) {
          if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP) {
            tv = (struct timeval *)CMSG_DATA(cmsg);
            break;
          }
        }

        if (!tv) {
          throw std::runtime_error("Could not get timestamp");
        }

        uint64_t timestamp = tv->tv_sec * 1'000'000'000 + tv->tv_usec * 1000;

        if (sender_.has_value()) {
          char str[INET_ADDRSTRLEN];
          inet_ntop(AF_INET, &sender_addr.sin_addr, str, INET_ADDRSTRLEN);
          if (sender_->ip != std::string(str)) {
            std::cout << str << " != " << sender_->ip << std::endl;
            continue;
          }
        }
        callback_(buffer, timestamp);
      }
    });
  }

  int sock_fd_;

  struct Endpoint
  {
    std::string ip;
    uint16_t port;
  };

  bool running_ = false;
  size_t buffer_size = 1500;
  Endpoint host_;
  std::optional<Endpoint> sender_;
  std::thread receive_thread_;
  callback_t callback_;
};

}  // namespace nebula::drivers