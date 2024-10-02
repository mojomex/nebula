// Copyright 2024 TIER IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <array>
#include <atomic>
#include <cassert>
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace nebula::drivers
{

using std::string_literals::operator""s;

class UdpDriver
{
public:
  using callback_t = void(std::vector<uint8_t> &, size_t);

  explicit UdpDriver(std::function<callback_t> callback) : callback_(std::move(callback))
  {
    if (!callback_) {
      throw std::runtime_error("No callback given");
    }
  }

  ~UdpDriver() { close(); }

  void init(const std::string & host_ip, uint16_t data_port)
  {
    close();

    running_ = true;
    receiver_ = std::thread{[this, data_port, host_ip]() {
      int sockfd = socket(AF_INET, SOCK_DGRAM, 0);
      if (sockfd < 0) {
        throw std::runtime_error("Could not connect to socket: "s + std::strerror(errno));
      }

      sockaddr_in server_addr{};
      server_addr.sin_family = AF_INET;
      server_addr.sin_port = htons(data_port);

      if (inet_pton(AF_INET, host_ip.c_str(), &server_addr.sin_addr) <= 0) {
        auto error = errno;
        ::close(sockfd);
        throw std::runtime_error("Invalid IP address: "s + std::strerror(error));
      }

      int opt = 1;
      if (setsockopt(sockfd, SOL_SOCKET, SO_RXQ_OVFL, &opt, sizeof(opt)) == -1) {
        auto error = errno;
        ::close(sockfd);
        throw std::runtime_error(
          "Could not configure overflow reporting: "s + std::strerror(error));
      }

      int recv_buf_size = 1500 * 3600;
      if (setsockopt(sockfd, SOL_SOCKET, SO_RCVBUF, &recv_buf_size, sizeof(recv_buf_size)) == -1) {
        auto error = errno;
        ::close(sockfd);
        throw std::runtime_error(
          "Could not configure internal socket buffer size: "s + std::strerror(error));
      }

      int reuse = 1;
      if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse)) < 0) {
        auto error = errno;
        ::close(sockfd);
        throw std::runtime_error("Could not set SO_REUSEADDR: "s + std::strerror(error));
      }

      if (bind(sockfd, reinterpret_cast<sockaddr *>(&server_addr), sizeof(server_addr)) < 0) {
        auto error = errno;
        ::close(sockfd);
        throw std::runtime_error("Could not bind socket: "s + std::strerror(error));
      }

      uint32_t dropped_previously = 0;
      while (running_) {
        std::vector<uint8_t> recv_buffer;
        recv_buffer.resize(1500);

        msghdr msg = {nullptr};
        iovec iov = {nullptr};
        std::array<char, CMSG_SPACE(sizeof(uint32_t))> control{};

        iov.iov_base = recv_buffer.data();
        iov.iov_len = recv_buffer.size();

        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_control = control.data();
        msg.msg_controllen = control.size();

        ssize_t n_received = recvmsg(sockfd, &msg, 0);
        if (n_received < 0) {
          ::close(sockfd);
          throw std::runtime_error("Could receive message: "s + std::strerror(errno));
        }

        if (n_received == 0) continue;

        recv_buffer.resize(n_received);

        uint32_t n_dropped = 0;
        for (cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
             cmsg = CMSG_NXTHDR(&msg, cmsg)) {
          if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_RXQ_OVFL) {
            n_dropped = *reinterpret_cast<uint32_t *>(CMSG_DATA(cmsg));
          }
        }

        if (n_dropped > 0) {
          uint32_t diff = n_dropped - dropped_previously;
          dropped_previously = n_dropped;
          n_dropped = diff;
        }

        callback_(recv_buffer, n_dropped);
      }

      ::close(sockfd);
    }};
  }

  void close()
  {
    running_ = false;
    if (receiver_.joinable()) {
      receiver_.join();
    }
  }

private:
  std::function<callback_t> callback_;
  std::atomic<bool> running_ = false;
  std::thread receiver_;
};

}  // namespace nebula::drivers
