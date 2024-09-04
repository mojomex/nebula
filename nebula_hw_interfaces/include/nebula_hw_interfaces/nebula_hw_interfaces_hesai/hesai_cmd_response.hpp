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

#ifndef HESAI_CMD_RESPONSE_HPP
#define HESAI_CMD_RESPONSE_HPP

#include <boost/algorithm/string/join.hpp>
#include <boost/endian/buffers.hpp>
#include <boost/format.hpp>

#include <array>
#include <cstdint>
#include <ostream>
#include <string>
#include <nlohmann/json.hpp>

using namespace boost::endian;  // NOLINT(build/namespaces)
using nlohmann::json;

namespace nebula
{

#pragma pack(push, 1)

/// @brief PTP STATUS struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagStatus
{
  big_int64_buf_t master_offset;
  big_int32_buf_t ptp_state;
  big_int32_buf_t elapsed_millisec;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagStatus const & arg)
  {
    os << "master_offset: " << arg.master_offset;
    os << ", ";
    os << "ptp_state: " << arg.ptp_state;
    os << ", ";
    os << "elapsed_millisec: " << arg.elapsed_millisec;

    return os;
  }
};

/// @brief PTP TLV PORT_DATA_SET struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagPort
{
  uint8_t portIdentity[10];
  big_int32_buf_t portState;
  big_int32_buf_t logMinDelayReqInterval;
  big_int64_buf_t peerMeanPathDelay;
  big_int32_buf_t logAnnounceInterval;
  big_int32_buf_t announceReceiptTimeout;
  big_int32_buf_t logSyncInterval;
  big_int32_buf_t delayMechanism;
  big_int32_buf_t logMinPdelayReqInterval;
  big_int32_buf_t versionNumber;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagPort const & arg)
  {
    os << "portIdentity: " << std::string(std::begin(arg.portIdentity), std::end(arg.portIdentity));
    os << ", ";
    os << "portState: " << arg.portState;
    os << ", ";
    os << "logMinDelayReqInterval: " << arg.logMinDelayReqInterval;
    os << ", ";
    os << "peerMeanPathDelay: " << arg.peerMeanPathDelay;
    os << ", ";
    os << "logAnnounceInterval: " << arg.logAnnounceInterval;
    os << ", ";
    os << "announceReceiptTimeout: " << arg.announceReceiptTimeout;
    os << ", ";
    os << "logSyncInterval: " << arg.logSyncInterval;
    os << ", ";
    os << "delayMechanism: " << arg.delayMechanism;
    os << ", ";
    os << "logMinPdelayReqInterval: " << arg.logMinPdelayReqInterval;
    os << ", ";
    os << "versionNumber: " << arg.versionNumber;

    return os;
  }
};

/// @brief LinuxPTP TLV TIME_STATUS_NP struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagTime
{
  big_int64_buf_t master_offset;
  big_int64_buf_t ingress_time;
  big_int32_buf_t cumulativeScaledRateOffset;
  big_int32_buf_t scaledLastGmPhaseChange;
  big_int16_buf_t gmTimeBaseIndicator;
  uint8_t lastGmPhaseChange[12];
  big_int32_buf_t gmPresent;
  big_int64_buf_t gmIdentity;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagTime const & arg)
  {
    os << "master_offset: " << arg.master_offset;
    os << ", ";
    os << "ingress_time: " << arg.ingress_time;
    os << ", ";
    os << "cumulativeScaledRateOffset: " << arg.cumulativeScaledRateOffset;
    os << ", ";
    os << "scaledLastGmPhaseChange: " << arg.scaledLastGmPhaseChange;
    os << ", ";
    os << "gmTimeBaseIndicator: " << arg.gmTimeBaseIndicator;
    os << ", ";
    // FIXME: lastGmPhaseChange is a binary number, displaying it as string is incorrect
    os << "lastGmPhaseChange: "
       << std::string(std::begin(arg.lastGmPhaseChange), std::end(arg.lastGmPhaseChange));
    os << ", ";
    os << "gmPresent: " << arg.gmPresent;
    os << ", ";
    os << "gmIdentity: " << arg.gmIdentity;

    return os;
  }
};

/// @brief LinuxPTP TLV GRANDMASTER_SETTINGS_NP struct of PTC_COMMAND_PTP_DIAGNOSTICS
struct HesaiPtpDiagGrandmaster
{
  big_int32_buf_t clockQuality;
  big_int16_buf_t utc_offset;
  int8_t time_flags;
  int8_t time_source;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpDiagGrandmaster const & arg)
  {
    os << "clockQuality: " << arg.clockQuality;
    os << ", ";
    os << "utc_offset: " << arg.utc_offset;
    os << ", ";
    os << "time_flags: " << +arg.time_flags;
    os << ", ";
    os << "time_source: " << +arg.time_source;

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_INVENTORY_INFO
struct HesaiInventory
{
  char sn[18];
  char date_of_manufacture[16];
  uint8_t mac[6];
  char sw_ver[16];
  char hw_ver[16];
  char control_fw_ver[16];
  char sensor_fw_ver[16];
  big_uint16_buf_t angle_offset;
  uint8_t model;
  uint8_t motor_type;
  uint8_t num_of_lines;
  uint8_t reserved[11];

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiInventory const & arg)
  {
    std::ios initial_format(nullptr);
    initial_format.copyfmt(os);

    os << "sn: " << std::string(arg.sn, strnlen(arg.sn, sizeof(arg.sn)));
    os << ", ";
    os << "date_of_manufacture: "
       << std::string(
            arg.date_of_manufacture,
            strnlen(arg.date_of_manufacture, sizeof(arg.date_of_manufacture)));
    os << ", ";
    os << "mac: ";

    for (size_t i = 0; i < sizeof(arg.mac); i++) {
      if (i != 0) {
        os << ':';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (+arg.mac[i]);
    }
    os.copyfmt(initial_format);

    os << ", ";
    os << "sw_ver: " << std::string(arg.sw_ver, strnlen(arg.sw_ver, sizeof(arg.sw_ver)));
    os << ", ";
    os << "hw_ver: " << std::string(arg.hw_ver, strnlen(arg.hw_ver, sizeof(arg.hw_ver)));
    os << ", ";
    os << "control_fw_ver: "
       << std::string(arg.control_fw_ver, strnlen(arg.control_fw_ver, sizeof(arg.control_fw_ver)));
    os << ", ";
    os << "sensor_fw_ver: "
       << std::string(arg.sensor_fw_ver, strnlen(arg.sensor_fw_ver, sizeof(arg.sensor_fw_ver)));
    os << ", ";
    os << "angle_offset: " << arg.angle_offset;
    os << ", ";
    os << "model: " << +arg.model;
    os << ", ";
    os << "motor_type: " << +arg.motor_type;
    os << ", ";
    os << "num_of_lines: " << +arg.num_of_lines;
    os << ", ";

    for (size_t i = 0; i < sizeof(arg.reserved); i++) {
      if (i != 0) {
        os << ' ';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (+arg.reserved[i]);
    }
    os.copyfmt(initial_format);

    return os;
  }

  std::string get_str_model()
  {
    switch (model) {
      case 0:
        return "Pandar40P";
      case 2:
        return "Pandar64";
      case 3:
        return "Pandar128";
      case 15:
        return "PandarQT";
      case 17:
        return "Pandar40M";
      case 20:
        return "PandarMind(PM64)";
      case 25:
        return "PandarXT32";
      case 26:
        return "PandarXT16";
      case 32:
        return "QT128C2X";
      case 38:
        return "PandarXT32M";
      case 42:
        return "OT128";
      case 48:
        return "PandarAT128";
      default:
        return "Unknown(" + std::to_string(static_cast<int>(model)) + ")";
    }
  }
};

/// @brief struct of PTC_COMMAND_GET_CONFIG_INFO
struct HesaiConfigBase
{
  uint8_t ipaddr[4];
  uint8_t mask[4];
  uint8_t gateway[4];
  uint8_t dest_ipaddr[4];
  big_uint16_buf_t dest_LiDAR_udp_port;
  big_uint16_buf_t dest_gps_udp_port;
  big_uint16_buf_t spin_rate;
  uint8_t sync;
  big_uint16_buf_t sync_angle;
  big_uint16_buf_t start_angle;
  big_uint16_buf_t stop_angle;
  uint8_t clock_source;
  uint8_t udp_seq;
  uint8_t trigger_method;
  uint8_t return_mode;
  uint8_t standby_mode;
  uint8_t motor_status;
  uint8_t vlan_flag;
  big_uint16_buf_t vlan_id;

  json to_json(HesaiConfigBase & instance)
  {
    json j;
    j["ipaddr"] = instance.ipaddr;
    j["mask"] = instance.mask;
    j["gateway"] = instance.gateway;
    j["dest_ipaddr"] = instance.dest_ipaddr;
    j["dest_LiDAR_udp_port"] = instance.dest_LiDAR_udp_port.value();
    j["dest_gps_udp_port"] = instance.dest_gps_udp_port.value();
    j["spin_rate"] = instance.spin_rate.value();
    j["sync"] = instance.sync;
    j["sync_angle"] = instance.sync_angle.value();
    j["start_angle"] = instance.start_angle.value();
    j["stop_angle"] = instance.stop_angle.value();
    j["clock_source"] = instance.clock_source;
    j["udp_seq"] = instance.udp_seq;
    j["trigger_method"] = instance.trigger_method;
    j["return_mode"] = instance.return_mode;
    j["standby_mode"] = instance.standby_mode;
    j["motor_status"] = instance.motor_status;
    j["vlan_flag"] = instance.vlan_flag;
    j["vlan_id"] = instance.vlan_id.value();

    return j;
  }
};

inline std::ostream & operator<<(std::ostream & os, HesaiConfigBase const & arg)
{
  os << "ipaddr: " << arg.ipaddr << '\n';
  os << "mask: " << arg.mask << '\n';
  os << "gateway: " << arg.gateway << '\n';
  os << "dest_ipaddr: " << arg.dest_ipaddr << '\n';
  os << "dest_LiDAR_udp_port: " << arg.dest_LiDAR_udp_port << '\n';
  os << "dest_gps_udp_port: " << arg.dest_gps_udp_port << '\n';
  os << "spin_rate: " << arg.spin_rate << '\n';
  os << "sync: " << +arg.sync << '\n';
  os << "sync_angle: " << arg.sync_angle << '\n';
  os << "start_angle: " << arg.start_angle << '\n';
  os << "stop_angle: " << arg.stop_angle << '\n';
  os << "clock_source: " << +arg.clock_source << '\n';
  os << "udp_seq: " << +arg.udp_seq << '\n';
  os << "trigger_method: " << +arg.trigger_method << '\n';
  os << "return_mode: " << +arg.return_mode << '\n';
  os << "standby_mode: " << +arg.standby_mode << '\n';
  os << "motor_status: " << +arg.motor_status << '\n';
  os << "vlan_flag: " << +arg.vlan_flag << '\n';
  os << "vlan_id: " << arg.vlan_id;
  return os;
}

struct OTAT128HesaiConfig: HesaiConfigBase
{
  uint8_t gps_nmea_sentence;
  uint8_t noise_filtering;
  uint8_t reflectivity_mapping;
  unsigned char reserved[6];

  json to_json(OTAT128HesaiConfig & instance)
  {
    json j = HesaiConfigBase::to_json(instance);
    j["gps_nmea_sentence"] = instance.gps_nmea_sentence;
    j["noise_filtering"] = instance.noise_filtering;
    j["reflectivity_mapping"] = instance.reflectivity_mapping;

    return j;
  }
};

inline std::ostream & operator<<(std::ostream & os, OTAT128HesaiConfig const & arg)
{
  os << (HesaiConfigBase)(arg) << '\n';
  os << ", ";
  os << "gps_nmea_sentence: " << +arg.gps_nmea_sentence;
  os << ", ";
  os << "noise_filtering: " << +arg.noise_filtering;
  os << ", ";
  os << "reflectivity_mapping: " << +arg.reflectivity_mapping;
  os << ", ";
  os << "reserved: ";
  for (size_t i = 0; i < sizeof(arg.reserved); i++) {
    if (i != 0) {
      os << ' ';
    }
    os << std::hex << std::setfill('0') << std::setw(2) << (+arg.reserved[i]);
  }
  return os;
}

struct XT40pHesaiConfig: HesaiConfigBase
{
  uint8_t clock_data_fmt;
  uint8_t noise_filtering;
  uint8_t reflectivity_mapping;
  unsigned char reserved[6];

  json to_json(XT40pHesaiConfig & instance)
  {
    json j = HesaiConfigBase::to_json(instance);
    j["clock_data_fmt"] = instance.clock_data_fmt;
    j["noise_filtering"] = instance.noise_filtering;
    j["reflectivity_mapping"] = instance.reflectivity_mapping;

    return j;
  }
};

inline std::ostream & operator<<(std::ostream & os, XT40pHesaiConfig const & arg)
{
  os << (HesaiConfigBase)(arg) << '\n';
  os << ", ";
  os << "clock_data_fmt: " << +arg.clock_data_fmt;
  os << ", ";
  os << "noise_filtering: " << +arg.noise_filtering;
  os << ", ";
  os << "reflectivity_mapping: " << +arg.reflectivity_mapping;
  os << ", ";
  os << "reserved: ";    return os;
  os << "reserved: ";
  for (size_t i = 0; i < sizeof(arg.reserved); i++) {
    if (i != 0) {
      os << ' ';
    }
    os << std::hex << std::setfill('0') << std::setw(2) << (+arg.reserved[i]);
  }
  return os;
}

/// @brief struct of PTC_COMMAND_GET_LIDAR_STATUS
struct HesaiLidarStatus
{
  big_uint32_buf_t system_uptime;
  big_uint16_buf_t motor_speed;
  big_int32_buf_t temperature[8];
  uint8_t gps_pps_lock;
  uint8_t gps_gprmc_status;
  big_uint32_buf_t startup_times;
  big_uint32_buf_t total_operation_time;
  uint8_t ptp_clock_status;
  uint8_t reserved[5];  // FIXME: 4 bytes labeled as humidity in OT128 datasheet

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarStatus const & arg)
  {
    std::ios initial_format(nullptr);
    initial_format.copyfmt(os);

    os << "system_uptime: " << arg.system_uptime;
    os << ", ";
    os << "motor_speed: " << arg.motor_speed;
    os << ", ";
    os << "temperature: ";

    for (size_t i = 0; i < sizeof(arg.temperature); i++) {
      if (i != 0) {
        os << ',';
      }
      os << arg.temperature[i];
    }

    os << ", ";
    os << "gps_pps_lock: " << static_cast<int>(arg.gps_pps_lock);
    os << ", ";
    os << "gps_gprmc_status: " << static_cast<int>(arg.gps_gprmc_status);
    os << ", ";
    os << "startup_times: " << arg.startup_times;
    os << ", ";
    os << "total_operation_time: " << arg.total_operation_time;
    os << ", ";
    os << "ptp_clock_status: " << static_cast<int>(arg.ptp_clock_status);
    os << ", ";
    os << "reserved: ";

    for (size_t i = 0; i < sizeof(arg.reserved); i++) {
      if (i != 0) {
        os << ' ';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (static_cast<int>(arg.reserved[i]));
    }
    os.copyfmt(initial_format);

    return os;
  }

  std::string get_str_gps_pps_lock()
  {
    switch (gps_pps_lock) {
      case 1:
        return "Lock";
      case 0:
        return "Unlock";
      default:
        return "Unknown";
    }
  }
  std::string get_str_gps_gprmc_status()
  {
    switch (gps_gprmc_status) {
      case 1:
        return "Lock";
      case 0:
        return "Unlock";
      default:
        return "Unknown";
    }
  }
  std::string get_str_ptp_clock_status()
  {
    switch (ptp_clock_status) {
      case 0:
        return "free run";
      case 1:
        return "tracking";
      case 2:
        return "locked";
      case 3:
        return "frozen";
      default:
        return "Unknown";
    }
  }
};

/// @brief struct of PTC_COMMAND_GET_LIDAR_RANGE
struct HesaiLidarRangeAll
{
  uint8_t method;
  big_uint16_buf_t start;
  big_uint16_buf_t end;

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarRangeAll const & arg)
  {
    os << "method: " << static_cast<int>(arg.method);
    os << ", ";
    os << "start: " << static_cast<int>(arg.start.value());
    os << ", ";
    os << "end: " << static_cast<int>(arg.end.value());

    return os;
  }
};

/// @brief struct of PTC_COMMAND_GET_PTP_CONFIG
struct HesaiPtpConfig
{
  int8_t status;
  int8_t profile;
  int8_t domain;
  int8_t network;
  int8_t logAnnounceInterval;
  int8_t logSyncInterval;
  int8_t logMinDelayReqInterval;
  // FIXME: this format is not correct for OT128, or for AT128 on 802.1AS

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiPtpConfig const & arg)
  {
    os << "status: " << static_cast<int>(arg.status);
    os << ", ";
    os << "profile: " << static_cast<int>(arg.profile);
    os << ", ";
    os << "domain: " << static_cast<int>(arg.domain);
    os << ", ";
    os << "network: " << static_cast<int>(arg.network);
    if (arg.status == 0) {
      os << ", ";
      os << "logAnnounceInterval: " << static_cast<int>(arg.logAnnounceInterval);
      os << ", ";
      os << "logSyncInterval: " << static_cast<int>(arg.logSyncInterval);
      os << ", ";
      os << "logMinDelayReqInterval: " << static_cast<int>(arg.logMinDelayReqInterval);
    }
    return os;
  }
};

/// @brief struct of PTC_COMMAND_LIDAR_MONITOR
struct HesaiLidarMonitor
{
  // FIXME: this format is not correct for OT128
  big_int32_buf_t input_voltage;
  big_int32_buf_t input_current;
  big_int32_buf_t input_power;
  uint8_t reserved[52];

  friend std::ostream & operator<<(std::ostream & os, nebula::HesaiLidarMonitor const & arg)
  {
    os << "input_voltage: " << arg.input_voltage;
    os << ", ";
    os << "input_current: " << arg.input_current;
    os << ", ";
    os << "input_power: " << arg.input_power;
    os << ", ";
    os << "reserved: ";

    for (size_t i = 0; i < sizeof(arg.reserved); i++) {
      if (i != 0) {
        os << ' ';
      }
      os << std::hex << std::setfill('0') << std::setw(2) << (static_cast<int>(arg.reserved[i]));
    }

    return os;
  }
};

#pragma pack(pop)

}  // namespace nebula
#endif  // HESAI_CMD_RESPONSE_HPP
