#include <boost/endian/buffers.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <variant>

using std::chrono::microseconds;
using std::chrono::milliseconds;
using std::chrono::nanoseconds;
using std::chrono::seconds;

using namespace boost::endian;  // NOLINT(build/namespaces)

namespace xt
{

const char PTC_COMMAND_GET_LIDAR_CALIBRATION = 0x05;
const char PTC_COMMAND_PTP_DIAGNOSTICS = 0x06;
const char PTC_COMMAND_GET_INVENTORY_INFO = 0x07;
const char PTC_COMMAND_GET_CONFIG_INFO = 0x08;
const char PTC_COMMAND_GET_LIDAR_STATUS = 0x09;
const char PTC_COMMAND_SET_SPIN_RATE = 0x17;
const char PTC_COMMAND_SET_TRIGGER_METHOD = 0x1b;
const char PTC_COMMAND_SET_RETURN_MODE = 0x1e;
const char PTC_COMMAND_SET_CLOCK_SOURCE = 0x1f;
const char PTC_COMMAND_SET_DESTINATION_IP = 0x20;
const char PTC_COMMAND_SET_CONTROL_PORT = 0x21;
const char PTC_COMMAND_SET_LIDAR_RANGE = 0x22;
const char PTC_COMMAND_GET_LIDAR_RANGE = 0x23;
const char PTC_COMMAND_SET_PTP_CONFIG = 0x24;
const char PTC_COMMAND_RESET = 0x25;

const char PTC_COMMAND_GET_PTP_CONFIG = 0x26;
const char PTC_COMMAND_SET_INTERSTITIAL_POINTS = 0x32;
const char PTC_COMMAND_GET_INTERSTITIAL_POINTS = 0x33;
const char PTC_COMMAND_SET_REFLECTIVITY_MAPPING = 0x1d;
const char PTC_COMMAND_SET_RETRO_MULTI_REFLECTION = 0x47;

const char PTC_COMMAND_GET_RETRO_MULTI_REFLECTION = 0x48;
const char PTC_COMMAND_SET_WIDENING_FILTERING = 0x49;
const char PTC_COMMAND_GET_WIDENING_FILTERING = 0x4A;
const char PTC_COMMAND_SET_POPOUT_FILTERING = 0x4B;
const char PTC_COMMAND_GET_POPOUT_FILTERING = 0x4C;

template <char CmdId, typename ReqT, typename ResT>
class PtcCommand
{
};

struct IdentityParser
{
  template <typename T>
  static T parse(const T & value)
  {
    return value;
  }
};

namespace unit_tags {
  const char rpm[] = "RPM";
  const char volt[] = "V";
  const char ampere[] = "A";
  const char degrees[] = "º";
  const char celsius[] = "ºC";
}

template <const char * UnitLetters, size_t NSubdivisions = 1>
struct Unit {
    static auto parse(const int & value) {
        Unit<UnitLetters> unit;
        unit.value_ = value;
        return unit;
    }

private:
    int value_;
};

using Volt = Unit<unit_tags::volt>;
using Ampere = Unit<unit_tags::ampere>;
using Degrees = Unit<unit_tags::degrees>;
using Degree100ths = Unit<unit_tags::degrees, 100>;
using Celsius = Unit<unit_tags::celsius>;
using Celsius100ths = Unit<unit_tags::celsius, 100>;
using Rpm = Unit<unit_tags::rpm>;

template <typename FieldT, typename ParserT = IdentityParser>
struct ParsedField
{
  ParsedField() = default;

  auto get() const { return ParserT::parse(getFieldValue(value_)); }

private:
  /// @brief Field accessor for Boost endian buffers. Can be used to generically access fields across
  /// packet types.
  /// @return t.value()
  template <typename T>
  typename T::value_type getFieldValue(const T & t)
  {
    return t.value();
  }

  /// @brief Field accessor for primitive types. Can be used to generically access fields across
  /// packet types.
  /// @return t (as-is)
  template <typename T>
  std::enable_if_t<std::is_fundamental_v<T>, T> getFieldValue(const T & t)
  {
    return t;
  }

  /// @brief Field accessor for array types (assumed to char or uint8_t arrays).
  /// @return t (as-is)
  template <typename T>
  std::enable_if_t<std::is_array_v<T>, T> getFieldValue(const T & t) {
    return t;
  }

  FieldT value_;
};

using None = std::nullptr_t;

template <typename OutT>
struct ConstructParser
{
  template <typename InT>
  static auto parse(const InT & value)
  {
    return OutT(value);
  }
};

class GetLidarCalibration : public PtcCommand<PTC_COMMAND_GET_LIDAR_CALIBRATION, None, std::string>
{
};

enum class PtpQuery {
  PTP_STATUS = 0x01,
  PTP_TLV_PORT_DATA_SET = 0x02,
  PTP_TLV_TIME_STATUS_NP = 0x03,
  PTP_TLV_GRANDMASTER_SETTINGS_NP = 0x04
};

enum class PtpState {
  NONE = 0,
  INITIALIZING = 1,
  FAULTY = 2,
  DISABLED = 3,
  LISTENING = 4,
  PRE_MASER = 5,
  MASTER = 6,
  PASSIVE = 7,
  UNCALIBRATED = 8,
  SLAVE = 9,
  GRAND_MASTER = 10
};

struct PtpStatusResult
{
  ParsedField<int64_t, ConstructParser<nanoseconds>> master_offset;
  ParsedField<int32_t, ConstructParser<PtpState>> ptp_state;
  ParsedField<int32_t, ConstructParser<milliseconds>> elapsed_millisec;
};

struct TlvPortIdentity
{
  uint64_t clock_id;
  uint16_t port;
};

struct ExponentialSecondsParser
{
  template <typename InT>
  static nanoseconds parse(const InT & value)
  {
    uint64_t num_nanoseconds = static_cast<uint64_t>(1'000'000'000 * std::pow(2, value));
    return nanoseconds(num_nanoseconds);
  }
};

// These values need confirmation as they are not in the datasheet
enum class PtpDelayMechanism { P2P = 0, E2E = 1, AUTO = 2 };

struct PtpTlvPortDataSetResult
{
  ParsedField<uint8_t[10], ConstructParser<TlvPortIdentity>> port_identity;
  ParsedField<uint8_t, ConstructParser<PtpState>> port_state;
  ParsedField<int8_t, ExponentialSecondsParser> logMinDelayReqInterval;
  ParsedField<uint64_t, ConstructParser<nanoseconds>> peerMeanPathDelay;
  ParsedField<int8_t, ExponentialSecondsParser> logAnnounceInterval;
  ParsedField<uint8_t> announceReceiptTimeout;
  ParsedField<int8_t, ExponentialSecondsParser> logSyncInterval;
  ParsedField<uint8_t, ConstructParser<PtpDelayMechanism>> delayMechanism;
  ParsedField<int8_t, ExponentialSecondsParser> logMinPdelayReqInterval;
  ParsedField<uint8_t> versionNumber;
};

struct PtpTlvTimeStatusNpResult
{
  ParsedField<uint64_t, ConstructParser<nanoseconds>> master_offset;
  ParsedField<uint64_t> ingress_time;
  ParsedField<uint32_t> cumulativeScaledRateOffset;
  ParsedField<uint32_t> scaledLastGmPhaseChange;
  ParsedField<uint16_t> gmTimeBaseIndicator;
  ParsedField<uint8_t[12]> lastGmPhaseChange;
  ParsedField<uint32_t> gmPresent;
  ParsedField<uint64_t> gmIdentity;
};

struct PtpTlcGrandmasterSettingsNpResult
{
  ParsedField<uint32_t> clockQuality;
  ParsedField<uint16_t> utc_offset;
  ParsedField<uint8_t> time_flags;
  ParsedField<uint8_t> time_source;
};

using ptp_diagnostics_result_t = std::variant<
  PtpStatusResult, PtpTlvPortDataSetResult, PtpTlvTimeStatusNpResult,
  PtpTlcGrandmasterSettingsNpResult>;

class PtpDiagnostics
: public PtcCommand<PTC_COMMAND_PTP_DIAGNOSTICS, PtpQuery, ptp_diagnostics_result_t>
{
};

enum class MotorType { SINGLE_DIRECTION = 0, DUAL_DIRECTION = 1 };

enum class HesaiModel {
  PANDAR_40P = 0,
  PANDAR_64 = 2,
  PANDAR_128 = 3,
  PANDAR_QT = 15,
  PANDAR_40M = 17,
  PANDAR_XT = 19
};

template <size_t MaxLength>
class StringParser
{
  static std::string parse(const char * value)
  {
    auto length = strnlen(value, MaxLength);
    return std::string(value, length);
  }
};

struct GetInventoryInfoResult
{
  ParsedField<char[18], StringParser<18>> sn;
  ParsedField<char[16], StringParser<16>> date_of_manufacture;
  ParsedField<char[6], StringParser<6>> mac;
  ParsedField<char[16], StringParser<16>> sw_ver;
  ParsedField<char[16], StringParser<16>> hw_ver;
  ParsedField<char[16], StringParser<16>> control_fw_ver;
  ParsedField<char[16], StringParser<16>> sensor_fw_ver;
  ParsedField<big_uint16_buf_t, Degrees> angle_offset;
  ParsedField<uint8_t, ConstructParser<HesaiModel>> model;
  ParsedField<uint8_t, ConstructParser<MotorType>> motor_type;
  ParsedField<uint8_t> num_of_lines;
  ParsedField<uint8_t[11]> reserved;
};

class GetInventoryInfo
: public PtcCommand<PTC_COMMAND_GET_INVENTORY_INFO, None, GetInventoryInfoResult>
{
};

class IpAddressParser
{
  static std::string parse(const uint8_t value[4])
  {
    std::string result;
    for (size_t i = 0; i < 4; ++i) {
      if (i != 0) {
        result += '.';
      }
      result += std::to_string(static_cast<int>(value[i]));
    }
    return result;
  }
};

enum class ClockSource { GPS = 0, PTP = 1 };
enum class ClockDataFmt { GPRMC = 0, GPGGA = 1 };
enum class UdpSequence { OFF = 0, INCREMENT_WHILE_INSIDE_FOV = 1, ALWAYS_INCREMENT = 2 };
enum class TriggerMethod { ANGLE_BASED = 0, TIME_BASED = 1 };
enum class ReflectivityMapping { LINEAR = 0, NON_LINEAR = 1 };
enum class PtpClockStatus { FREE_RUN = 0, TRACKING = 1, LOCKED = 2, FROZEN = 3 };

struct MotorStatus
{
  bool can_reverse;
  bool is_currently_clockwise;

  static MotorStatus parse(const uint8_t & value)
  {
    MotorStatus ms;
    ms.can_reverse = value & 0x10;
    ms.is_currently_clockwise = !(value & 0x01);
    return ms;
  }
};

struct Rpm
{
  static Rpm parse(const int & value)
  {
    Rpm rpm;
    rpm.value_ = value;
    return rpm;
  }

private:
  int value_;
};

struct GetConfigInfoResult
{
  ParsedField<uint8_t[4], IpAddressParser> ipaddr;
  ParsedField<uint8_t[4], IpAddressParser> mask;
  ParsedField<uint8_t[4], IpAddressParser> gateway;
  ParsedField<uint8_t[4], IpAddressParser> dest_ipaddr;
  ParsedField<uint16_t> dest_LiDAR_udp_port;
  ParsedField<uint16_t> dest_gps_udp_port;
  ParsedField<uint16_t, Rpm> spin_rate;
  ParsedField<uint8_t, ConstructParser<bool>> sync;
  ParsedField<uint16_t, Degree100ths> sync_angle;   // in units of 0.01°
  ParsedField<uint16_t, Degree100ths> start_angle;  // Not used
  ParsedField<uint16_t, Degree100ths> stop_angle;   // Not used
  ParsedField<uint8_t, ConstructParser<ClockSource>> clock_source;
  ParsedField<uint8_t, ConstructParser<UdpSequence>> udp_seq;
  ParsedField<uint8_t, ConstructParser<TriggerMethod>> trigger_method;
  ParsedField<uint8_t> return_mode;
  ParsedField<uint8_t, ConstructParser<bool>> standby_mode;
  ParsedField<uint8_t, MotorStatus> motor_status;
  ParsedField<uint8_t, ConstructParser<bool>> vlan_flag;
  ParsedField<uint16_t> vlan_id;
  ParsedField<uint8_t, ConstructParser<ClockDataFmt>> clock_data_fmt;
  ParsedField<uint8_t, ConstructParser<bool>> noise_filtering;
  ParsedField<uint8_t, ConstructParser<ReflectivityMapping>> reflectivity_mapping;
  ParsedField<uint8_t[6]> reserved;
};

class GetConfigInfo : public PtcCommand<PTC_COMMAND_GET_CONFIG_INFO, None, GetConfigInfoResult>
{
};

struct GetLidarStatusResult
{
  ParsedField<uint32_t, ConstructParser<seconds>> system_uptime;
  ParsedField<uint16_t, Rpm> motor_speed;
  ParsedField<int32_t, Celsius100ths> temp_bottom_circuit_board_t1;
  ParsedField<int32_t, Celsius100ths> temp_bottom_circuit_board_t2;
  ParsedField<int32_t, Celsius100ths> temp_laser_emitting_board_rt_l1;
  ParsedField<int32_t, Celsius100ths> temp_laser_emitting_board_rt_l2;
  ParsedField<int32_t, Celsius100ths> temp_receiving_board_rt_r;
  ParsedField<int32_t, Celsius100ths> temp_receiving_board_rt2;
  ParsedField<int32_t, Celsius100ths> temp_top_circuit_rt3;
  ParsedField<int32_t, Celsius100ths> temp_reserved;
  ParsedField<uint8_t, ConstructParser<bool>> gps_pps_lock;
  ParsedField<uint8_t, ConstructParser<bool>> gps_gprmc_status;
  ParsedField<uint32_t> startup_times;
  ParsedField<uint8_t, ConstructParser<seconds>> total_operation_time; // TODO(mojomex): confirm this is seconds
  ParsedField<uint8_t, ConstructParser<PtpClockStatus>> ptp_clock_status;
  ParsedField<uint8_t[5]> reserved;
};

class GetLidarStatus : public PtcCommand<PTC_COMMAND_GET_LIDAR_STATUS, None, GetLidarStatusResult>
{
};

// Not implemented: PTC_COMMAND_SET_CLOCK_DATA_FMT

struct SetSpinRateRequest {
  ParsedField<uint16_t, ConstructParser<Rpm>> fmt;
};

class SetSpinRate : public PtcCommand<PTC_COMMAND_SET_SPIN_RATE, SetSpinRateRequest, None> {};

// Not implemented: PTC_COMMAND_SET_SYNC_ANGLE

struct SetTriggerMethodRequest {
  ParsedField<uint16_t, ConstructParser<TriggerMethod>> trigger_method;
};

class SetTriggerMethod : public PtcCommand<PTC_COMMAND_SET_TRIGGER_METHOD, TriggerMethod, None> {};

enum class ReturnMode {
  SINGLE_FIRST = 0x33,
  SINGLE_SECOND = 0x34,
  SINGLE_STRONGEST = 0x37,
  SINGLE_LAST = 0x38,
  DUAL_LAST_STRONGEST = 0x39,
  DUAL_FIRST_SECOND = 0x3a,
  DUAL_FIRST_LAST = 0x3b,
  DUAL_FIRST_STRONGEST = 0x3c,
  TRIPLE_FIRST_LAST_STRONGEST = 0x3d,
  DUAL_STRONGEST_SECONDSTRONGEST = 0x3,
};

struct SetReturnModeRequest {
  ParsedField<uint16_t, ConstructParser<ReturnMode>> return_mode;
};

class SetReturnMode : public PtcCommand<PTC_COMMAND_SET_RETURN_MODE, SetReturnModeRequest, None> {};

struct SetClockSourceRequest {
  ParsedField<uint16_t, ConstructParser<ClockSource>> clock_source;
};

class SetClockSource : public PtcCommand<PTC_COMMAND_SET_CLOCK_SOURCE, SetClockSourceRequest, None> {};

struct SetDestinationIpRequest {
  ParsedField<uint8_t[4], IpAddressParser> destination;
  ParsedField<uint16_t> port;
  ParsedField<uint16_t> gps_port;
};

class SetDestinationIp : public PtcCommand<PTC_COMMAND_SET_DESTINATION_IP, SetDestinationIpRequest, None> {};

struct SetControlPortRequest {
  ParsedField<uint8_t[4], IpAddressParser> ipv4;
  ParsedField<uint8_t[4], IpAddressParser> mask;
  ParsedField<uint8_t[4], IpAddressParser> gateway;
  ParsedField<uint8_t, ConstructParser<bool>> vlan_flag;
  ParsedField<uint16_t> vlan_id; // range [1, 4094]
};

class SetControlPort : public PtcCommand<PTC_COMMAND_SET_CONTROL_PORT, SetControlPortRequest, None> {};

// Not implemented: PTC_COMMAND_SET_LIDAR_RANGE
// Not implemented: PTC_COMMAND_GET_LIDAR_RANGE

enum class PtpProfile { IEEE_1588V2 = 0, IEEE_802_1_AS = 1 };

// TODO(mojomex)
struct SetPtpConfigRequest {
  ParsedField<uint8_t> profile;
  ParsedField<uint8_t> domain;
  ParsedField<uint8_t> network;
  ParsedField<uint8_t> logAnnounceInterval;
  ParsedField<uint8_t> logSyncInterval;
  ParsedField<uint8_t> logMinDelayReqInterval;
};

class SetPtpConfig : public PtcCommand<PTC_COMMAND_SET_PTP_CONFIG, SetPtpConfigRequest, None> {};

class Reset : public PtcCommand<PTC_COMMAND_RESET, None, None> {};

// Not implemented: PTC_COMMAND_GET_PTP_CONFIG

struct BoolRequest {
  ParsedField<uint8_t, ConstructParser<bool>> enable;
};

class SetInterstitialPointsFiltering : public PtcCommand<PTC_COMMAND_SET_INTERSTITIAL_POINTS, BoolRequest, None> {};
class GetInterstitialPointsFiltering : public PtcCommand<PTC_COMMAND_GET_INTERSTITIAL_POINTS, BoolRequest, None> {};

// Not implemented: PTC_COMMAND_SET_REFLECTIVITY_MAPPING

class SetRetroMultiReflectionFiltering : public PtcCommand<PTC_COMMAND_SET_RETRO_MULTI_REFLECTION, BoolRequest, None> {};
class GetRetroMultiReflectionFiltering : public PtcCommand<PTC_COMMAND_GET_RETRO_MULTI_REFLECTION, BoolRequest, None> {};

class SetWideningFiltering : public PtcCommand<PTC_COMMAND_SET_WIDENING_FILTERING, BoolRequest, None> {};
class GetWideningFiltering : public PtcCommand<PTC_COMMAND_GET_WIDENING_FILTERING, BoolRequest, None> {};

class SetPopoutFiltering : public PtcCommand<PTC_COMMAND_SET_POPOUT_FILTERING, BoolRequest, None> {};
class GetPopoutFiltering : public PtcCommand<PTC_COMMAND_GET_POPOUT_FILTERING, BoolRequest, None> {};

}  // namespace xt
