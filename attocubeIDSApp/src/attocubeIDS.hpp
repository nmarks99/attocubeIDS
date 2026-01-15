#pragma once
#include "json.hpp"
#include <array>
#include <asynPortDriver.h>
#include <iostream>
#include <optional>

using json = nlohmann::json;

namespace Method {
inline constexpr std::string_view AxisDisplacement = "com.attocube.ids.displacement.getAxisDisplacement";
inline constexpr std::string_view AxesDisplacement = "com.attocube.ids.displacement.getAxesDisplacement";
inline constexpr std::string_view AbsolutePosition = "com.attocube.ids.displacement.getAbsolutePosition";
inline constexpr std::string_view AbsolutePositions = "com.attocube.ids.displacement.getAbsolutePositions";
inline constexpr std::string_view ReferencePositions = "com.attocube.ids.displacement.getReferencePositions";
inline constexpr std::string_view MeasurementEnabled = "com.attocube.ids.displacement.getMeasurementEnabled";
}; // namespace Method

// asyn parameter names
inline constexpr char AXIS0_DISPLACEMENT_STR[] = "AXIS0_DISPLACEMENT";
inline constexpr char AXIS1_DISPLACEMENT_STR[] = "AXIS1_DISPLACEMENT";
inline constexpr char AXIS2_DISPLACEMENT_STR[] = "AXIS2_DISPLACEMENT";
inline constexpr char AXIS0_ABSOLUTE_POS_STR[] = "AXIS0_ABSOLUTE_POS";
inline constexpr char AXIS1_ABSOLUTE_POS_STR[] = "AXIS1_ABSOLUTE_POS";
inline constexpr char AXIS2_ABSOLUTE_POS_STR[] = "AXIS2_ABSOLUTE_POS";
inline constexpr char AXIS0_REFERENCE_POS_STR[] = "AXIS0_REFERENCE_POS";
inline constexpr char AXIS1_REFERENCE_POS_STR[] = "AXIS1_REFERENCE_POS";
inline constexpr char AXIS2_REFERENCE_POS_STR[] = "AXIS2_REFERENCE_POS";
inline constexpr char MEASUREMENT_ENABLED_STR[] = "MEASUREMENT_ENABLED";
inline constexpr char POLL_PERIOD_STR[] = "POLL_PERIOD";
inline constexpr char SUSPEND_POLLER_STR[] = "SUSPEND_POLLER";
inline constexpr char RESUME_POLLER_STR[] = "RESUME_POLLER";

inline constexpr size_t IO_BUFFER_SIZE = 512;
inline constexpr double IO_TIMEOUT = 1.0;
inline constexpr double POLL_PERIOD_MIN = 0.01;
inline constexpr size_t NUM_AXES = 3;

class AttocubeIDS : public asynPortDriver {
  public:
    AttocubeIDS(const char* conn_port, const char* driver_port);
    virtual void poll(void);
    virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    // virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

  private:
    asynUser* pasynUserDriver_;                   ///< Pointer to the asynUser for this driver.
    std::array<char, IO_BUFFER_SIZE> in_buffer_;  ///< Input data buffer (received from device).
    std::array<char, IO_BUFFER_SIZE> out_buffer_; ///< Output data buffer (sent to device).
    size_t nbytesout_ = 0;                        ///< Number of bytes sent on the last write operation.
    size_t nbytesin_ = 0;                         ///< Number of bytes received on the last read operation.
    int eom_reason_ = 0;                          ///< Reason for End of Message (EOM) on last read.
    epicsThreadId poller_thread_id_;              ///< Identifier for the background polling thread.
    bool poller_should_suspend_ = false;          ///< Flag to request suspension of the poller thread.

    // Some internal type aliases
    using I64Array3 = std::array<int64_t, 3>; ///< 3-element 64-bit integer array (e.g., axes displacement).
    using I64Array4 = std::array<int64_t, 4>; ///< 4-element 64-bit integer array.
    using IntPair = std::tuple<int, int>;     ///< Pair of integers for coordinate or status mapping.

    /// @brief Writes out_buffer_ to device and reads reply into in_buffer_
    ///
    /// @param write_len (Optional) Number of characters in out_buffer_ to write.
    /// @return asynStatus.
    asynStatus write_read(size_t write_len = IO_BUFFER_SIZE);

    /// @brief Constructs JSON-RPC formatted command, writes it to the device
    /// then reads the reply and attempts to parse it to json.
    ///
    /// @param method The JSON-RPC method to call
    /// @param params (Optional) A JSON object for parameters to pass.
    /// @return The parsed reply as a JSON object, std::nullopt on communication or parse error.
    std::optional<json> write_read_json(std::string_view method, json params = json{});

    /// @brief Sends a JSON-RPC command and attempts to parse the result into the requested type.
    ///
    /// @tparam T The expected return type of the RPC result.
    /// @param method The JSON-RPC method to call.
    /// @param params (Optional) A JSON object for parameters to pass.
    /// @return The parsed value of type T if successful, std::nullopt on communication or parse error.
    template <typename T>
    std::optional<T> do_rpc(std::string_view method, json params = json{}) {
        if (auto data = write_read_json(method); data) {
            if (data->contains("result")) {
                try {
                    return data.value()["result"].get<T>();
                } catch (...) {
                    return std::nullopt;
                }
            }
        }
        return std::nullopt;
    }

  protected:
    // Indices in asyn parameter library
    int resumePollerId_;
    int suspendPollerId_;
    int pollPeriodId_;
    int measurementEnabledId_;
    int axis0DisplacementId_;
    int axis1DisplacementId_;
    int axis2DisplacementId_;
    int axis0AbsolutePosId_;
    int axis1AbsolutePosId_;
    int axis2AbsolutePosId_;
    int axis0ReferencePosId_;
    int axis1ReferencePosId_;
    int axis2ReferencePosId_;
};
