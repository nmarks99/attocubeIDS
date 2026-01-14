#pragma once
#include "json.hpp"
#include <array>
#include <asynPortDriver.h>
#include <optional>

using json = nlohmann::json;

namespace Method {
inline constexpr std::string_view AxisDisplacement = "com.attocube.ids.displacement.getAxisDisplacement";
inline constexpr std::string_view AxesDisplacement = "com.attocube.ids.displacement.getAxesDisplacement";
inline constexpr std::string_view AbsolutePosition = "com.attocube.ids.displacement.getAbsolutePosition";
inline constexpr std::string_view AbsolutePositions = "com.attocube.ids.displacement.getAbsolutePositions";
};

inline constexpr char AXIS0_DISPLACEMENT_STR[] = "AXIS0_DISPLACEMENT";
inline constexpr char AXIS1_DISPLACEMENT_STR[] = "AXIS1_DISPLACEMENT";
inline constexpr char AXIS2_DISPLACEMENT_STR[] = "AXIS2_DISPLACEMENT";

inline constexpr size_t BUFFER_SIZE = 512;
inline constexpr double IO_TIMEOUT = 1.0;
inline constexpr size_t NUM_AXES = 3;

class AttocubeIDS : public asynPortDriver {
  public:
    AttocubeIDS(const char* conn_port, const char* driver_port);
    virtual void poll(void);
    // virtual asynStatus writeInt32(asynUser* pasynUser, epicsInt32 value);
    // virtual asynStatus writeFloat64(asynUser* pasynUser, epicsFloat64 value);

  private:
    asynUser* pasynUserDriver_; // pointer to the asynUser for this driver
    std::array<char, BUFFER_SIZE> in_buffer_;
    std::array<char, BUFFER_SIZE> out_buffer_;
    size_t nbytesout_ = 0;
    size_t nbytesin_ = 0;
    int eom_reason_ = 0;

    asynStatus write_read(size_t write_len = BUFFER_SIZE);

    std::optional<json> write_read_json(std::string_view method, json params);

    std::optional<std::array<int64_t, NUM_AXES>> get_displacements();

  protected:
    int axis0DisplacementId_;
    int axis1DisplacementId_;
    int axis2DisplacementId_;
};
