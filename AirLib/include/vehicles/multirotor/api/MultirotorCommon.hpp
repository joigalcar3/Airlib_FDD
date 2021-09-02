#ifndef air_DroneCommon_hpp
#define air_DroneCommon_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "physics/Kinematics.hpp"

namespace msr {  namespace airlib {

enum class DrivetrainType {
    MaxDegreeOfFreedom = 0,
    ForwardOnly
};

enum class LandedState : uint {
    Landed = 0,
    Flying = 1
};
// Structs for rotor state API
struct RotorParameters {
    real_T thrust = 0;
    real_T torque_scaler = 0;
    real_T speed = 0;

    RotorParameters()
    {}

    RotorParameters(const real_T& thrust_val, const real_T& torque_scaler_val, const real_T& speed_val)
        : thrust(thrust_val), torque_scaler(torque_scaler_val), speed(speed_val)
    {
    }

    void update(const real_T& thrust_val, const real_T& torque_scaler_val, const real_T& speed_val)
    {
        thrust = thrust_val;
        torque_scaler = torque_scaler_val;
        speed = speed_val;
    }
};

struct RotorStates {
    std::vector<RotorParameters> rotors;
    uint64_t timestamp;

    RotorStates()
    {}
    RotorStates(const std::vector<RotorParameters>& rotors_val, uint64_t timestamp_val)
        : rotors(rotors_val), timestamp(timestamp_val)
    {
    }
};

//Yaw mode specifies if yaw should be set as angle or angular velocity around the center of drone
struct YawMode {
    bool is_rate = true;
    float yaw_or_rate = 0.0f;

    YawMode()
    {}

    YawMode(bool is_rate_val, float yaw_or_rate_val)
    {
        is_rate = is_rate_val;
        yaw_or_rate = yaw_or_rate_val;
    }

    static YawMode Zero()
    {
        return YawMode(true, 0);
    }

    void setZeroRate()
    {
        is_rate = true;
        yaw_or_rate = 0;
    }
};

//properties of vehicle
struct MultirotorApiParams {
    MultirotorApiParams() {};
    //what is the breaking distance for given velocity?
    //Below is just proportionality constant to convert from velocity to breaking distance
    float vel_to_breaking_dist = 0.5f;   //ideally this should be 2X for very high speed but for testing we are keeping it 0.5
    float min_breaking_dist = 1; //min breaking distance
    float max_breaking_dist = 3; //min breaking distance
    float breaking_vel = 1.0f;
    float min_vel_for_breaking = 3;

    //what is the differential positional accuracy of cur_loc?
    //this is not same as GPS accuracy because translational errors
    //usually cancel out. Typically this would be 0.2m or less
    float distance_accuracy = 0.1f;

    //what is the minimum clearance from obstacles?
    float obs_clearance = 2;

    //what is the +/-window we should check on obstacle map?
    //for example 2 means check from ticks -2 to 2
    int obs_window = 0;
};

struct MultirotorState {
    CollisionInfo collision;
    Kinematics::State kinematics_estimated;
    GeoPoint gps_location;
    uint64_t timestamp;
    LandedState landed_state;
    RCData rc_data;
    bool ready;  // indicates drone is ready for commands
    std::string ready_message;  // can show error message if drone is not reachable over the network or is not responding
    bool can_arm;  // indicates drone is ready to be armed

    MultirotorState()
    {}
    MultirotorState(const CollisionInfo& collision_val, const Kinematics::State& kinematics_estimated_val, 
        const GeoPoint& gps_location_val, uint64_t timestamp_val,
        LandedState landed_state_val, const RCData& rc_data_val, bool ready_val, const std::string& message, bool can_arm_val)
        : collision(collision_val), kinematics_estimated(kinematics_estimated_val),
        gps_location(gps_location_val), timestamp(timestamp_val),
        landed_state(landed_state_val), rc_data(rc_data_val), ready(ready_val), ready_message(message), can_arm(can_arm_val)
    {
    }

    //shortcuts
    const Vector3r& getPosition() const
    {
        return kinematics_estimated.pose.position;
    }
    const Quaternionr& getOrientation() const
    {
        return kinematics_estimated.pose.orientation;
    }
};

struct DamageCoefficients
{
    float front_right;
    float back_left;
    float front_left;
    float back_right;
    DamageCoefficients()
    {}

    DamageCoefficients(float fr, float bl, float fl, float br) : front_right(fr), back_left(bl), front_left(fl), back_right(br)
    {
    }
};

struct LockedPropellers
{
    bool front_right;
    bool back_left;
    bool front_left;
    bool back_right;
    LockedPropellers()
    {}

    LockedPropellers(bool fr, bool bl, bool fl, bool br) : front_right(fr), back_left(bl), front_left(fl), back_right(br)
    {
    }
};

struct IMUStoredData
{
    std::vector<uint64_t> timestamps;
    std::vector<float> orientations_w;
    std::vector<float> orientations_x;
    std::vector<float> orientations_y;
    std::vector<float> orientations_z;

    std::vector<float> angular_velocity_x;
    std::vector<float> angular_velocity_y;
    std::vector<float> angular_velocity_z;

    std::vector<float> linear_acceleration_x;
    std::vector<float> linear_acceleration_y;
    std::vector<float> linear_acceleration_z;
    IMUStoredData()
    {}

    IMUStoredData(const std::vector<uint64_t>& t, const std::vector<float>& ow, const std::vector<float>& ox, const std::vector<float>& oy, const std::vector<float>& oz,
        const std::vector<float>& avx, const std::vector<float>& avy, const std::vector<float>& avz, const std::vector<float>& lax, const std::vector<float>& lay, const std::vector<float>& laz)
    {
        timestamps = t;
        orientations_w = ow;
        orientations_x = ox;
        orientations_y = oy;
        orientations_z = oz;

        angular_velocity_x = avx;
        angular_velocity_y = avy;
        angular_velocity_z = avz;

        linear_acceleration_x = lax;
        linear_acceleration_y = lay;
        linear_acceleration_z = laz;
    }
};

struct PositionStoredData
{
    std::vector<float> positions_x;
    std::vector<float> positions_y;
    std::vector<float> positions_z;
    PositionStoredData()
    {}

    PositionStoredData(const std::vector<float>& px, const std::vector<float>& py, const std::vector<float>& pz)
    {
        positions_x = px;
        positions_y = py;
        positions_z = pz;
    }
};

struct PWMStoredData
{
    std::vector<float> PWM_1;
    std::vector<float> PWM_2;
    std::vector<float> PWM_3;
    std::vector<float> PWM_4;
    PWMStoredData()
    {}

    PWMStoredData(const std::vector<float>& p1, const std::vector<float>& p2, const std::vector<float>& p3, const std::vector<float>& p4)
    {
        PWM_1 = p1;
        PWM_2 = p2;
        PWM_3 = p3;
        PWM_4 = p4;
    }
};

struct BarometerStoredData
{
    std::vector<uint64_t> timestamps;
    std::vector<float> altitudes;
    std::vector<float> pressures;
    std::vector<float> qnhs;

    BarometerStoredData()
    {}

    BarometerStoredData(const std::vector<uint64_t>& t, const std::vector<float>& h, const std::vector<float>& p, const std::vector<float>& q)
    {
        timestamps = t;
        altitudes = h;
        pressures = p;
        qnhs = q;
    }
};

struct MagnetometerStoredData
{
    std::vector<uint64_t> timestamps;
    std::vector<float> magnetic_field_body_x;
    std::vector<float> magnetic_field_body_y;
    std::vector<float> magnetic_field_body_z;
    std::vector<float> magnetic_field_covariance_1;
    std::vector<float> magnetic_field_covariance_2;
    std::vector<float> magnetic_field_covariance_3;
    std::vector<float> magnetic_field_covariance_4;
    std::vector<float> magnetic_field_covariance_5;
    std::vector<float> magnetic_field_covariance_6;
    std::vector<float> magnetic_field_covariance_7;
    std::vector<float> magnetic_field_covariance_8;
    std::vector<float> magnetic_field_covariance_9;

    MagnetometerStoredData()
    {}

    MagnetometerStoredData(const std::vector<uint64_t>& t, const std::vector<float>& mfbx, const std::vector<float>& mfby, const std::vector<float>& mfbz, const std::vector<float>& mfc1, 
        const std::vector<float>& mfc2, const std::vector<float>& mfc3, const std::vector<float>& mfc4, const std::vector<float>& mfc5, const std::vector<float>& mfc6,
        const std::vector<float>& mfc7, const std::vector<float>& mfc8, const std::vector<float>& mfc9)
    {
        timestamps = t;
        magnetic_field_body_x = mfbx;
        magnetic_field_body_y = mfby;
        magnetic_field_body_z = mfbz;
        magnetic_field_covariance_1 = mfc1;
        magnetic_field_covariance_2 = mfc2;
        magnetic_field_covariance_3 = mfc3;
        magnetic_field_covariance_4 = mfc4;
        magnetic_field_covariance_5 = mfc5;
        magnetic_field_covariance_6 = mfc6;
        magnetic_field_covariance_7 = mfc7;
        magnetic_field_covariance_8 = mfc8;
        magnetic_field_covariance_9 = mfc9;
    }
};

struct GPSStoredData
{
    std::vector<uint64_t> timestamps;
    std::vector<float> geo_point_altitude;
    std::vector<double> geo_point_latitude;
    std::vector<double> geo_point_longitude;
    std::vector<float> ephs;
    std::vector<float> epvs;
    std::vector<float> velocity_x;
    std::vector<float> velocity_y;
    std::vector<float> velocity_z;
    std::vector<int> fix_types;
    std::vector<uint64_t> time_utcs;
    std::vector<bool> is_valids;

    GPSStoredData()
    {}

    GPSStoredData(const std::vector<uint64_t>& t, const std::vector<float>& gph, const std::vector<double>& gplat, const std::vector<double>& gplon, const std::vector<float>& eph,
        const std::vector<float>& epv, const std::vector<float>& vx, const std::vector<float>& vy, const std::vector<float>& vz, const std::vector<int>& ft,
        const std::vector<uint64_t>& tutc, const std::vector<bool>& v)
    {
        timestamps = t;
        geo_point_altitude = gph;
        geo_point_latitude = gplat;
        geo_point_longitude = gplon;
        ephs = eph;
        epvs = epv;
        velocity_x = vx;
        velocity_y = vy;
        velocity_z = vz;
        fix_types = ft;
        time_utcs = tutc;
        is_valids = v;
    }
};

}} //namespace
#endif