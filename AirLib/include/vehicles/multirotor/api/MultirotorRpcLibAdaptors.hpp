// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibAdaptors_hpp
#define air_MultirotorRpcLibAdaptors_hpp

#include "common/Common.hpp"
#include "common/CommonStructs.hpp"
#include "api/RpcLibAdaptorsBase.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "common/ImageCaptureBase.hpp"
#include "safety/SafetyEval.hpp"

#include "common/common_utils/WindowsApisCommonPre.hpp"
#include "rpc/msgpack.hpp"
#include "common/common_utils/WindowsApisCommonPost.hpp"

namespace msr { namespace airlib_rpclib {

class MultirotorRpcLibAdaptors : public RpcLibAdaptorsBase {
public:
    struct YawMode {
        bool is_rate = true;
        float yaw_or_rate = 0;
        MSGPACK_DEFINE_MAP(is_rate, yaw_or_rate);
    
        YawMode()
        {}

        YawMode(const msr::airlib::YawMode& s)
        {
            is_rate = s.is_rate;
            yaw_or_rate = s.yaw_or_rate;
        }
        msr::airlib::YawMode to() const
        {
            return msr::airlib::YawMode(is_rate, yaw_or_rate);
        }
    };

    struct RotorParameters {
        msr::airlib::real_T thrust;
        msr::airlib::real_T torque_scaler;
        msr::airlib::real_T speed;

        MSGPACK_DEFINE_MAP(thrust, torque_scaler, speed);

        RotorParameters()
        {}

        RotorParameters(const msr::airlib::RotorParameters& s)
        {
            thrust = s.thrust;
            torque_scaler = s.torque_scaler;
            speed = s.speed;
        }

        msr::airlib::RotorParameters to() const
        {
            return msr::airlib::RotorParameters(thrust, torque_scaler, speed);
        }
    };

    struct RotorStates {
        std::vector<RotorParameters> rotors;
        uint64_t timestamp;

        MSGPACK_DEFINE_MAP(rotors, timestamp);

        RotorStates()
        {}

        RotorStates(const msr::airlib::RotorStates& s)
        {
            for (const auto& r : s.rotors)
            {
                rotors.push_back(RotorParameters(r));
            }
            timestamp = s.timestamp;
        }

        msr::airlib::RotorStates to() const
        {
            std::vector<msr::airlib::RotorParameters> d;
            for (const auto& r : rotors)
            {
                d.push_back(r.to());
            }
            return msr::airlib::RotorStates(d, timestamp);
        }
    };

    struct MultirotorState {
        CollisionInfo collision;
        KinematicsState kinematics_estimated;
        KinematicsState kinematics_true;
        GeoPoint gps_location;
        uint64_t timestamp;
        LandedState landed_state;
        RCData rc_data;
        bool ready;
        std::string ready_message;
        std::vector<std::string> controller_messages;
        bool can_arm;

        MSGPACK_DEFINE_MAP(collision, kinematics_estimated, gps_location, timestamp, landed_state, rc_data);

        MultirotorState()
        {}

        MultirotorState(const msr::airlib::MultirotorState& s)
        {
            collision = s.collision;
            kinematics_estimated = s.kinematics_estimated;
            gps_location = s.gps_location;
            timestamp = s.timestamp;
            landed_state = s.landed_state;
            rc_data = RCData(s.rc_data);
            ready = s.ready;
            ready_message = s.ready_message;
            can_arm = s.can_arm;
        }

        msr::airlib::MultirotorState to() const
        {
            return msr::airlib::MultirotorState(collision.to(), kinematics_estimated.to(), 
                gps_location.to(), timestamp, landed_state, rc_data.to(), ready, ready_message, can_arm);
        }
    };

    struct DamageCoefficients
    {
        float front_right;
        float back_left;
        float front_left;
        float back_right;
        MSGPACK_DEFINE_MAP(front_right, back_left, front_left, back_right);

        DamageCoefficients()
        {}

        DamageCoefficients(const msr::airlib::DamageCoefficients& dc)
        {
            front_right = dc.front_right;
            back_left = dc.back_left;
            front_left = dc.front_left;
            back_right = dc.back_right;
        }

        msr::airlib::DamageCoefficients to() const
        {
            return msr::airlib::DamageCoefficients(front_right, back_left, front_left, back_right);
        }
    };

    struct LockedPropellers
    {
        bool front_right;
        bool back_left;
        bool front_left;
        bool back_right;
        MSGPACK_DEFINE_MAP(front_right, back_left, front_left, back_right);

        LockedPropellers()
        {}

        LockedPropellers(const msr::airlib::LockedPropellers& dc)
        {
            front_right = dc.front_right;
            back_left = dc.back_left;
            front_left = dc.front_left;
            back_right = dc.back_right;
        }

        msr::airlib::LockedPropellers to() const
        {
            return msr::airlib::LockedPropellers(front_right, back_left, front_left, back_right);
        }
    };

    struct IMUStoredData {
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

        MSGPACK_DEFINE_MAP(timestamps, orientations_w, orientations_x, orientations_y, orientations_z, angular_velocity_x, angular_velocity_y, angular_velocity_z,
            linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);

        IMUStoredData()
        {}

        IMUStoredData(msr::airlib::IMUStoredData original)
        {
            timestamps = original.timestamps;
            orientations_w = original.orientations_w;
            orientations_x = original.orientations_x;
            orientations_y = original.orientations_y;
            orientations_z = original.orientations_z;

            angular_velocity_x = original.angular_velocity_x;
            angular_velocity_y = original.angular_velocity_y;
            angular_velocity_z = original.angular_velocity_z;

            linear_acceleration_x = original.linear_acceleration_x;
            linear_acceleration_y = original.linear_acceleration_y;
            linear_acceleration_z = original.linear_acceleration_z;
        }

        msr::airlib::IMUStoredData to() const
        {
            msr::airlib::IMUStoredData d(timestamps, orientations_w, orientations_x, orientations_y, orientations_z, angular_velocity_x,
                angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);
            return d;
        }
    };

    struct BarometerStoredData {
        std::vector<uint64_t> timestamps;
        std::vector<float> altitudes;
        std::vector<float> pressures;
        std::vector<float> qnhs;

        MSGPACK_DEFINE_MAP(timestamps, altitudes, pressures, qnhs);

        BarometerStoredData()
        {}

        BarometerStoredData(msr::airlib::BarometerStoredData original)
        {
            timestamps = original.timestamps;
            altitudes = original.altitudes;
            pressures = original.pressures;
            qnhs = original.qnhs;
        }

        msr::airlib::BarometerStoredData to() const
        {
            msr::airlib::BarometerStoredData d(timestamps, altitudes, pressures, qnhs);
            return d;
        }
    };

    struct MagnetometerStoredData {
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

        MSGPACK_DEFINE_MAP(timestamps, magnetic_field_body_x, magnetic_field_body_y, magnetic_field_body_z, magnetic_field_covariance_1,
            magnetic_field_covariance_2, magnetic_field_covariance_3, magnetic_field_covariance_4, magnetic_field_covariance_5, magnetic_field_covariance_6,
            magnetic_field_covariance_7, magnetic_field_covariance_8, magnetic_field_covariance_9);

        MagnetometerStoredData()
        {}

        MagnetometerStoredData(msr::airlib::MagnetometerStoredData original)
        {
            timestamps = original.timestamps;
            magnetic_field_body_x = original.magnetic_field_body_x;
            magnetic_field_body_y = original.magnetic_field_body_y;
            magnetic_field_body_z = original.magnetic_field_body_z;
            magnetic_field_covariance_1 = original.magnetic_field_covariance_1;
            magnetic_field_covariance_2 = original.magnetic_field_covariance_2;
            magnetic_field_covariance_3 = original.magnetic_field_covariance_3;
            magnetic_field_covariance_4 = original.magnetic_field_covariance_4;
            magnetic_field_covariance_5 = original.magnetic_field_covariance_5;
            magnetic_field_covariance_6 = original.magnetic_field_covariance_6;
            magnetic_field_covariance_7 = original.magnetic_field_covariance_7;
            magnetic_field_covariance_8 = original.magnetic_field_covariance_8;
            magnetic_field_covariance_9 = original.magnetic_field_covariance_9;
        }

        msr::airlib::MagnetometerStoredData to() const
        {
            msr::airlib::MagnetometerStoredData d(timestamps, magnetic_field_body_x, magnetic_field_body_y, magnetic_field_body_z, magnetic_field_covariance_1,
                magnetic_field_covariance_2, magnetic_field_covariance_3, magnetic_field_covariance_4, magnetic_field_covariance_5, magnetic_field_covariance_6,
                magnetic_field_covariance_7, magnetic_field_covariance_8, magnetic_field_covariance_9);
            return d;
        }
    };

    struct GPSStoredData {
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

        MSGPACK_DEFINE_MAP(timestamps, geo_point_altitude, geo_point_latitude, geo_point_longitude, ephs,
            epvs, velocity_x, velocity_y, velocity_z, fix_types, time_utcs, is_valids);

        GPSStoredData()
        {}

        GPSStoredData(msr::airlib::GPSStoredData original)
        {
            timestamps = original.timestamps;
            geo_point_altitude = original.geo_point_altitude;
            geo_point_latitude = original.geo_point_latitude;
            geo_point_longitude = original.geo_point_longitude;
            ephs = original.ephs;
            epvs = original.epvs;
            velocity_x = original.velocity_x;
            velocity_y = original.velocity_y;
            velocity_z = original.velocity_z;
            fix_types = original.fix_types;
            time_utcs = original.time_utcs;
            is_valids = original.is_valids;
        }

        msr::airlib::GPSStoredData to() const
        {
            msr::airlib::GPSStoredData d(timestamps, geo_point_altitude, geo_point_latitude, geo_point_longitude, ephs,
                epvs, velocity_x, velocity_y, velocity_z, fix_types, time_utcs, is_valids);
            return d;
        }
    };
    struct ImageRequest {
        std::string camera_name;
        msr::airlib::ImageCaptureBase::ImageType image_type;
        bool pixels_as_float;
        bool compress;

        MSGPACK_DEFINE_MAP(camera_name, image_type, pixels_as_float, compress);

        ImageRequest()
        {}

        ImageRequest(const msr::airlib::ImageCaptureBase::ImageRequest& s)
        {
            camera_name = s.camera_name;
            image_type = s.image_type;
            pixels_as_float = s.pixels_as_float;
            compress = s.compress;
        }

        msr::airlib::ImageCaptureBase::ImageRequest to() const
        {
            msr::airlib::ImageCaptureBase::ImageRequest d;
            d.camera_name = camera_name;
            d.image_type = image_type;
            d.pixels_as_float = pixels_as_float;
            d.compress = compress;

            return d;
        }

        static std::vector<ImageRequest> from(
            const std::vector<msr::airlib::ImageCaptureBase::ImageRequest>& request
        ) {
            std::vector<ImageRequest> request_adaptor;
            for (const auto& item : request)
                request_adaptor.push_back(ImageRequest(item));

            return request_adaptor;
        }
        static std::vector<msr::airlib::ImageCaptureBase::ImageRequest> to(
            const std::vector<ImageRequest>& request_adapter
        ) {
            std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request;
            for (const auto& item : request_adapter)
                request.push_back(item.to());

            return request;
        }
    };
};

}} //namespace

MSGPACK_ADD_ENUM(msr::airlib::DrivetrainType);
MSGPACK_ADD_ENUM(msr::airlib::LandedState);


#endif