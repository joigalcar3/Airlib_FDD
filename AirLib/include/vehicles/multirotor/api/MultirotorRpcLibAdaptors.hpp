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

    // ----------------------------------------------------------------------------------------------------------------------------------
    // The next functions are used for the scoping of simulation data or the injection of in-flight actuator failure
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

    struct PosRefStoredData {
        std::vector<float> pos_ref_x;
        std::vector<float> pos_ref_y;
        std::vector<float> pos_ref_z;

        MSGPACK_DEFINE_MAP(pos_ref_x, pos_ref_y, pos_ref_z);

        PosRefStoredData()
        {}

        PosRefStoredData(msr::airlib::PosRefStoredData original)
        {
            pos_ref_x = original.pos_ref_x;
            pos_ref_y = original.pos_ref_y;
            pos_ref_z = original.pos_ref_z;
        }

        msr::airlib::PosRefStoredData to() const
        {
            msr::airlib::PosRefStoredData d(pos_ref_x, pos_ref_y, pos_ref_z);
            return d;
        }
    };

    struct PosErrorStoredData {
        std::vector<float> pos_error_x;
        std::vector<float> pos_error_y;
        std::vector<float> pos_error_z;

        MSGPACK_DEFINE_MAP(pos_error_x, pos_error_y, pos_error_z);

        PosErrorStoredData()
        {}

        PosErrorStoredData(msr::airlib::PosErrorStoredData original)
        {
            pos_error_x = original.pos_error_x;
            pos_error_y = original.pos_error_y;
            pos_error_z = original.pos_error_z;
        }

        msr::airlib::PosErrorStoredData to() const
        {
            msr::airlib::PosErrorStoredData d(pos_error_x, pos_error_y, pos_error_z);
            return d;
        }
    };

    struct PosErrorDotStoredData {
        std::vector<float> pos_error_dot_x;
        std::vector<float> pos_error_dot_y;
        std::vector<float> pos_error_dot_z;

        MSGPACK_DEFINE_MAP(pos_error_dot_x, pos_error_dot_y, pos_error_dot_z);

        PosErrorDotStoredData()
        {}

        PosErrorDotStoredData(msr::airlib::PosErrorDotStoredData original)
        {
            pos_error_dot_x = original.pos_error_dot_x;
            pos_error_dot_y = original.pos_error_dot_y;
            pos_error_dot_z = original.pos_error_dot_z;
        }

        msr::airlib::PosErrorDotStoredData to() const
        {
            msr::airlib::PosErrorDotStoredData d(pos_error_dot_x, pos_error_dot_y, pos_error_dot_z);
            return d;
        }
    };

    struct VelRefStoredData {
        std::vector<float> vel_ref_x;
        std::vector<float> vel_ref_y;
        std::vector<float> vel_ref_z;

        MSGPACK_DEFINE_MAP(vel_ref_x, vel_ref_y, vel_ref_z);

        VelRefStoredData()
        {}

        VelRefStoredData(msr::airlib::VelRefStoredData original)
        {
            vel_ref_x = original.vel_ref_x;
            vel_ref_y = original.vel_ref_y;
            vel_ref_z = original.vel_ref_z;
        }

        msr::airlib::VelRefStoredData to() const
        {
            msr::airlib::VelRefStoredData d(vel_ref_x, vel_ref_y, vel_ref_z);
            return d;
        }
    };

    struct VelStoredData {
        std::vector<float> vel_x;
        std::vector<float> vel_y;
        std::vector<float> vel_z;

        MSGPACK_DEFINE_MAP(vel_x, vel_y, vel_z);

        VelStoredData()
        {}

        VelStoredData(msr::airlib::VelStoredData original)
        {
            vel_x = original.vel_x;
            vel_y = original.vel_y;
            vel_z = original.vel_z;
        }

        msr::airlib::VelStoredData to() const
        {
            msr::airlib::VelStoredData d(vel_x, vel_y, vel_z);
            return d;
        }
    };

    struct AccRefStoredData {
        std::vector<float> acc_ref_x;
        std::vector<float> acc_ref_y;
        std::vector<float> acc_ref_z;

        MSGPACK_DEFINE_MAP(acc_ref_x, acc_ref_y, acc_ref_z);

        AccRefStoredData()
        {}

        AccRefStoredData(msr::airlib::AccRefStoredData original)
        {
            acc_ref_x = original.acc_ref_x;
            acc_ref_y = original.acc_ref_y;
            acc_ref_z = original.acc_ref_z;
        }

        msr::airlib::AccRefStoredData to() const
        {
            msr::airlib::AccRefStoredData d(acc_ref_x, acc_ref_y, acc_ref_z);
            return d;
        }
    };

    struct YawTransferFcnStoredData {
        std::vector<float> yaw_transfer_fcn_3;
        std::vector<float> yaw_transfer_fcn_1;
        std::vector<float> yaw_transfer_fcn_1_1;

        MSGPACK_DEFINE_MAP(yaw_transfer_fcn_3, yaw_transfer_fcn_1, yaw_transfer_fcn_1_1);

        YawTransferFcnStoredData()
        {}

        YawTransferFcnStoredData(msr::airlib::YawTransferFcnStoredData original)
        {
            yaw_transfer_fcn_3 = original.yaw_transfer_fcn_3;
            yaw_transfer_fcn_1 = original.yaw_transfer_fcn_1;
            yaw_transfer_fcn_1_1 = original.yaw_transfer_fcn_1_1;
        }

        msr::airlib::YawTransferFcnStoredData to() const
        {
            msr::airlib::YawTransferFcnStoredData d(yaw_transfer_fcn_3, yaw_transfer_fcn_1 , yaw_transfer_fcn_1_1);
            return d;
        }
    };

    struct PqrRefStoredData {
        std::vector<float> pqr_ref_x;
        std::vector<float> pqr_ref_y;
        std::vector<float> pqr_ref_z;

        MSGPACK_DEFINE_MAP(pqr_ref_x, pqr_ref_y, pqr_ref_z);

        PqrRefStoredData()
        {}

        PqrRefStoredData(msr::airlib::PqrRefStoredData original)
        {
            pqr_ref_x = original.pqr_ref_x;
            pqr_ref_y = original.pqr_ref_y;
            pqr_ref_z = original.pqr_ref_z;
        }

        msr::airlib::PqrRefStoredData to() const
        {
            msr::airlib::PqrRefStoredData d(pqr_ref_x, pqr_ref_y, pqr_ref_z);
            return d;
        }
    };

    struct PqrStoredData {
        std::vector<float> pqr_x;
        std::vector<float> pqr_y;
        std::vector<float> pqr_z;

        MSGPACK_DEFINE_MAP(pqr_x, pqr_y, pqr_z);

        PqrStoredData()
        {}

        PqrStoredData(msr::airlib::PqrStoredData original)
        {
            pqr_x = original.pqr_x;
            pqr_y = original.pqr_y;
            pqr_z = original.pqr_z;
        }

        msr::airlib::PqrStoredData to() const
        {
            msr::airlib::PqrStoredData d(pqr_x, pqr_y, pqr_z);
            return d;
        }
    };

    struct ThrustRefStoredData {
        std::vector<float> current_thrust_ref_fb;
        std::vector<float> current_thrust_ref_ff;

        MSGPACK_DEFINE_MAP(current_thrust_ref_fb, current_thrust_ref_ff);

        ThrustRefStoredData()
        {}

        ThrustRefStoredData(msr::airlib::ThrustRefStoredData original)
        {
            current_thrust_ref_fb = original.current_thrust_ref_fb;
            current_thrust_ref_ff = original.current_thrust_ref_ff;
        }

        msr::airlib::ThrustRefStoredData to() const
        {
            msr::airlib::ThrustRefStoredData d(current_thrust_ref_fb, current_thrust_ref_ff);
            return d;
        }
    };

    struct OmegasStoredData {
        std::vector<float> front_left;
        std::vector<float> front_right;
        std::vector<float> back_right;
        std::vector<float> back_left;

        MSGPACK_DEFINE_MAP(front_left, front_right, back_right, back_left);

        OmegasStoredData()
        {}

        OmegasStoredData(msr::airlib::OmegasStoredData original)
        {
            front_left = original.front_left;
            front_right = original.front_right;
            back_right = original.back_right;
            back_left = original.back_left;
        }

        msr::airlib::OmegasStoredData to() const
        {
            msr::airlib::OmegasStoredData d(front_left, front_right, back_right, back_left);
            return d;
        }
    };

    struct YawRefStoredData {
        std::vector<float> yaw_ref;
        std::vector<float> yaw_ref_corrected;

        MSGPACK_DEFINE_MAP(yaw_ref, yaw_ref_corrected);

        YawRefStoredData()
        {}

        YawRefStoredData(msr::airlib::YawRefStoredData original)
        {
            yaw_ref = original.yaw_ref;
            yaw_ref_corrected = original.yaw_ref_corrected;
        }

        msr::airlib::YawRefStoredData to() const
        {
            msr::airlib::YawRefStoredData d(yaw_ref, yaw_ref_corrected);
            return d;
        }
    };

    struct OrientationStoredData {
        std::vector<float> orientation_x;
        std::vector<float> orientation_y;
        std::vector<float> orientation_z;

        MSGPACK_DEFINE_MAP(orientation_x, orientation_y, orientation_z);

        OrientationStoredData()
        {}

        OrientationStoredData(msr::airlib::OrientationStoredData original)
        {
            orientation_x = original.orientation_x;
            orientation_y = original.orientation_y;
            orientation_z = original.orientation_z;
        }

        msr::airlib::OrientationStoredData to() const
        {
            msr::airlib::OrientationStoredData d(orientation_x, orientation_y, orientation_z);
            return d;
        }
    };

    struct PositionIntegratorStoredData {
        std::vector<float> position_integrator_x;
        std::vector<float> position_integrator_y;
        std::vector<float> position_integrator_z;

        MSGPACK_DEFINE_MAP(position_integrator_x, position_integrator_y, position_integrator_z);

        PositionIntegratorStoredData()
        {}

        PositionIntegratorStoredData(msr::airlib::PositionIntegratorStoredData original)
        {
            position_integrator_x = original.position_integrator_x;
            position_integrator_y = original.position_integrator_y;
            position_integrator_z = original.position_integrator_z;
        }

        msr::airlib::PositionIntegratorStoredData to() const
        {
            msr::airlib::PositionIntegratorStoredData d(position_integrator_x, position_integrator_y, position_integrator_z);
            return d;
        }
    };

    struct ThrustPiStoredData {
        std::vector<float> thrust_P;
        std::vector<float> thrust_I;

        MSGPACK_DEFINE_MAP(thrust_P, thrust_I);

        ThrustPiStoredData()
        {}

        ThrustPiStoredData(msr::airlib::ThrustPiStoredData original)
        {
            thrust_P = original.thrust_P;
            thrust_I = original.thrust_I;
        }

        msr::airlib::ThrustPiStoredData to() const
        {
            msr::airlib::ThrustPiStoredData d(thrust_P, thrust_I);
            return d;
        }
    };

    struct DamagedMassForcesStoredData {
        std::vector<float> damaged_mass_forces_x;
        std::vector<float> damaged_mass_forces_y;
        std::vector<float> damaged_mass_forces_z;

        MSGPACK_DEFINE_MAP(damaged_mass_forces_x, damaged_mass_forces_y, damaged_mass_forces_z);

        DamagedMassForcesStoredData()
        {}

        DamagedMassForcesStoredData(msr::airlib::DamagedMassForcesStoredData original)
        {
            damaged_mass_forces_x = original.damaged_mass_forces_x;
            damaged_mass_forces_y = original.damaged_mass_forces_y;
            damaged_mass_forces_z = original.damaged_mass_forces_z;
        }

        msr::airlib::DamagedMassForcesStoredData to() const
        {
            msr::airlib::DamagedMassForcesStoredData d(damaged_mass_forces_x, damaged_mass_forces_y, damaged_mass_forces_z);
            return d;
        }
    };

    struct DamagedMassMomentsStoredData {
        std::vector<float> damaged_mass_moments_x;
        std::vector<float> damaged_mass_moments_y;
        std::vector<float> damaged_mass_moments_z;

        MSGPACK_DEFINE_MAP(damaged_mass_moments_x, damaged_mass_moments_y, damaged_mass_moments_z);

        DamagedMassMomentsStoredData()
        {}

        DamagedMassMomentsStoredData(msr::airlib::DamagedMassMomentsStoredData original)
        {
            damaged_mass_moments_x = original.damaged_mass_moments_x;
            damaged_mass_moments_y = original.damaged_mass_moments_y;
            damaged_mass_moments_z = original.damaged_mass_moments_z;
        }

        msr::airlib::DamagedMassMomentsStoredData to() const
        {
            msr::airlib::DamagedMassMomentsStoredData d(damaged_mass_moments_x, damaged_mass_moments_y, damaged_mass_moments_z);
            return d;
        }
    };

    struct DamagedAeroForcesStoredData {
        std::vector<float> damaged_aero_forces_x;
        std::vector<float> damaged_aero_forces_y;
        std::vector<float> damaged_aero_forces_z;

        MSGPACK_DEFINE_MAP(damaged_aero_forces_x, damaged_aero_forces_y, damaged_aero_forces_z);

        DamagedAeroForcesStoredData()
        {}

        DamagedAeroForcesStoredData(msr::airlib::DamagedAeroForcesStoredData original)
        {
            damaged_aero_forces_x = original.damaged_aero_forces_x;
            damaged_aero_forces_y = original.damaged_aero_forces_y;
            damaged_aero_forces_z = original.damaged_aero_forces_z;
        }

        msr::airlib::DamagedAeroForcesStoredData to() const
        {
            msr::airlib::DamagedAeroForcesStoredData d(damaged_aero_forces_x, damaged_aero_forces_y, damaged_aero_forces_z);
            return d;
        }
    };

    struct DamagedAeroMomentsStoredData {
        std::vector<float> damaged_aero_moments_x;
        std::vector<float> damaged_aero_moments_y;
        std::vector<float> damaged_aero_moments_z;

        MSGPACK_DEFINE_MAP(damaged_aero_moments_x, damaged_aero_moments_y, damaged_aero_moments_z);

        DamagedAeroMomentsStoredData()
        {}

        DamagedAeroMomentsStoredData(msr::airlib::DamagedAeroMomentsStoredData original)
        {
            damaged_aero_moments_x = original.damaged_aero_moments_x;
            damaged_aero_moments_y = original.damaged_aero_moments_y;
            damaged_aero_moments_z = original.damaged_aero_moments_z;
        }

        msr::airlib::DamagedAeroMomentsStoredData to() const
        {
            msr::airlib::DamagedAeroMomentsStoredData d(damaged_aero_moments_x, damaged_aero_moments_y, damaged_aero_moments_z);
            return d;
        }
    };

    struct TimeInfoStoredData {
        std::vector<float> time;
        std::vector<float> dt_real;
        std::vector<float> sampling_frequency;

        MSGPACK_DEFINE_MAP(time, dt_real, sampling_frequency);

        TimeInfoStoredData()
        {}

        TimeInfoStoredData(msr::airlib::TimeInfoStoredData original)
        {
            time = original.time;
            dt_real = original.dt_real;
            sampling_frequency = original.sampling_frequency;
        }

        msr::airlib::TimeInfoStoredData to() const
        {
            msr::airlib::TimeInfoStoredData d(time, dt_real, sampling_frequency);
            return d;
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

    struct PWMStoredData {
        std::vector<float> PWM_1;
        std::vector<float> PWM_2;
        std::vector<float> PWM_3;
        std::vector<float> PWM_4;

        MSGPACK_DEFINE_MAP(PWM_1, PWM_2, PWM_3, PWM_4);

        PWMStoredData()
        {}

        PWMStoredData(msr::airlib::PWMStoredData original)
        {
            PWM_1 = original.PWM_1;
            PWM_2 = original.PWM_2;
            PWM_3 = original.PWM_3;
            PWM_4 = original.PWM_4;
        }

        msr::airlib::PWMStoredData to() const
        {
            msr::airlib::PWMStoredData d(PWM_1, PWM_2, PWM_3, PWM_4);
            return d;
        }
    };

    struct PositionStoredData {
        std::vector<float> positions_x;
        std::vector<float> positions_y;
        std::vector<float> positions_z;

        MSGPACK_DEFINE_MAP(positions_x, positions_y, positions_z);

        PositionStoredData()
        {}

        PositionStoredData(msr::airlib::PositionStoredData original)
        {
            positions_x = original.positions_x;
            positions_y = original.positions_y;
            positions_z = original.positions_z;
        }

        msr::airlib::PositionStoredData to() const
        {
            msr::airlib::PositionStoredData d(positions_x, positions_y, positions_z);
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
