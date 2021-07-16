// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY
//RPC code requires C++14. If build system like Unreal doesn't support it then use compiled binaries
#ifndef AIRLIB_NO_RPC
//if using Unreal Build system then include pre-compiled header file first

#include "vehicles/multirotor/api/MultirotorRpcLibServer.hpp"


#include "common/Common.hpp"
STRICT_MODE_OFF

#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "common/common_utils/MinWinDefines.hpp"
#undef NOUSER

#include "common/common_utils/WindowsApisCommonPre.hpp"
#undef FLOAT
#undef check
#include "rpc/server.h"
//TODO: HACK: UE4 defines macro with stupid names like "check" that conflicts with msgpack library
#ifndef check
#define check(expr) (static_cast<void>((expr)))
#endif
#include "common/common_utils/WindowsApisCommonPost.hpp"

#include "vehicles/multirotor/api/MultirotorRpcLibAdaptors.hpp"

STRICT_MODE_ON


namespace msr { namespace airlib {

typedef msr::airlib_rpclib::MultirotorRpcLibAdaptors MultirotorRpcLibAdaptors;

MultirotorRpcLibServer::MultirotorRpcLibServer(ApiProvider* api_provider, string server_address, uint16_t port)
        : RpcLibServerBase(api_provider, server_address, port)
{
    (static_cast<rpc::server*>(getServer()))->
        bind("takeoff", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->takeoff(timeout_sec); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("land", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->land(timeout_sec); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("goHome", [&](float timeout_sec, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->goHome(timeout_sec); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("dummyprinter", [&](float numerito, const std::string& vehicle_name) -> bool {
        return bool(getVehicleApi(vehicle_name)->dummyprinter(numerito));
            });

    // Methods related to the Camera data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setCameraActivation", [&](bool activation, int sample_rate, const std::vector<MultirotorRpcLibAdaptors::ImageRequest>& request_adapter, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setCameraActivation(activation, sample_rate, MultirotorRpcLibAdaptors::ImageRequest::to(request_adapter), getVehicleSimApi(vehicle_name));
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanCameraStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanCameraStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("saveCameraStoredData", [&](const std::string& path, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->saveCameraStoredData(path);
            });

    // Methods related to the IMU data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setImuActivation", [&](bool activation, int sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setImuActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanImuStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanImuStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getImuStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::IMUStoredData {
        return MultirotorRpcLibAdaptors::IMUStoredData(getVehicleApi(vehicle_name)->getImuStoredDataVec());
            });

    // Methods related to the barometer data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setBarometerActivation", [&](bool activation, int sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setBarometerActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanBarometerStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanBarometerStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getBarometerStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::BarometerStoredData {
        return MultirotorRpcLibAdaptors::BarometerStoredData(getVehicleApi(vehicle_name)->getBarometerStoredDataVec());
            });

    // Methods related to the magnetometer data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setMagnetometerActivation", [&](bool activation, int sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setMagnetometerActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanMagnetometerStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanMagnetometerStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getMagnetometerStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::MagnetometerStoredData {
        return MultirotorRpcLibAdaptors::MagnetometerStoredData(getVehicleApi(vehicle_name)->getMagnetometerStoredDataVec());
            });

    // Methods related to the GPS data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setGPSActivation", [&](bool activation, int sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setGPSActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanGPSStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanGPSStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getGPSStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::GPSStoredData {
        return MultirotorRpcLibAdaptors::GPSStoredData(getVehicleApi(vehicle_name)->getGPSStoredDataVec());
            });

    //Methods related to the drone failures
    (static_cast<rpc::server*>(getServer()))->
        bind("setDamageCoefficients", [&](float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setDamageCoefficients(new_coeff_1, new_coeff_2, new_coeff_3, new_coeff_4);
            });
    (static_cast<rpc::server*>(getServer()))->
        bind("setLockedProppellers", [&](bool locked_1, bool locked_2, bool locked_3, bool locked_4, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setLockedProppellers(locked_1, locked_2, locked_3, locked_4);
            });
    (static_cast<rpc::server*>(getServer()))->
        bind("setLockedPropellerCoefficients", [&](float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setLockedPropellerCoefficients(new_coeff_1, new_coeff_2, new_coeff_3, new_coeff_4);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getDamageCoefficients", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::DamageCoefficients {
        return MultirotorRpcLibAdaptors::DamageCoefficients(getVehicleApi(vehicle_name)->getDamageCoefficients());
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getLockedPropellers", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::LockedPropellers {
        return MultirotorRpcLibAdaptors::LockedPropellers(getVehicleApi(vehicle_name)->getLockedPropellers());
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getLockedPropellerCoefficients", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::DamageCoefficients {
        return MultirotorRpcLibAdaptors::DamageCoefficients(getVehicleApi(vehicle_name)->getLockedPropellerCoefficients());
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getMotorPWMs", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::DamageCoefficients {
        return MultirotorRpcLibAdaptors::DamageCoefficients(getVehicleApi(vehicle_name)->getMotorPWMs());
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocityBodyFrame", [&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain,
            const MultirotorRpcLibAdaptors::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveByVelocityBodyFrame(vx, vy, vz, duration, drivetrain, yaw_mode.to());
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocityZBodyFrame", [&](float vx, float vy, float z, float duration, DrivetrainType drivetrain,
            const MultirotorRpcLibAdaptors::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveByVelocityZBodyFrame(vx, vy, z, duration, drivetrain, yaw_mode.to());
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByMotorPWMs", [&](float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration, const std::string& vehicle_name) ->
        bool { return getVehicleApi(vehicle_name)->moveByMotorPWMs(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm, duration);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRollPitchYawZ", [&](float roll, float pitch, float yaw, float z, float duration, const std::string& vehicle_name) ->
        bool { return getVehicleApi(vehicle_name)->moveByRollPitchYawZ(roll, pitch, yaw, z, duration);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRollPitchYawThrottle", [&](float roll, float pitch, float yaw, float throttle, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByRollPitchYawThrottle(roll, pitch, yaw, throttle, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRollPitchYawrateThrottle", [&](float roll, float pitch, float yaw_rate, float throttle, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByRollPitchYawrateThrottle(roll, pitch, yaw_rate, throttle, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRollPitchYawrateZ", [&](float roll, float pitch, float yaw_rate, float z, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByRollPitchYawrateZ(roll, pitch, yaw_rate, z, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngleRatesZ", [&](float roll_rate, float pitch_rate, float yaw_rate, float z, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByAngleRatesZ(roll_rate, pitch_rate, yaw_rate, z, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByAngleRatesThrottle", [&](float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration, 
            const std::string& vehicle_name) -> bool { 
                return getVehicleApi(vehicle_name)->moveByAngleRatesThrottle(roll_rate, pitch_rate, yaw_rate, throttle, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocity", [&](float vx, float vy, float vz, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdaptors::YawMode& yaw_mode, const std::string& vehicle_name) -> bool { 
        return getVehicleApi(vehicle_name)->moveByVelocity(vx, vy, vz, duration, drivetrain, yaw_mode.to()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByVelocityZ", [&](float vx, float vy, float z, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdaptors::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
            return getVehicleApi(vehicle_name)->moveByVelocityZ(vx, vy, z, duration, drivetrain, yaw_mode.to()); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveOnPath", [&](const vector<MultirotorRpcLibAdaptors::Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const MultirotorRpcLibAdaptors::YawMode& yaw_mode,
        float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
            vector<Vector3r> conv_path;
            MultirotorRpcLibAdaptors::to(path, conv_path);
            return getVehicleApi(vehicle_name)->moveOnPath(conv_path, velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead);
        });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToPosition", [&](float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
        const MultirotorRpcLibAdaptors::YawMode& yaw_mode, float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveToPosition(x, y, z, velocity, timeout_sec, drivetrain, yaw_mode.to(), lookahead, adaptive_lookahead); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveToZ", [&](float z, float velocity, float timeout_sec, const MultirotorRpcLibAdaptors::YawMode& yaw_mode, 
            float lookahead, float adaptive_lookahead, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveToZ(z, velocity, timeout_sec, yaw_mode.to(), lookahead, adaptive_lookahead); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByManual", [&](float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, 
            const MultirotorRpcLibAdaptors::YawMode& yaw_mode, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->moveByManual(vx_max, vy_max, z_min, duration, drivetrain, yaw_mode.to()); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("rotateToYaw", [&](float yaw, float timeout_sec, float margin, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->rotateToYaw(yaw, timeout_sec, margin); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("rotateByYawRate", [&](float yaw_rate, float duration, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->rotateByYawRate(yaw_rate, duration); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("hover", [&](const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->hover(); 
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("setAngleLevelControllerGains", [&](const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setAngleLevelControllerGains(kp, ki, kd);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("setAngleRateControllerGains", [&](const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setAngleRateControllerGains(kp, ki, kd);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("setVelocityControllerGains", [&](const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setVelocityControllerGains(kp, ki, kd);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("setPositionControllerGains", [&](const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name) -> void {
            getVehicleApi(vehicle_name)->setPositionControllerGains(kp, ki, kd);
    });
    (static_cast<rpc::server*>(getServer()))->
        bind("moveByRC", [&](const MultirotorRpcLibAdaptors::RCData& data, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->moveByRC(data.to()); 
    });

    (static_cast<rpc::server*>(getServer()))->
        bind("setSafety", [&](uint enable_reasons, float obs_clearance, const SafetyEval::ObsAvoidanceStrategy& obs_startegy,
        float obs_avoidance_vel, const MultirotorRpcLibAdaptors::Vector3r& origin, float xy_length, 
            float max_z, float min_z, const std::string& vehicle_name) -> bool {
        return getVehicleApi(vehicle_name)->setSafety(SafetyEval::SafetyViolationType(enable_reasons), obs_clearance, obs_startegy,
            obs_avoidance_vel, origin.to(), xy_length, max_z, min_z); 
    });

    //getters
    // Rotor state
    (static_cast<rpc::server*>(getServer()))->
        bind("getRotorStates", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::RotorStates {
        return MultirotorRpcLibAdaptors::RotorStates(getVehicleApi(vehicle_name)->getRotorStates());
    });
    // Multirotor state
    (static_cast<rpc::server*>(getServer()))->
        bind("getMultirotorState", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::MultirotorState {
        return MultirotorRpcLibAdaptors::MultirotorState(getVehicleApi(vehicle_name)->getMultirotorState()); 
    });
}

//required for pimpl
MultirotorRpcLibServer::~MultirotorRpcLibServer()
{
}


}} //namespace


#endif
#endif
