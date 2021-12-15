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

    // Methods related to the general gathering of plot data
    (static_cast<rpc::server*>(getServer()))->
        bind("setPlotDataCollectionActivation", [&](bool activation, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPlotDataCollectionActivation(activation);
            });

    // Methods related to the reference position data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPosRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPosRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPosRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPosRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPosRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PosRefStoredData {
        return MultirotorRpcLibAdaptors::PosRefStoredData(getVehicleApi(vehicle_name)->getPosRefStoredDataVec());
            });

    // Methods related to the position error data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPosErrorActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPosErrorActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPosErrorStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPosErrorStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPosErrorStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PosErrorStoredData {
        return MultirotorRpcLibAdaptors::PosErrorStoredData(getVehicleApi(vehicle_name)->getPosErrorStoredDataVec());
            });

    // Methods related to the position error derivative data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPosErrorDotActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPosErrorDotActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPosErrorDotStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPosErrorDotStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPosErrorDotStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PosErrorDotStoredData {
        return MultirotorRpcLibAdaptors::PosErrorDotStoredData(getVehicleApi(vehicle_name)->getPosErrorDotStoredDataVec());
            });

    // Methods related to the reference velocity data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setVelRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setVelRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanVelRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanVelRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getVelRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::VelRefStoredData {
        return MultirotorRpcLibAdaptors::VelRefStoredData(getVehicleApi(vehicle_name)->getVelRefStoredDataVec());
            });

    // Methods related to the velocity data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setVelActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setVelActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanVelStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanVelStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getVelStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::VelStoredData {
        return MultirotorRpcLibAdaptors::VelStoredData(getVehicleApi(vehicle_name)->getVelStoredDataVec());
            });

    // Methods related to the reference acceleration data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setAccRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setAccRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanAccRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanAccRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getAccRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::AccRefStoredData {
        return MultirotorRpcLibAdaptors::AccRefStoredData(getVehicleApi(vehicle_name)->getAccRefStoredDataVec());
            });

    // Methods related to the yaw transfer function data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setYawTransferFcnActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setYawTransferFcnActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanYawTransferFcnStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanYawTransferFcnStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getYawTransferFcnStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::YawTransferFcnStoredData {
        return MultirotorRpcLibAdaptors::YawTransferFcnStoredData(getVehicleApi(vehicle_name)->getYawTransferFcnStoredDataVec());
            });

    // Methods related to the reference rotational rates data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPqrRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPqrRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPqrRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPqrRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPqrRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PqrRefStoredData {
        return MultirotorRpcLibAdaptors::PqrRefStoredData(getVehicleApi(vehicle_name)->getPqrRefStoredDataVec());
            });

    // Methods related to the rotational rates data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPqrActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPqrActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPqrStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPqrStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPqrStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PqrStoredData {
        return MultirotorRpcLibAdaptors::PqrStoredData(getVehicleApi(vehicle_name)->getPqrStoredDataVec());
            });

    // Methods related to the reference thrust data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setThrustRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setThrustRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanThrustRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanThrustRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getThrustRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::ThrustRefStoredData {
        return MultirotorRpcLibAdaptors::ThrustRefStoredData(getVehicleApi(vehicle_name)->getThrustRefStoredDataVec());
            });

    // Methods related to the omegas (motor rotations) data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setOmegasActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setOmegasActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanOmegasStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanOmegasStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getOmegasStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::OmegasStoredData {
        return MultirotorRpcLibAdaptors::OmegasStoredData(getVehicleApi(vehicle_name)->getOmegasStoredDataVec());
            });

    // Methods related to the reference yaw data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setYawRefActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setYawRefActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanYawRefStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanYawRefStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getYawRefStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::YawRefStoredData {
        return MultirotorRpcLibAdaptors::YawRefStoredData(getVehicleApi(vehicle_name)->getYawRefStoredDataVec());
            });

    // Methods related to the reference orientation data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setOrientationActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setOrientationActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanOrientationStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanOrientationStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getOrientationStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::OrientationStoredData {
        return MultirotorRpcLibAdaptors::OrientationStoredData(getVehicleApi(vehicle_name)->getOrientationStoredDataVec());
            });

    // Methods related to the position integrator data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPositionIntegratorActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPositionIntegratorActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPositionIntegratorStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPositionIntegratorStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPositionIntegratorStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PositionIntegratorStoredData {
        return MultirotorRpcLibAdaptors::PositionIntegratorStoredData(getVehicleApi(vehicle_name)->getPositionIntegratorStoredDataVec());
            });

    // Methods related to the thrust PI controller data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setThrustPiActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setThrustPiActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanThrustPiStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanThrustPiStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getThrustPiStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::ThrustPiStoredData {
        return MultirotorRpcLibAdaptors::ThrustPiStoredData(getVehicleApi(vehicle_name)->getThrustPiStoredDataVec());
            });

    // Methods related to the Camera data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setCameraActivation", [&](bool activation, float sample_rate, const std::vector<MultirotorRpcLibAdaptors::ImageRequest>& request_adapter, const std::string& vehicle_name) -> void {
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
        bind("setImuActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
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

    // Methods related to the PWMs data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPwmActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPWMActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPwmStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPWMStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPwmStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PWMStoredData {
        return MultirotorRpcLibAdaptors::PWMStoredData(getVehicleApi(vehicle_name)->getPWMStoredDataVec());
            });

    // Methods related to the ground truth position data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setPositionActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setPositionActivation(activation, sample_rate);
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("cleanPositionStoredData", [&](const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->cleanPositionStoredData();
            });

    (static_cast<rpc::server*>(getServer()))->
        bind("getPositionStoredDataVec", [&](const std::string& vehicle_name) -> MultirotorRpcLibAdaptors::PositionStoredData {
        return MultirotorRpcLibAdaptors::PositionStoredData(getVehicleApi(vehicle_name)->getPositionStoredDataVec());
            });

    // Methods related to the barometer data gathering
    (static_cast<rpc::server*>(getServer()))->
        bind("setBarometerActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
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
        bind("setMagnetometerActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
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
        bind("setGPSActivation", [&](bool activation, float sample_rate, const std::string& vehicle_name) -> void {
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

    // Methods related to the drone teleportation
    (static_cast<rpc::server*>(getServer()))->
        bind("setTeleportYawRef", [&](float yaw_angle_ref, const std::string& vehicle_name) -> void {
        getVehicleApi(vehicle_name)->setTeleportYawRef(yaw_angle_ref);
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
