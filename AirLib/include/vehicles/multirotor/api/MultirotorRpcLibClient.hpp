// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_MultirotorRpcLibClient_hpp
#define air_MultirotorRpcLibClient_hpp

#include "common/Common.hpp"
#include <functional>
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "api/RpcLibClientBase.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"


namespace msr { namespace airlib {

class MultirotorRpcLibClient : public RpcLibClientBase {
public:
    MultirotorRpcLibClient(const string& ip_address = "localhost", uint16_t port = RpcLibPort, float timeout_sec = 60);

    MultirotorRpcLibClient* takeoffAsync(float timeout_sec = 20, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* landAsync(float timeout_sec = 60, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* goHomeAsync(float timeout_sec = Utils::max<float>(), const std::string& vehicle_name = "");
    bool dummyprinter(float numerito, const std::string& vehicle_name = "");

    // Methods related to the reference position data gathering
    void setPosRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPosRefStoredData(const std::string& vehicle_name = "");
    PosRefStoredData getPosRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the position error data gathering
    void setPosErrorActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPosErrorStoredData(const std::string& vehicle_name = "");
    PosErrorStoredData getPosErrorStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the position error derivative data gathering
    void setPosErrorDotActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPosErrorDotStoredData(const std::string& vehicle_name = "");
    PosErrorDotStoredData getPosErrorDotStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference velocity data gathering
    void setVelRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanVelRefStoredData(const std::string& vehicle_name = "");
    VelRefStoredData getVelRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the velocity data gathering
    void setVelActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanVelStoredData(const std::string& vehicle_name = "");
    VelStoredData getVelStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference acceleration data gathering
    void setAccRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanAccRefStoredData(const std::string& vehicle_name = "");
    AccRefStoredData getAccRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference rotational rates data gathering
    void setPqrRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPqrRefStoredData(const std::string& vehicle_name = "");
    PqrRefStoredData getPqrRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the rotational rates data gathering
    void setPqrActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPqrStoredData(const std::string& vehicle_name = "");
    PqrStoredData getPqrStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference thrust data gathering
    void setThrustRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanThrustRefStoredData(const std::string& vehicle_name = "");
    ThrustRefStoredData getThrustRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the omegas (motor rotations) data gathering
    void setOmegasActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanOmegasStoredData(const std::string& vehicle_name = "");
    OmegasStoredData getOmegasStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference yaw data gathering
    void setYawRefActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanYawRefStoredData(const std::string& vehicle_name = "");
    YawRefStoredData getYawRefStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the reference orientation data gathering
    void setOrientationActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanOrientationStoredData(const std::string& vehicle_name = "");
    OrientationStoredData getOrientationStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the position integrator data gathering
    void setPositionIntegratorActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPositionIntegratorStoredData(const std::string& vehicle_name = "");
    PositionIntegratorStoredData getPositionIntegratorStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the thrust PI controller data gathering
    void setThrustPiActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanThrustPiStoredData(const std::string& vehicle_name = "");
    ThrustPiStoredData getThrustPiStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the Camera data gathering
    void setCameraActivation(bool activation, float sample_rate, vector<ImageCaptureBase::ImageRequest> request, const std::string& vehicle_name = "");
    void cleanCameraStoredData(const std::string& vehicle_name = "");
    void saveCameraStoredData(const std::string& path = "", const std::string& vehicle_name = "");

    // Methods related to the IMU data gathering
    void setImuActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanImuStoredData(const std::string& vehicle_name = "");
    IMUStoredData getImuStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the PWM data gathering
    void setPwmActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPwmStoredData(const std::string& vehicle_name = "");
    PWMStoredData getPwmStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the ground truth position data gathering
    void setPositionActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanPositionStoredData(const std::string& vehicle_name = "");
    PositionStoredData getPositionStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the barometer data gathering
    void setBarometerActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanBarometerStoredData(const std::string& vehicle_name = "");
    BarometerStoredData getBarometerStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the magnetometer data gathering
    void setMagnetometerActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanMagnetometerStoredData(const std::string& vehicle_name = "");
    MagnetometerStoredData getMagnetometerStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to the GPS data gathering
    void setGPSActivation(bool activation, float sample_rate, const std::string& vehicle_name = "");
    void cleanGPSStoredData(const std::string& vehicle_name = "");
    GPSStoredData getGPSStoredDataVec(const std::string& vehicle_name = "");

    // Methods related to drone teleportation
    void setTeleportYawRef(float yaw_angle_ref, const std::string& vehicle_name = "");

    // Methods related to drone failures
    void setDamageCoefficients(float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4, const std::string& vehicle_name = "");
    void setLockedProppellers(bool locked_1, bool locked_2, bool locked_3, bool locked_4, const std::string& vehicle_name = "");
    void setLockedPropellerCoefficients(float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4, const std::string& vehicle_name = "");
    DamageCoefficients getDamageCoefficients(const std::string& vehicle_name = "");
    LockedPropellers getLockedPropellers(const std::string& vehicle_name = "");
    DamageCoefficients getLockedPropellerCoefficients(const std::string& vehicle_name = "");
    DamageCoefficients getMotorPWMs(const std::string& vehicle_name = "");

    MultirotorRpcLibClient* moveByVelocityBodyFrameAsync(float vx, float vy, float vz, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByVelocityZBodyFrameAsync(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByMotorPWMsAsync(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawZAsync(float roll, float pitch, float yaw, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawThrottleAsync(float roll, float pitch, float yaw, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawrateThrottleAsync(float roll, float pitch, float yaw_rate, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByRollPitchYawrateZAsync(float roll, float pitch, float yaw_rate, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByAngleRatesZAsync(float roll_rate, float pitch_rate, float yaw_rate, float z, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByAngleRatesThrottleAsync(float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByVelocityAsync(float vx, float vy, float vz, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByVelocityZAsync(float vx, float vy, float z, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveOnPathAsync(const vector<Vector3r>& path, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveToPositionAsync(float x, float y, float z, float velocity, float timeout_sec = Utils::max<float>(),
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), 
        float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveToZAsync(float z, float velocity, float timeout_sec = Utils::max<float>(),
        const YawMode& yaw_mode = YawMode(), float lookahead = -1, float adaptive_lookahead = 1, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* moveByManualAsync(float vx_max, float vy_max, float z_min, float duration,
        DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom, const YawMode& yaw_mode = YawMode(), const std::string& vehicle_name = "");
    MultirotorRpcLibClient* rotateToYawAsync(float yaw, float timeout_sec = Utils::max<float>(), float margin = 5, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* rotateByYawRateAsync(float yaw_rate, float duration, const std::string& vehicle_name = "");
    MultirotorRpcLibClient* hoverAsync(const std::string& vehicle_name = "");

    void setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd, const std::string& vehicle_name="");
    void moveByRC(const RCData& rc_data, const std::string& vehicle_name = "");

    MultirotorState getMultirotorState(const std::string& vehicle_name = "");
    RotorStates getRotorStates(const std::string& vehicle_name = "");

    bool setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
        float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z, const std::string& vehicle_name = "");

    virtual MultirotorRpcLibClient* waitOnLastTask(bool* task_result = nullptr, float timeout_sec = Utils::nan<float>()) override;

    virtual ~MultirotorRpcLibClient();    //required for pimpl

private:
    struct impl;
    std::unique_ptr<impl> pimpl_;
};

}} //namespace
#endif
