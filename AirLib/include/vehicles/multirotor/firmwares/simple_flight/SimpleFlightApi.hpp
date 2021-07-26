// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "physics/Environment.hpp"
#include "physics/Kinematics.hpp"
#include "vehicles/multirotor/MultiRotorParams.hpp"
#include "common/Common.hpp"
#include "firmware/Firmware.hpp"
#include "AirSimSimpleFlightBoard.hpp"
#include "AirSimSimpleFlightCommLink.hpp"
#include "AirSimSimpleFlightEstimator.hpp"
#include "AirSimSimpleFlightCommon.hpp"
#include "physics/PhysicsBody.hpp"
#include "common/AirSimSettings.hpp"

//TODO: we need to protect contention between physics thread and API server thread

namespace msr { namespace airlib {

class SimpleFlightApi : public MultirotorApiBase {

public:
    SimpleFlightApi(const MultiRotorParams* vehicle_params, const AirSimSettings::VehicleSetting* vehicle_setting)
        : vehicle_params_(vehicle_params)
    {
        readSettings(*vehicle_setting);

        //TODO: set below properly for better high speed safety
        safety_params_.vel_to_breaking_dist = safety_params_.min_breaking_dist = 0;

        //create sim implementations of board and commlink
        board_.reset(new AirSimSimpleFlightBoard(&params_));
        comm_link_.reset(new AirSimSimpleFlightCommLink());
        estimator_.reset(new AirSimSimpleFlightEstimator());

        //create firmware
        firmware_.reset(new simple_flight::Firmware(&params_, board_.get(), comm_link_.get(), estimator_.get()));
    }


public: //VehicleApiBase implementation
    virtual void resetImplementation() override
    {
        MultirotorApiBase::resetImplementation();

        firmware_->reset();
    }
    virtual void update() override
    {
        MultirotorApiBase::update();

        //update controller which will update actuator control signal
        firmware_->update();
    }
    virtual bool isApiControlEnabled() const override
    {
        return firmware_->offboardApi().hasApiControl();
    }
    virtual void enableApiControl(bool is_enabled) override
    {
        if (is_enabled) {
            //comm_link should print message so no extra handling for errors
            std::string message;
            firmware_->offboardApi().requestApiControl(message);
        }
        else
            firmware_->offboardApi().releaseApiControl();
    }
    virtual bool armDisarm(bool arm) override
    {
        std::string message;
        if (arm)
            return firmware_->offboardApi().arm(message);
        else
            return firmware_->offboardApi().disarm(message);
    }
    virtual GeoPoint getHomeGeoPoint() const override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getHomeGeoPoint());
    }
    virtual void getStatusMessages(std::vector<std::string>& messages) override
    {
        comm_link_->getStatusMessages(messages);
    }

    virtual const SensorCollection& getSensors() const override
    {
        return vehicle_params_->getSensors();
    }

public: //MultirotorApiBase implementation
    //virtual real_T getActuation(unsigned int rotor_index, std::mutex& mutexito) override
    //{
    //    //unused(mutexito);
    //    storeIMUData();
    //    storeBarometerData();
    //    storeMagnetometerData();
    //    storeGPSData();

    //    mutexito.unlock();
    //    storeCameraData();
    //    mutexito.lock();

    //    auto control_signal_old = board_->getMotorControlSignal(rotor_index);
    //    real_T control_signal;
    //    if (locked_propeller[rotor_index] == true) {
    //        control_signal = lock_coefficients[rotor_index];
    //    }
    //    else {
    //        real_T prop_damage = propeller_damage_coefficients[rotor_index];
    //        control_signal = control_signal_old * prop_damage;
    //    }
    //    return control_signal;
    //}

    virtual real_T getActuation(unsigned int rotor_index) override
    {
        storeIMUData();
        storeBarometerData();
        storeMagnetometerData();
        storeGPSData();
        //storeCameraData();
        auto control_signal_old = board_->getMotorControlSignal(rotor_index);
        real_T control_signal;
        if (locked_propeller[rotor_index] == true) {
            control_signal = lock_coefficients[rotor_index];
        }
        else {
            real_T prop_damage = propeller_damage_coefficients[rotor_index];
            control_signal = control_signal_old * prop_damage;
        }
        return control_signal;
    }

    // Methods related to the Camera data gathering
    void storeCameraData() override
    {
        if (Camera_activate_store) {
            uint64_t time_new = clock()->nowNanos();
            int Camera_threshold = UE4_second / Camera_sample_rate;
            uint64_t time_threshold = Camera_time_old + Camera_threshold;
            if (time_new >= time_threshold) {
                const auto& local_Camera_data = Camera_api->getImages(Camera_request);
                Camera_time_old = time_new;
                Camera_data.push_back(local_Camera_data);
            }
        }
    }

    void setCameraAct(bool activation, float sample_rate, const std::vector<ImageCaptureBase::ImageRequest>& request, VehicleSimApiBase* const& api) override
    {
        Camera_activate_store = activation;
        Camera_sample_rate = sample_rate;
        //unused(request);
        Camera_request = request;
        Camera_api = api;
    }

    void cleanCameraSD() override
    {
        Camera_activate_store = false;
        Camera_data.clear();
    }

    void saveCameraSD(std::string path) override
    {
        unused(path);
    }

    // Methods related to the IMU data gathering
    void storeIMUData() override
    {
        if (IMU_activate_store) {
            auto local_IMU_data = getImuData("");
            uint64_t time_new = local_IMU_data.time_stamp;
            int IMU_threshold = UE4_second / IMU_sample_rate;
            uint64_t time_threshold = IMU_time_old + IMU_threshold;
            if (time_new >= time_threshold) {
                IMU_time_old = time_new;
                IMU_data.push_back(local_IMU_data);
            }
        }
    }

    void setIMUAct(bool activation, float sample_rate) override
    {
        IMU_activate_store = activation;
        IMU_sample_rate = sample_rate;
    }

    void cleanIMUSD() override
    {
        IMU_activate_store = false;
        IMU_data.clear();
    }

    std::vector<msr::airlib::ImuBase::Output> getIMUStoredData() override
    {
        return IMU_data;
    }

    // Methods related to the barometer data gathering
    void storeBarometerData() override
    {
        if (barometer_activate_store) {
            auto local_barometer_data = getBarometerData("");
            uint64_t time_new = local_barometer_data.time_stamp;
            int barometer_threshold = UE4_second / barometer_sample_rate;
            uint64_t time_threshold = barometer_time_old + barometer_threshold;
            if (time_new >= time_threshold) {
                barometer_time_old = time_new;
                barometer_data.push_back(local_barometer_data);
            }
        }
    }

    void setBarometerAct(bool activation, float sample_rate) override
    {
        barometer_activate_store = activation;
        barometer_sample_rate = sample_rate;
    }

    void cleanBarometerSD() override
    {
        barometer_activate_store = false;
        barometer_data.clear();
    }

    std::vector<msr::airlib::BarometerBase::Output> getBarometerStoredData() override
    {
        return barometer_data;
    }

    // Methods related to the magnetometer data gathering
    void storeMagnetometerData() override
    {
        if (magnetometer_activate_store) {
            auto local_magnetometer_data = getMagnetometerData("");
            uint64_t time_new = local_magnetometer_data.time_stamp;
            int magnetometer_threshold = UE4_second / magnetometer_sample_rate;
            uint64_t time_threshold = magnetometer_time_old + magnetometer_threshold;
            if (time_new >= time_threshold) {
                magnetometer_time_old = time_new;
                magnetometer_data.push_back(local_magnetometer_data);
            }
        }
    }

    void setMagnetometerAct(bool activation, float sample_rate) override
    {
        magnetometer_activate_store = activation;
        magnetometer_sample_rate = sample_rate;
    }

    void cleanMagnetometerSD() override
    {
        magnetometer_activate_store = false;
        magnetometer_data.clear();
    }

    std::vector<msr::airlib::MagnetometerBase::Output> getMagnetometerStoredData() override
    {
        return magnetometer_data;
    }

    // Methods related to the GPS data gathering
    void storeGPSData() override
    {
        if (GPS_activate_store) {
            auto local_GPS_data = getGpsData("");
            uint64_t time_new = local_GPS_data.time_stamp;
            int GPS_threshold = UE4_second / GPS_sample_rate;
            uint64_t time_threshold = GPS_time_old + GPS_threshold;
            if (time_new >= time_threshold) {
                GPS_time_old = time_new;
                GPS_data.push_back(local_GPS_data);
            }
        }
    }

    void setGPSAct(bool activation, float sample_rate) override
    {
        GPS_activate_store = activation;
        GPS_sample_rate = sample_rate;
    }

    void cleanGPSSD() override
    {
        GPS_activate_store = false;
        GPS_data.clear();
    }

    std::vector<msr::airlib::GpsBase::Output> getGPSStoredData() override
    {
        return GPS_data;
    }

    // Methods related to the drone failures
    void setDamageCoeff(float new_coeffs[4]) override
    {
        propeller_damage_coefficients[0] = new_coeffs[0];
        propeller_damage_coefficients[1] = new_coeffs[1];
        propeller_damage_coefficients[2] = new_coeffs[2];
        propeller_damage_coefficients[3] = new_coeffs[3];
    }

    void setLockedProp(bool locked[4]) override
    {
        locked_propeller[0] = locked[0];
        locked_propeller[1] = locked[1];
        locked_propeller[2] = locked[2];
        locked_propeller[3] = locked[3];
    }

    void setLockedPropellerCoeff(float locked_coeff[4]) override
    {
        lock_coefficients[0] = locked_coeff[0];
        lock_coefficients[1] = locked_coeff[1];
        lock_coefficients[2] = locked_coeff[2];
        lock_coefficients[3] = locked_coeff[3];
    }

    float* getDamageCoeff() override
    {
        return propeller_damage_coefficients;
    }

    bool* getLockedProp() override
    {
        return locked_propeller;
    }

    float* getLockedPropellerCoeff() override
    {
        return lock_coefficients;
    }

    float* getPWMs() override
    {
        PWMs[0] = board_->getMotorControlSignal(0);
        PWMs[1] = board_->getMotorControlSignal(1);
        PWMs[2] = board_->getMotorControlSignal(2);
        PWMs[3] = board_->getMotorControlSignal(3);
        return PWMs;
    }

    virtual size_t getActuatorCount() const override
    {
        return vehicle_params_->getParams().rotor_count;
    }
    virtual void moveByRC(const RCData& rc_data) override
    {
        setRCData(rc_data);
    }
    virtual void setSimulatedGroundTruth(const Kinematics::State* kinematics, const Environment* environment) override
    {
        board_->setGroundTruthKinematics(kinematics);
        estimator_->setGroundTruthKinematics(kinematics, environment);
    }
    virtual bool setRCData(const RCData& rc_data) override
    {
        last_rcData_ = rc_data;
        if (rc_data.is_valid) {
            board_->setIsRcConnected(true);
            board_->setInputChannel(0, rc_data.roll); //X
            board_->setInputChannel(1, rc_data.yaw); //Y
            board_->setInputChannel(2, rc_data.throttle); //F
            board_->setInputChannel(3, -rc_data.pitch); //Z
            board_->setInputChannel(4, static_cast<float>(rc_data.getSwitch(0))); //angle rate or level
            board_->setInputChannel(5, static_cast<float>(rc_data.getSwitch(1))); //Allow API control
            board_->setInputChannel(6, static_cast<float>(rc_data.getSwitch(2)));
            board_->setInputChannel(7, static_cast<float>(rc_data.getSwitch(3)));
            board_->setInputChannel(8, static_cast<float>(rc_data.getSwitch(4)));
            board_->setInputChannel(9, static_cast<float>(rc_data.getSwitch(5)));
            board_->setInputChannel(10, static_cast<float>(rc_data.getSwitch(6)));
            board_->setInputChannel(11, static_cast<float>(rc_data.getSwitch(7)));
        }
        else { //else we don't have RC data
            board_->setIsRcConnected(false);
        }

        return true;
    }

protected:
    virtual Kinematics::State getKinematicsEstimated() const override
    {
        return AirSimSimpleFlightCommon::toKinematicsState3r(firmware_->offboardApi().
            getStateEstimator().getKinematicsEstimated());
    }

    virtual Vector3r getPosition() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getPosition();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Vector3r getVelocity() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getLinearVelocity();
        return AirSimSimpleFlightCommon::toVector3r(val);
    }

    virtual Quaternionr getOrientation() const override
    {
        const auto& val = firmware_->offboardApi().getStateEstimator().getOrientation();
        return AirSimSimpleFlightCommon::toQuaternion(val);
    }

    virtual LandedState getLandedState() const override
    {
        return firmware_->offboardApi().getLandedState() ? LandedState::Landed : LandedState::Flying;
    }

    virtual RCData getRCData() const override
    {
        //return what we received last time through setRCData
        return last_rcData_;
    }

    virtual GeoPoint getGpsLocation() const override
    {
        return AirSimSimpleFlightCommon::toGeoPoint(firmware_->offboardApi().getGeoPoint());
    }

    virtual float getCommandPeriod() const override
    {
        return 1.0f / 50; //50hz
    }

    virtual float getTakeoffZ() const override
    {
        // pick a number, 3 meters is probably safe
        // enough to get out of the backwash turbulence.  Negative due to NED coordinate system.
        return params_.takeoff.takeoff_z;
    }

    virtual float getDistanceAccuracy() const override
    {
        return 0.5f;    //measured in simulator by firing commands "MoveToLocation -x 0 -y 0" multiple times and looking at distance traveled
    }

    virtual void commandMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm) override
    {
        //Utils::log(Utils::stringf("commandMotorPWMs %f, %f, %f, %f", front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough, GoalModeType::Passthrough);

        simple_flight::Axis4r goal(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchYawZ(float roll, float pitch, float yaw, float z) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawZ %f, %f, %f, %f", pitch, roll, z, yaw));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(roll, pitch, yaw, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw, throttle));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::Passthrough);

        simple_flight::Axis4r goal(roll, pitch, yaw, throttle);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw, throttle));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleRate, GoalModeType::Passthrough);

        simple_flight::Axis4r goal(roll, pitch, yaw_rate, throttle);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleLevel, GoalModeType::AngleLevel, GoalModeType::AngleRate, GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(roll, pitch, yaw_rate, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(roll_rate, pitch_rate, yaw_rate, z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle) override
    {
        //Utils::log(Utils::stringf("commandRollPitchYawThrottle %f, %f, %f, %f", roll, pitch, yaw_rate, throttle));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::AngleRate, GoalModeType::Passthrough);

        simple_flight::Axis4r goal(roll_rate, pitch_rate, yaw_rate, throttle);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocity(float vx, float vy, float vz, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandVelocity %f, %f, %f, %f", vx, vy, vz, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld,
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel,
            GoalModeType::VelocityWorld);

        simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), vz);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void commandVelocityZ(float vx, float vy, float z, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandVelocityZ %f, %f, %f, %f", vx, vy, z, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::VelocityWorld, GoalModeType::VelocityWorld,
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel,
            GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(vy, vx, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual void setControllerGains(uint8_t controller_type, const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) override
    {
        simple_flight::GoalModeType controller_type_enum = static_cast<simple_flight::GoalModeType>(controller_type);

        vector<float> kp_axis4(4);
        vector<float> ki_axis4(4);
        vector<float> kd_axis4(4);

        switch(controller_type_enum) {
            // roll gain, pitch gain, yaw gain, and no gains in throttle / z axis
            case simple_flight::GoalModeType::AngleRate:
                kp_axis4 = {kp[0], kp[1], kp[2], 1.0};
                ki_axis4  ={ki[0], ki[1], ki[2], 0.0};
                kd_axis4 = {kd[0], kd[1], kd[2], 0.0};
                params_.angle_rate_pid.p.setValues(kp_axis4);
                params_.angle_rate_pid.i.setValues(ki_axis4);
                params_.angle_rate_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::AngleLevel:
                kp_axis4 = {kp[0], kp[1], kp[2], 1.0};
                ki_axis4 = {ki[0], ki[1], ki[2], 0.0};
                kd_axis4 = {kd[0], kd[1], kd[2], 0.0};
                params_.angle_level_pid.p.setValues(kp_axis4);
                params_.angle_level_pid.i.setValues(ki_axis4);
                params_.angle_level_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::VelocityWorld:
                kp_axis4 = {kp[1], kp[0], 0.0, kp[2]};
                ki_axis4 = {ki[1], ki[0], 0.0, ki[2]};
                kd_axis4 = {kd[1], kd[0], 0.0, kd[2]};
                params_.velocity_pid.p.setValues(kp_axis4);
                params_.velocity_pid.i.setValues(ki_axis4);
                params_.velocity_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            case simple_flight::GoalModeType::PositionWorld:
                kp_axis4 = {kp[1], kp[0], 0.0, kp[2]};
                ki_axis4 = {ki[1], ki[0], 0.0, ki[2]};
                kd_axis4 = {kd[1], kd[0], 0.0, kd[2]};
                params_.position_pid.p.setValues(kp_axis4);
                params_.position_pid.i.setValues(ki_axis4);
                params_.position_pid.d.setValues(kd_axis4);
                params_.gains_changed = true;
                break;
            default:
                Utils::log("Unimplemented controller type");
                break;
        }
    }

    virtual void commandPosition(float x, float y, float z, const YawMode& yaw_mode) override
    {
        //Utils::log(Utils::stringf("commandPosition %f, %f, %f, %f", x, y, z, yaw_mode.yaw_or_rate));

        typedef simple_flight::GoalModeType GoalModeType;
        simple_flight::GoalMode mode(GoalModeType::PositionWorld, GoalModeType::PositionWorld,
            yaw_mode.is_rate ? GoalModeType::AngleRate : GoalModeType::AngleLevel,
            GoalModeType::PositionWorld);

        simple_flight::Axis4r goal(y, x, Utils::degreesToRadians(yaw_mode.yaw_or_rate), z);

        std::string message;
        firmware_->offboardApi().setGoalAndMode(&goal, &mode, message);
    }

    virtual const MultirotorApiParams& getMultirotorApiParams() const override
    {
        return safety_params_;
    }

    //*** End: MultirotorApiBase implementation ***//

private:
    //convert pitch, roll, yaw from -1 to 1 to PWM
    static uint16_t angleToPwm(float angle)
    {
        return static_cast<uint16_t>(angle * 500.0f + 1500.0f);
    }
    static uint16_t thrustToPwm(float thrust)
    {
        return static_cast<uint16_t>((thrust < 0 ? 0 : thrust) * 1000.0f + 1000.0f);
    }
    static uint16_t switchTopwm(float switchVal, uint maxSwitchVal = 1)
    {
        return static_cast<uint16_t>(1000.0f * switchVal / maxSwitchVal + 1000.0f);
    }

    void readSettings(const AirSimSettings::VehicleSetting& vehicle_setting)
    {
        params_.default_vehicle_state = simple_flight::VehicleState::fromString(
            vehicle_setting.default_vehicle_state == "" ? "Armed" : vehicle_setting.default_vehicle_state);

        remote_control_id_ = vehicle_setting.rc.remote_control_id;
        params_.rc.allow_api_when_disconnected = vehicle_setting.rc.allow_api_when_disconnected;
        params_.rc.allow_api_always = vehicle_setting.allow_api_always;
    }

private:
    const MultiRotorParams* vehicle_params_;
    //float UE4_second = 993705198.592;
    int UE4_second = 1000000000;

    // Variables related to potential failures
    float propeller_damage_coefficients[4] = { 1.0, 1.0, 1.0, 1.0 };
    bool locked_propeller[4] = { false, false, false, false };
    float lock_coefficients[4] = { 1.0, 1.0, 1.0, 1.0 };
    float PWMs[4] = { 1.0, 1.0, 1.0, 1.0 };

    // Variables related to Camera data gathering
    bool Camera_activate_store = false;
    int Camera_sample_rate = 60;
    uint64_t Camera_time_old = 0;
    std::vector<vector<ImageCaptureBase::ImageResponse>> Camera_data;
    VehicleSimApiBase* Camera_api;
    std::vector<ImageCaptureBase::ImageRequest> Camera_request = {ImageCaptureBase::ImageRequest("0", ImageCaptureBase::ImageType::Scene), ImageCaptureBase::ImageRequest("0", ImageCaptureBase::ImageType::Scene) };

    // Variables related to IMU data gathering
    bool IMU_activate_store = false;
    float IMU_sample_rate = 1000;
    uint64_t IMU_time_old = 0;
    std::vector<msr::airlib::ImuBase::Output> IMU_data;

    // Variables related to barometer data gathering
    bool barometer_activate_store = false;
    float barometer_sample_rate = 50;
    uint64_t barometer_time_old = 0;
    std::vector<msr::airlib::BarometerBase::Output> barometer_data;

    // Variables related to magnetometer data gathering
    bool magnetometer_activate_store = false;
    float magnetometer_sample_rate = 50;
    uint64_t magnetometer_time_old = 0;
    std::vector<msr::airlib::MagnetometerBase::Output> magnetometer_data;

    // Variables related to GPS data gathering
    bool GPS_activate_store = false;
    float GPS_sample_rate = 50;
    uint64_t GPS_time_old = 0;
    std::vector<msr::airlib::GpsBase::Output> GPS_data;

    int remote_control_id_ = 0;
    simple_flight::Params params_;

    unique_ptr<AirSimSimpleFlightBoard> board_;
    unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
    unique_ptr<AirSimSimpleFlightEstimator> estimator_;
    unique_ptr<simple_flight::IFirmware> firmware_;

    MultirotorApiParams safety_params_;

    RCData last_rcData_;
};

}} //namespace
#endif
