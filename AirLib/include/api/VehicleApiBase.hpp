// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef air_VehicleApiBase_hpp
#define air_VehicleApiBase_hpp

#include "api/VehicleSimApiBase.hpp"

#include "common/CommonStructs.hpp"
#include "common/UpdatableObject.hpp"
#include "common/Common.hpp"
#include "common/Waiter.hpp"
#include "safety/SafetyEval.hpp"
#include "common/CommonStructs.hpp"
#include "common/ImageCaptureBase.hpp"
#include "sensors/SensorCollection.hpp"
#include "sensors/lidar/LidarBase.hpp"
#include "sensors/imu/ImuBase.hpp"
#include "sensors/barometer/BarometerBase.hpp"
#include "sensors/magnetometer/MagnetometerBase.hpp"
#include "sensors/distance/DistanceBase.hpp"
#include "sensors/gps/GpsBase.hpp"
#include <exception>
#include <string>

namespace msr { namespace airlib {

/*
Vehicle controller allows to obtain state from vehicle and send control commands to the vehicle.
State can include many things including sensor data, logs, estimated state from onboard computer etc.
Control commands can be low level actuation commands or high level movement commands.
The base class defines usually available methods that all vehicle controllers may implement.
Some methods may not be applicable to specific vehicle in which case an exception may be raised or call may be ignored.
*/
class VehicleApiBase : public UpdatableObject {
public:
    virtual void enableApiControl(bool is_enabled) = 0;
    virtual bool isApiControlEnabled() const = 0;
    virtual bool armDisarm(bool arm) = 0;
    virtual GeoPoint getHomeGeoPoint() const = 0;

    virtual void update() override
    {
        UpdatableObject::update();
    }

    virtual void cancelLastTask()
    {
        //if derived class supports async task then override this method
    }

    virtual bool isReady(std::string& message) const
    {
        unused(message);
        return true;
    }

    virtual bool canArm() const
    {
        return true;
    }

    //if vehicle supports it, call this method to send
    //kinematics and other info to somewhere (ex. log viewer, file, cloud etc)
    virtual void sendTelemetry(float last_interval = -1)
    {
        //no default action
        unused(last_interval);
    }

    //below APIs are used by FastPhysicsEngine
    //virtual real_T getActuation(unsigned int actuator_index, std::mutex& mutexito)
    //{
    //    unused(actuator_index);
    //    unused(mutexito);
    //    throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
    //}

    virtual void droneTeleportReset()
    {
        throw VehicleCommandNotImplementedException("droneTeleportReset API is not supported for this vehicle");
    }

    virtual real_T getActuation(unsigned int actuator_index)
    {
        unused(actuator_index);
        throw VehicleCommandNotImplementedException("getActuation API is not supported for this vehicle");
    }

    virtual std::vector<real_T> actuator_dyn(Kinematics::State previous)
    {
        unused(previous);
        throw VehicleCommandNotImplementedException("actuator_dyn API is not supported for this vehicle");
    }

    virtual void setNextPathLocObj(Pose pose)
    {
        unused(pose);
        throw VehicleCommandNotImplementedException("setNextPathLocObj API is not supported for this vehicle");
    }

    // Methods related to the reference position data gathering
    virtual void storePosRefData()
    {
        throw VehicleCommandNotImplementedException("storePosRefData API is not supported for this vehicle");
    }

    virtual void setPosRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPosRefAct API is not supported for this vehicle");
    }

    virtual void cleanPosRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanPosRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPosRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getPosRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the position error data gathering
    virtual void storePosErrorData()
    {
        throw VehicleCommandNotImplementedException("storePosErrorData API is not supported for this vehicle");
    }

    virtual void setPosErrorAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPosErrorAct API is not supported for this vehicle");
    }

    virtual void cleanPosErrorSD()
    {
        throw VehicleCommandNotImplementedException("cleanPosErrorSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPosErrorStoredData()
    {
        throw VehicleCommandNotImplementedException("getPosErrorStoredData API is not supported for this vehicle");
    }

    // Methods related to the position error derivative data gathering
    virtual void storePosErrorDotData()
    {
        throw VehicleCommandNotImplementedException("storePosErrorDotData API is not supported for this vehicle");
    }

    virtual void setPosErrorDotAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPosErrorDotAct API is not supported for this vehicle");
    }

    virtual void cleanPosErrorDotSD()
    {
        throw VehicleCommandNotImplementedException("cleanPosErrorDotSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPosErrorDotStoredData()
    {
        throw VehicleCommandNotImplementedException("getPosErrorDotStoredData API is not supported for this vehicle");
    }

    // Methods related to the reference velocity data gathering
    virtual void storeVelRefData()
    {
        throw VehicleCommandNotImplementedException("storeVelRefData API is not supported for this vehicle");
    }

    virtual void setVelRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setVelRefAct API is not supported for this vehicle");
    }

    virtual void cleanVelRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanVelRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getVelRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getVelRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the velocity data gathering
    virtual void storeVelData()
    {
        throw VehicleCommandNotImplementedException("storeVelData API is not supported for this vehicle");
    }

    virtual void setVelAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setVelAct API is not supported for this vehicle");
    }

    virtual void cleanVelSD()
    {
        throw VehicleCommandNotImplementedException("cleanVelSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getVelStoredData()
    {
        throw VehicleCommandNotImplementedException("getVelStoredData API is not supported for this vehicle");
    }

    // Methods related to the reference acceleration data gathering
    virtual void storeAccRefData()
    {
        throw VehicleCommandNotImplementedException("storeAccRefData API is not supported for this vehicle");
    }

    virtual void setAccRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setAccRefAct API is not supported for this vehicle");
    }

    virtual void cleanAccRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanAccRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getAccRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getAccRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the reference rotational rates data gathering
    virtual void storePqrRefData()
    {
        throw VehicleCommandNotImplementedException("storePqrRefData API is not supported for this vehicle");
    }

    virtual void setPqrRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPqrRefAct API is not supported for this vehicle");
    }

    virtual void cleanPqrRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanPqrRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPqrRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getPqrRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the rotational rates data gathering
    virtual void storePqrData()
    {
        throw VehicleCommandNotImplementedException("storePqrData API is not supported for this vehicle");
    }

    virtual void setPqrAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPqrAct API is not supported for this vehicle");
    }

    virtual void cleanPqrSD()
    {
        throw VehicleCommandNotImplementedException("cleanPqrSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPqrStoredData()
    {
        throw VehicleCommandNotImplementedException("getPqrStoredData API is not supported for this vehicle");
    }

    // Methods related to the reference thrust data gathering
    virtual void storeThrustRefData()
    {
        throw VehicleCommandNotImplementedException("storeThrustRefData API is not supported for this vehicle");
    }

    virtual void setThrustRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setThrustRefAct API is not supported for this vehicle");
    }

    virtual void cleanThrustRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanThrustRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getThrustRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getThrustRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the omega (motor rotations) data gathering
    virtual void storeOmegasData()
    {
        throw VehicleCommandNotImplementedException("storeOmegasData API is not supported for this vehicle");
    }

    virtual void setOmegasAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setOmegasAct API is not supported for this vehicle");
    }

    virtual void cleanOmegasSD()
    {
        throw VehicleCommandNotImplementedException("cleanOmegasSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getOmegasStoredData()
    {
        throw VehicleCommandNotImplementedException("getOmegasStoredData API is not supported for this vehicle");
    }

    // Methods related to the reference yaw data gathering
    virtual void storeYawRefData()
    {
        throw VehicleCommandNotImplementedException("storeYawRefData API is not supported for this vehicle");
    }

    virtual void setYawRefAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setYawRefAct API is not supported for this vehicle");
    }

    virtual void cleanYawRefSD()
    {
        throw VehicleCommandNotImplementedException("cleanYawRefSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getYawRefStoredData()
    {
        throw VehicleCommandNotImplementedException("getYawRefStoredData API is not supported for this vehicle");
    }

    // Methods related to the orientation data gathering
    virtual void storeOrientationData()
    {
        throw VehicleCommandNotImplementedException("storeOrientationData API is not supported for this vehicle");
    }

    virtual void setOrientationAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setOrientationAct API is not supported for this vehicle");
    }

    virtual void cleanOrientationSD()
    {
        throw VehicleCommandNotImplementedException("cleanOrientationSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getOrientationStoredData()
    {
        throw VehicleCommandNotImplementedException("getOrientationStoredData API is not supported for this vehicle");
    }

    // Methods related to the position integrator data gathering
    virtual void storePositionIntegratorData()
    {
        throw VehicleCommandNotImplementedException("storePositionIntegratorData API is not supported for this vehicle");
    }

    virtual void setPositionIntegratorAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPositionIntegratorAct API is not supported for this vehicle");
    }

    virtual void cleanPositionIntegratorSD()
    {
        throw VehicleCommandNotImplementedException("cleanPositionIntegratorSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPositionIntegratorStoredData()
    {
        throw VehicleCommandNotImplementedException("getPositionIntegratorStoredData API is not supported for this vehicle");
    }

    // Methods related to the thrust PI controller data gathering
    virtual void storeThrustPiData()
    {
        throw VehicleCommandNotImplementedException("storeThrustPiData API is not supported for this vehicle");
    }

    virtual void setThrustPiAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setThrustPiAct API is not supported for this vehicle");
    }

    virtual void cleanThrustPiSD()
    {
        throw VehicleCommandNotImplementedException("cleanThrustPiSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getThrustPiStoredData()
    {
        throw VehicleCommandNotImplementedException("getThrustPiStoredData API is not supported for this vehicle");
    }

    // Methods related to the Camera data gathering
    virtual void storeCameraData()
    {
        throw VehicleCommandNotImplementedException("storeCameraData API is not supported for this vehicle");
    }

    virtual void setCameraAct(bool activation, float sample_rate, const std::vector<ImageCaptureBase::ImageRequest>& request, VehicleSimApiBase* const& api)
    {
        unused(activation);
        unused(sample_rate);
        unused(request);
        unused(api);
        throw VehicleCommandNotImplementedException("setCameraAct API is not supported for this vehicle");
    }

    virtual void cleanCameraSD()
    {
        throw VehicleCommandNotImplementedException("cleanCameraSD API is not supported for this vehicle");
    }

    virtual void saveCameraSD(std::string path)
    {
        throw VehicleCommandNotImplementedException("saveCameraStoredData API is not supported for this vehicle");
    }


    // Methods related to the IMU data gathering
    virtual void storeIMUData()
    {
        throw VehicleCommandNotImplementedException("storeIMUData API is not supported for this vehicle");
    }

    virtual void setIMUAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setIMUAct API is not supported for this vehicle");
    }

    virtual void cleanIMUSD()
    {
        throw VehicleCommandNotImplementedException("cleanIMUStoredData API is not supported for this vehicle");
    }

    virtual std::vector<msr::airlib::ImuBase::Output> getIMUStoredData()
    {
        throw VehicleCommandNotImplementedException("getIMUStoredData API is not supported for this vehicle");
    }

    // Methods related to the PWMs data gathering
    virtual void storePWMData()
    {
        throw VehicleCommandNotImplementedException("storePWMData API is not supported for this vehicle");
    }

    virtual void setPWMAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPWMAct API is not supported for this vehicle");
    }

    virtual void cleanPWMSD()
    {
        throw VehicleCommandNotImplementedException("cleanPWMSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPWMStoredData()
    {
        throw VehicleCommandNotImplementedException("getPWMStoredData API is not supported for this vehicle");
    }

    // Methods related to the position data gathering
    virtual void storePositionData()
    {
        throw VehicleCommandNotImplementedException("storePositionData API is not supported for this vehicle");
    }

    virtual void setPositionAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setPositionAct API is not supported for this vehicle");
    }

    virtual void cleanPositionSD()
    {
        throw VehicleCommandNotImplementedException("cleanPositionSD API is not supported for this vehicle");
    }

    virtual std::vector<std::vector<float>> getPositionStoredData()
    {
        throw VehicleCommandNotImplementedException("getPositionStoredData API is not supported for this vehicle");
    }

    // Methods related to the barometer data gathering

    virtual void storeBarometerData()
    {
        throw VehicleCommandNotImplementedException("storeBarometerData API is not supported for this vehicle");
    }

    virtual void setBarometerAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setBarometerAct API is not supported for this vehicle");
    }

    virtual void cleanBarometerSD()
    {
        throw VehicleCommandNotImplementedException("cleanBarometerSD API is not supported for this vehicle");
    }

    virtual std::vector<msr::airlib::BarometerBase::Output> getBarometerStoredData()
    {
        throw VehicleCommandNotImplementedException("getBarometerStoredData API is not supported for this vehicle");
    }

    // Methods related to the Magnetometer data gathering
    virtual void storeMagnetometerData()
    {
        throw VehicleCommandNotImplementedException("storeMagnetometerData API is not supported for this vehicle");
    }

    virtual void setMagnetometerAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setMagnetometerAct API is not supported for this vehicle");
    }

    virtual void cleanMagnetometerSD()
    {
        throw VehicleCommandNotImplementedException("cleanMagnetometerSD API is not supported for this vehicle");
    }

    virtual std::vector<msr::airlib::MagnetometerBase::Output> getMagnetometerStoredData()
    {
        throw VehicleCommandNotImplementedException("getMagnetometerStoredData API is not supported for this vehicle");
    }

    // Methods related to the GPS data gathering
    virtual void storeGPSData()
    {
        throw VehicleCommandNotImplementedException("storeGpsData API is not supported for this vehicle");
    }

    virtual void setGPSAct(bool activation, float sample_rate)
    {
        unused(activation);
        unused(sample_rate);
        throw VehicleCommandNotImplementedException("setGPSAct API is not supported for this vehicle");
    }

    virtual void cleanGPSSD()
    {
        throw VehicleCommandNotImplementedException("cleanGPSSD API is not supported for this vehicle");
    }

    virtual std::vector<msr::airlib::GpsBase::Output> getGPSStoredData()
    {
        throw VehicleCommandNotImplementedException("getGPSStoredData API is not supported for this vehicle");
    }

    // Methods related to the drone teleportation
    virtual void setTeleportYawReference(float yaw_angle_ref)
    {
        unused(yaw_angle_ref);
        throw VehicleCommandNotImplementedException("setTeleportYawReference API is not supported for this vehicle");
    }

    // Methods related to the drone failure
    virtual void setDamageCoeff(float new_coeffs[4])
    {
        unused(new_coeffs);
        throw VehicleCommandNotImplementedException("setDamageCoeff API is not supported for this vehicle");
    }

    virtual void setLockedProp(bool locked[4])
    {
        unused(locked);
        throw VehicleCommandNotImplementedException("setLockedProp API is not supported for this vehicle");
    }

    virtual void setLockedPropellerCoeff(float locked_coeff[4])
    {
        unused(locked_coeff);
        throw VehicleCommandNotImplementedException("setLockedPropellerCoeff API is not supported for this vehicle");
    }

    virtual float* getDamageCoeff()
    {
        throw VehicleCommandNotImplementedException("getDamageCoeff API is not supported for this vehicle");
    }

    virtual bool* getLockedProp()
    {
        throw VehicleCommandNotImplementedException("getLockedPropeller API is not supported for this vehicle");
    }

    virtual float* getLockedPropellerCoeff()
    {
        throw VehicleCommandNotImplementedException("getLockedPropellerCoeff API is not supported for this vehicle");
    }

    virtual float* getPWMs()
    {
        throw VehicleCommandNotImplementedException("getPWMs API is not supported for this vehicle");
    }

    virtual size_t getActuatorCount() const
    {
        throw VehicleCommandNotImplementedException("getActuatorCount API is not supported for this vehicle");
    }

    virtual void getStatusMessages(std::vector<std::string>& messages)
    {
        unused(messages);
        //default implementation
    }

    /*
    For RCs, there are two cases: (1) vehicle may be configured to use
    RC bound to its hardware (2) vehicle may be configured to get RC data
    supplied via API calls. Below two APIs are not symmetrical, i.e.,
    getRCData() may or may not return same thing as setRCData().
    */
    //get reading from RC bound to vehicle (if unsupported then RCData::is_valid = false)
    virtual RCData getRCData() const
    {
        static const RCData invalid_rc_data {};
        return invalid_rc_data;
    }

    //set external RC data to vehicle (if unsupported then returns false)
    virtual bool setRCData(const RCData& rc_data)
    {
        unused(rc_data);
        return false;
    }

    // Sensors APIs
    virtual const SensorCollection& getSensors() const
    {
        throw VehicleCommandNotImplementedException("getSensors API is not supported for this vehicle");
    }

    // Lidar APIs
    virtual LidarData getLidarData(const std::string& lidar_name) const
    {
        auto *lidar = static_cast<const LidarBase*>(findSensorByName(lidar_name, SensorBase::SensorType::Lidar));
        if (lidar == nullptr)
            throw VehicleControllerException(Utils::stringf("No lidar with name %s exist on vehicle", lidar_name.c_str()));

        return lidar->getOutput();
    }

    // IMU API
    virtual ImuBase::Output getImuData(const std::string& imu_name) const
    {
        auto *imu = static_cast<const ImuBase*>(findSensorByName(imu_name, SensorBase::SensorType::Imu));
        if (imu == nullptr)
            throw VehicleControllerException(Utils::stringf("No IMU with name %s exist on vehicle", imu_name.c_str()));

        return imu->getOutput();
    }

    // Barometer API
    virtual BarometerBase::Output getBarometerData(const std::string& barometer_name) const
    {
        auto *barometer = static_cast<const BarometerBase*>(findSensorByName(barometer_name, SensorBase::SensorType::Barometer));
        if (barometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No barometer with name %s exist on vehicle", barometer_name.c_str()));

        return barometer->getOutput();
    }

    // Magnetometer API
    virtual MagnetometerBase::Output getMagnetometerData(const std::string& magnetometer_name) const
    {
        auto *magnetometer = static_cast<const MagnetometerBase*>(findSensorByName(magnetometer_name, SensorBase::SensorType::Magnetometer));
        if (magnetometer == nullptr)
            throw VehicleControllerException(Utils::stringf("No magnetometer with name %s exist on vehicle", magnetometer_name.c_str()));

        return magnetometer->getOutput();
    }

    // Gps API
    virtual GpsBase::Output getGpsData(const std::string& gps_name) const
    {
        auto *gps = static_cast<const GpsBase*>(findSensorByName(gps_name, SensorBase::SensorType::Gps));
        if (gps == nullptr)
            throw VehicleControllerException(Utils::stringf("No gps with name %s exist on vehicle", gps_name.c_str()));

        return gps->getOutput();
    }

    // Distance Sensor API
    virtual DistanceSensorData getDistanceSensorData(const std::string& distance_sensor_name) const
    {
        auto *distance_sensor = static_cast<const DistanceBase*>(findSensorByName(distance_sensor_name, SensorBase::SensorType::Distance));
        if (distance_sensor == nullptr)
            throw VehicleControllerException(Utils::stringf("No distance sensor with name %s exist on vehicle", distance_sensor_name.c_str()));

        return distance_sensor->getOutput();
    }

    virtual ~VehicleApiBase() = default;

    //exceptions
    class VehicleControllerException : public std::runtime_error {
    public:
        VehicleControllerException(const std::string& message)
            : runtime_error(message) {
        }
    };

    class VehicleCommandNotImplementedException : public VehicleControllerException {
    public:
        VehicleCommandNotImplementedException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    class VehicleMoveException : public VehicleControllerException {
    public:
        VehicleMoveException(const std::string& message)
            : VehicleControllerException(message) {
        }
    };

    private:
    const SensorBase* findSensorByName(const std::string& sensor_name, const SensorBase::SensorType type) const
    {
        const SensorBase* sensor = nullptr;

        // Find sensor with the given name (for empty input name, return the first one found)
        // Not efficient but should suffice given small number of sensors
        uint count_sensors = getSensors().size(type);
        for (uint i = 0; i < count_sensors; i++)
        {
            const SensorBase* current_sensor = getSensors().getByType(type, i);
            if (current_sensor != nullptr && (current_sensor->getName() == sensor_name || sensor_name == ""))
            {
                sensor = current_sensor;
                break;
            }
        }

        return sensor;
    }
};


}} //namespace
#endif
