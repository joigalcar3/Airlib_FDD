// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

//in header only mode, control library is not available
#ifndef AIRLIB_HEADER_ONLY

#include "vehicles/multirotor/api/MultirotorApiBase.hpp"
#include <functional>
#include <exception>
#include <vector>
#include <iostream>
#include <fstream>

namespace msr { namespace airlib {

void MultirotorApiBase::resetImplementation()
{
    next_path_loc_obj.position = Vector3r(0,0,-0.146495879f);
    yaw_ref_deg = 0;
    cancelLastTask();
    SingleTaskCall lock(this); //cancel previous tasks
}

bool MultirotorApiBase::takeoff(float timeout_sec)
{
    SingleTaskCall lock(this);

    auto kinematics = getKinematicsEstimated();
    if (kinematics.twist.linear.norm() > approx_zero_vel_) { 
        throw VehicleMoveException(Utils::stringf(
            "Cannot perform takeoff because vehicle is already moving with velocity %f m/s",
            kinematics.twist.linear.norm()));
    }

    bool ret = moveToPosition(kinematics.pose.position.x(),
        kinematics.pose.position.y(), kinematics.pose.position.z() + getTakeoffZ(),
        0.5f, timeout_sec, DrivetrainType::MaxDegreeOfFreedom, YawMode::Zero(), -1, 1);

    //last command is to hold on to position
    //commandPosition(0, 0, getTakeoffZ(), YawMode::Zero());

    return ret;
}

bool MultirotorApiBase::land(float timeout_sec)
{
    SingleTaskCall lock(this);

    //after landing we detect if drone has stopped moving
    int near_zero_vel_count = 0;

    return waitForFunction([&]() {
        moveByVelocityInternal(0, 0, landing_vel_, YawMode::Zero());

        float z_vel = getVelocity().z();
        if (z_vel <= approx_zero_vel_)
            ++near_zero_vel_count;
        else
            near_zero_vel_count = 0;

        if (near_zero_vel_count > 10)
            return true;
        else {
            moveByVelocityInternal(0, 0, landing_vel_, YawMode::Zero());
            return false;
        }
    }, timeout_sec).isComplete();
}

bool MultirotorApiBase::goHome(float timeout_sec)
{
    SingleTaskCall lock(this);

    return moveToPosition(0, 0, 0, 0.5f, timeout_sec, DrivetrainType::MaxDegreeOfFreedom, YawMode::Zero(), -1, 1);
}

bool MultirotorApiBase::dummyprinter(float numerito)
{
    SingleTaskCall lock(this);
    if (numerito > 2.0)
        return true;
    else
        return false;
}

// --------------------------------------------------------------------------------------------------------------------------------------------------------
// The following functions connect the commands coming from the client with the corresponding functions in SimpleFlightApi.hpp. 
// Additionally, it interacts with the attributes in MultirotorApiBase.hpp.
// It basically connects the Python code with the C++ functions
// Change the path location object
void MultirotorApiBase::setNextPathLocObj(Pose pose)
{
    next_path_loc_obj.position.x() = pose.position.x();
    next_path_loc_obj.position.y() = pose.position.y();
    next_path_loc_obj.position.z() = pose.position.z();
}

// Methods related to the general activation of plot data gathering
void MultirotorApiBase::setPlotDataCollectionActivation(bool activation)
{
    setPlotDataCollectionAct(activation);
}

// General methods for data collection
void MultirotorApiBase::setActivation(bool activation, float sample_rate, std::string data_name)
{
    setAct(activation, sample_rate, data_name);
}

// Methods related to the reference position data gathering
void MultirotorApiBase::setPosRefActivation(bool activation, float sample_rate)
{
    setPosRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPosRefStoredData()
{
    cleanPosRefSD();
}

PosRefStoredData MultirotorApiBase::getPosRefStoredDataVec()
{
    std::vector<std::vector<float>> pos_ref_data = getPosRefStoredData();
    std::vector<float> pos_ref_x;
    std::vector<float> pos_ref_y;
    std::vector<float> pos_ref_z;

    for (int i = 0; i < pos_ref_data.size(); i++)
    {
        pos_ref_x.push_back(pos_ref_data[i][0]);
        pos_ref_y.push_back(pos_ref_data[i][1]);
        pos_ref_z.push_back(pos_ref_data[i][2]);
    }

    PosRefStoredData dc = PosRefStoredData(pos_ref_x, pos_ref_y, pos_ref_z);
    return dc;
}

// Methods related to the position error data gathering
void MultirotorApiBase::setPosErrorActivation(bool activation, float sample_rate)
{
    setPosErrorAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPosErrorStoredData()
{
    cleanPosErrorSD();
}

PosErrorStoredData MultirotorApiBase::getPosErrorStoredDataVec()
{
    std::vector<std::vector<float>> pos_error_data = getPosErrorStoredData();
    std::vector<float> pos_error_x;
    std::vector<float> pos_error_y;
    std::vector<float> pos_error_z;

    for (int i = 0; i < pos_error_data.size(); i++)
    {
        pos_error_x.push_back(pos_error_data[i][0]);
        pos_error_y.push_back(pos_error_data[i][1]);
        pos_error_z.push_back(pos_error_data[i][2]);
    }

    PosErrorStoredData dc = PosErrorStoredData(pos_error_x, pos_error_y, pos_error_z);
    return dc;
}

// Methods related to the position error derivative data gathering
void MultirotorApiBase::setPosErrorDotActivation(bool activation, float sample_rate)
{
    setPosErrorDotAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPosErrorDotStoredData()
{
    cleanPosErrorDotSD();
}

PosErrorDotStoredData MultirotorApiBase::getPosErrorDotStoredDataVec()
{
    std::vector<std::vector<float>> pos_error_dot_data = getPosErrorDotStoredData();
    std::vector<float> pos_error_dot_x;
    std::vector<float> pos_error_dot_y;
    std::vector<float> pos_error_dot_z;

    for (int i = 0; i < pos_error_dot_data.size(); i++)
    {
        pos_error_dot_x.push_back(pos_error_dot_data[i][0]);
        pos_error_dot_y.push_back(pos_error_dot_data[i][1]);
        pos_error_dot_z.push_back(pos_error_dot_data[i][2]);
    }

    PosErrorDotStoredData dc = PosErrorDotStoredData(pos_error_dot_x, pos_error_dot_y, pos_error_dot_z);
    return dc;
}

// Methods related to the reference velocity data gathering
void MultirotorApiBase::setVelRefActivation(bool activation, float sample_rate)
{
    setVelRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanVelRefStoredData()
{
    cleanVelRefSD();
}

VelRefStoredData MultirotorApiBase::getVelRefStoredDataVec()
{
    std::vector<std::vector<float>> vel_ref_data = getVelRefStoredData();
    std::vector<float> vel_ref_x;
    std::vector<float> vel_ref_y;
    std::vector<float> vel_ref_z;

    for (int i = 0; i < vel_ref_data.size(); i++)
    {
        vel_ref_x.push_back(vel_ref_data[i][0]);
        vel_ref_y.push_back(vel_ref_data[i][1]);
        vel_ref_z.push_back(vel_ref_data[i][2]);
    }

    VelRefStoredData dc = VelRefStoredData(vel_ref_x, vel_ref_y, vel_ref_z);
    return dc;
}

// Methods related to the velocity data gathering
void MultirotorApiBase::setVelActivation(bool activation, float sample_rate)
{
    setVelAct(activation, sample_rate);
}

void MultirotorApiBase::cleanVelStoredData()
{
    cleanVelSD();
}

VelStoredData MultirotorApiBase::getVelStoredDataVec()
{
    std::vector<std::vector<float>> vel_data = getVelStoredData();
    std::vector<float> vel_x;
    std::vector<float> vel_y;
    std::vector<float> vel_z;

    for (int i = 0; i < vel_data.size(); i++)
    {
        vel_x.push_back(vel_data[i][0]);
        vel_y.push_back(vel_data[i][1]);
        vel_z.push_back(vel_data[i][2]);
    }

    VelStoredData dc = VelStoredData(vel_x, vel_y, vel_z);
    return dc;
}

// Methods related to the reference acceleration data gathering
void MultirotorApiBase::setAccRefActivation(bool activation, float sample_rate)
{
    setAccRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanAccRefStoredData()
{
    cleanAccRefSD();
}

AccRefStoredData MultirotorApiBase::getAccRefStoredDataVec()
{
    std::vector<std::vector<float>> acc_ref_data = getAccRefStoredData();
    std::vector<float> acc_ref_x;
    std::vector<float> acc_ref_y;
    std::vector<float> acc_ref_z;

    for (int i = 0; i < acc_ref_data.size(); i++)
    {
        acc_ref_x.push_back(acc_ref_data[i][0]);
        acc_ref_y.push_back(acc_ref_data[i][1]);
        acc_ref_z.push_back(acc_ref_data[i][2]);
    }

    AccRefStoredData dc = AccRefStoredData(acc_ref_x, acc_ref_y, acc_ref_z);
    return dc;
}

// Methods related to the yaw transfer function data gathering
void MultirotorApiBase::setYawTransferFcnActivation(bool activation, float sample_rate)
{
    setYawTransferFcnAct(activation, sample_rate);
}

void MultirotorApiBase::cleanYawTransferFcnStoredData()
{
    cleanYawTransferFcnSD();
}

YawTransferFcnStoredData MultirotorApiBase::getYawTransferFcnStoredDataVec()
{
    std::vector<std::vector<float>> yaw_transfer_fcn_data = getYawTransferFcnStoredData();
    std::vector<float> yaw_transfer_fcn_3;
    std::vector<float> yaw_transfer_fcn_1;
    std::vector<float> yaw_transfer_fcn_1_1;

    for (int i = 0; i < yaw_transfer_fcn_data.size(); i++)
    {
        yaw_transfer_fcn_3.push_back(yaw_transfer_fcn_data[i][0]);
        yaw_transfer_fcn_1.push_back(yaw_transfer_fcn_data[i][1]);
        yaw_transfer_fcn_1_1.push_back(yaw_transfer_fcn_data[i][2]);
    }

    YawTransferFcnStoredData dc = YawTransferFcnStoredData(yaw_transfer_fcn_3, yaw_transfer_fcn_1, yaw_transfer_fcn_1_1);
    return dc;
}

// Methods related to the reference rotational rates data gathering
void MultirotorApiBase::setPqrRefActivation(bool activation, float sample_rate)
{
    setPqrRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPqrRefStoredData()
{
    cleanPqrRefSD();
}

PqrRefStoredData MultirotorApiBase::getPqrRefStoredDataVec()
{
    std::vector<std::vector<float>> pqr_ref_data = getPqrRefStoredData();
    std::vector<float> pqr_ref_x;
    std::vector<float> pqr_ref_y;
    std::vector<float> pqr_ref_z;

    for (int i = 0; i < pqr_ref_data.size(); i++)
    {
        pqr_ref_x.push_back(pqr_ref_data[i][0]);
        pqr_ref_y.push_back(pqr_ref_data[i][1]);
        pqr_ref_z.push_back(pqr_ref_data[i][2]);
    }

    PqrRefStoredData dc = PqrRefStoredData(pqr_ref_x, pqr_ref_y, pqr_ref_z);
    return dc;
}

// Methods related to the rotational rates data gathering
void MultirotorApiBase::setPqrActivation(bool activation, float sample_rate)
{
    setPqrAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPqrStoredData()
{
    cleanPqrSD();
}

PqrStoredData MultirotorApiBase::getPqrStoredDataVec()
{
    std::vector<std::vector<float>> pqr_data = getPqrStoredData();
    std::vector<float> pqr_x;
    std::vector<float> pqr_y;
    std::vector<float> pqr_z;

    for (int i = 0; i < pqr_data.size(); i++)
    {
        pqr_x.push_back(pqr_data[i][0]);
        pqr_y.push_back(pqr_data[i][1]);
        pqr_z.push_back(pqr_data[i][2]);
    }

    PqrStoredData dc = PqrStoredData(pqr_x, pqr_y, pqr_z);
    return dc;
}

// Methods related to the reference thrust data gathering
void MultirotorApiBase::setThrustRefActivation(bool activation, float sample_rate)
{
    setThrustRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanThrustRefStoredData()
{
    cleanThrustRefSD();
}

ThrustRefStoredData MultirotorApiBase::getThrustRefStoredDataVec()
{
    std::vector<std::vector<float>> thrust_ref_data = getThrustRefStoredData();
    std::vector<float> current_thrust_ref_fb;
    std::vector<float> current_thrust_ref_ff;

    for (int i = 0; i < thrust_ref_data.size(); i++)
    {
        current_thrust_ref_fb.push_back(thrust_ref_data[i][0]);
        current_thrust_ref_ff.push_back(thrust_ref_data[i][1]);
    }

    ThrustRefStoredData dc = ThrustRefStoredData(current_thrust_ref_fb, current_thrust_ref_ff);
    return dc;
}

// Methods related to the omegas (motor rotations) data gathering
void MultirotorApiBase::setOmegasActivation(bool activation, float sample_rate)
{
    setOmegasAct(activation, sample_rate);
}

void MultirotorApiBase::cleanOmegasStoredData()
{
    cleanOmegasSD();
}

OmegasStoredData MultirotorApiBase::getOmegasStoredDataVec()
{
    std::vector<std::vector<float>> omegas_data = getOmegasStoredData();
    std::vector<float> front_left;
    std::vector<float> front_right;
    std::vector<float> back_right;
    std::vector<float> back_left;

    for (int i = 0; i < omegas_data.size(); i++)
    {
        front_left.push_back(omegas_data[i][0]);
        front_right.push_back(omegas_data[i][1]);
        back_right.push_back(omegas_data[i][2]);
        back_left.push_back(omegas_data[i][3]);
    }

    OmegasStoredData dc = OmegasStoredData(front_left, front_right, back_right, back_left);
    return dc;
}

// Methods related to the reference yaw data gathering
void MultirotorApiBase::setYawRefActivation(bool activation, float sample_rate)
{
    setYawRefAct(activation, sample_rate);
}

void MultirotorApiBase::cleanYawRefStoredData()
{
    cleanYawRefSD();
}

YawRefStoredData MultirotorApiBase::getYawRefStoredDataVec()
{
    std::vector<std::vector<float>> yaw_ref_data = getYawRefStoredData();
    std::vector<float> yaw_ref;
    std::vector<float> yaw_ref_corrected;

    for (int i = 0; i < yaw_ref_data.size(); i++)
    {
        yaw_ref.push_back(yaw_ref_data[i][0]);
        yaw_ref_corrected.push_back(yaw_ref_data[i][1]);
    }

    YawRefStoredData dc = YawRefStoredData(yaw_ref, yaw_ref_corrected);
    return dc;
}

// Methods related to the orientation data gathering
void MultirotorApiBase::setOrientationActivation(bool activation, float sample_rate)
{
    setOrientationAct(activation, sample_rate);
}

void MultirotorApiBase::cleanOrientationStoredData()
{
    cleanOrientationSD();
}

OrientationStoredData MultirotorApiBase::getOrientationStoredDataVec()
{
    std::vector<std::vector<float>> orientation_data = getOrientationStoredData();
    std::vector<float> orientation_x;
    std::vector<float> orientation_y;
    std::vector<float> orientation_z;

    for (int i = 0; i < orientation_data.size(); i++)
    {
        orientation_x.push_back(orientation_data[i][0]);
        orientation_y.push_back(orientation_data[i][1]);
        orientation_z.push_back(orientation_data[i][2]);
    }

    OrientationStoredData dc = OrientationStoredData(orientation_x, orientation_y, orientation_z);
    return dc;
}

// Methods related to the position integrator data gathering
void MultirotorApiBase::setPositionIntegratorActivation(bool activation, float sample_rate)
{
    setPositionIntegratorAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPositionIntegratorStoredData()
{
    cleanPositionIntegratorSD();
}

PositionIntegratorStoredData MultirotorApiBase::getPositionIntegratorStoredDataVec()
{
    std::vector<std::vector<float>> position_integrator_data = getPositionIntegratorStoredData();
    std::vector<float> position_integrator_x;
    std::vector<float> position_integrator_y;
    std::vector<float> position_integrator_z;

    for (int i = 0; i < position_integrator_data.size(); i++)
    {
        position_integrator_x.push_back(position_integrator_data[i][0]);
        position_integrator_y.push_back(position_integrator_data[i][1]);
        position_integrator_z.push_back(position_integrator_data[i][2]);
    }

    PositionIntegratorStoredData dc = PositionIntegratorStoredData(position_integrator_x, position_integrator_y, position_integrator_z);
    return dc;
}


// Methods related to the thrust PI controller data gathering
void MultirotorApiBase::setThrustPiActivation(bool activation, float sample_rate)
{
    setThrustPiAct(activation, sample_rate);
}

void MultirotorApiBase::cleanThrustPiStoredData()
{
    cleanThrustPiSD();
}

ThrustPiStoredData MultirotorApiBase::getThrustPiStoredDataVec()
{
    std::vector<std::vector<float>> thrust_PI_data = getThrustPiStoredData();
    std::vector<float> thrust_P;
    std::vector<float> thrust_I;

    for (int i = 0; i < thrust_PI_data.size(); i++)
    {
        thrust_P.push_back(thrust_PI_data[i][0]);
        thrust_I.push_back(thrust_PI_data[i][1]);
    }

    ThrustPiStoredData dc = ThrustPiStoredData(thrust_P, thrust_I);
    return dc;
}

// Methods related to the damaged mass forces
void MultirotorApiBase::setDamagedMassForcesActivation(bool activation, float sample_rate)
{
    setDamagedMassForcesAct(activation, sample_rate);
}

void MultirotorApiBase::cleanDamagedMassForcesStoredData()
{
    cleanDamagedMassForcesSD();
}

DamagedMassForcesStoredData MultirotorApiBase::getDamagedMassForcesStoredDataVec()
{
    std::vector<std::vector<float>> damaged_mass_forces_data = getDamagedMassForcesStoredData();
    std::vector<float> damaged_mass_forces_x;
    std::vector<float> damaged_mass_forces_y;
    std::vector<float> damaged_mass_forces_z;

    for (int i = 0; i < damaged_mass_forces_data.size(); i++)
    {
        damaged_mass_forces_x.push_back(damaged_mass_forces_data[i][0]);
        damaged_mass_forces_y.push_back(damaged_mass_forces_data[i][1]);
        damaged_mass_forces_z.push_back(damaged_mass_forces_data[i][2]);
    }

    DamagedMassForcesStoredData dc = DamagedMassForcesStoredData(damaged_mass_forces_x, damaged_mass_forces_y, damaged_mass_forces_z);
    return dc;
}

// Methods related to the damaged mass moments
void MultirotorApiBase::setDamagedMassMomentsActivation(bool activation, float sample_rate)
{
    setDamagedMassMomentsAct(activation, sample_rate);
}

void MultirotorApiBase::cleanDamagedMassMomentsStoredData()
{
    cleanDamagedMassMomentsSD();
}

DamagedMassMomentsStoredData MultirotorApiBase::getDamagedMassMomentsStoredDataVec()
{
    std::vector<std::vector<float>> damaged_mass_moments_data = getDamagedMassMomentsStoredData();
    std::vector<float> damaged_mass_moments_x;
    std::vector<float> damaged_mass_moments_y;
    std::vector<float> damaged_mass_moments_z;

    for (int i = 0; i < damaged_mass_moments_data.size(); i++)
    {
        damaged_mass_moments_x.push_back(damaged_mass_moments_data[i][0]);
        damaged_mass_moments_y.push_back(damaged_mass_moments_data[i][1]);
        damaged_mass_moments_z.push_back(damaged_mass_moments_data[i][2]);
    }

    DamagedMassMomentsStoredData dc = DamagedMassMomentsStoredData(damaged_mass_moments_x, damaged_mass_moments_y, damaged_mass_moments_z);
    return dc;
}

// Methods related to the damaged aero forces
void MultirotorApiBase::setDamagedAeroForcesActivation(bool activation, float sample_rate)
{
    setDamagedAeroForcesAct(activation, sample_rate);
}

void MultirotorApiBase::cleanDamagedAeroForcesStoredData()
{
    cleanDamagedAeroForcesSD();
}

DamagedAeroForcesStoredData MultirotorApiBase::getDamagedAeroForcesStoredDataVec()
{
    std::vector<std::vector<float>> damaged_aero_forces_data = getDamagedAeroForcesStoredData();
    std::vector<float> damaged_aero_forces_x;
    std::vector<float> damaged_aero_forces_y;
    std::vector<float> damaged_aero_forces_z;

    for (int i = 0; i < damaged_aero_forces_data.size(); i++)
    {
        damaged_aero_forces_x.push_back(damaged_aero_forces_data[i][0]);
        damaged_aero_forces_y.push_back(damaged_aero_forces_data[i][1]);
        damaged_aero_forces_z.push_back(damaged_aero_forces_data[i][2]);
    }

    DamagedAeroForcesStoredData dc = DamagedAeroForcesStoredData(damaged_aero_forces_x, damaged_aero_forces_y, damaged_aero_forces_z);
    return dc;
}

// Methods related to the damaged aero moments
void MultirotorApiBase::setDamagedAeroMomentsActivation(bool activation, float sample_rate)
{
    setDamagedAeroMomentsAct(activation, sample_rate);
}

void MultirotorApiBase::cleanDamagedAeroMomentsStoredData()
{
    cleanDamagedAeroMomentsSD();
}

DamagedAeroMomentsStoredData MultirotorApiBase::getDamagedAeroMomentsStoredDataVec()
{
    std::vector<std::vector<float>> damaged_aero_moments_data = getDamagedAeroMomentsStoredData();
    std::vector<float> damaged_aero_moments_x;
    std::vector<float> damaged_aero_moments_y;
    std::vector<float> damaged_aero_moments_z;

    for (int i = 0; i < damaged_aero_moments_data.size(); i++)
    {
        damaged_aero_moments_x.push_back(damaged_aero_moments_data[i][0]);
        damaged_aero_moments_y.push_back(damaged_aero_moments_data[i][1]);
        damaged_aero_moments_z.push_back(damaged_aero_moments_data[i][2]);
    }

    DamagedAeroMomentsStoredData dc = DamagedAeroMomentsStoredData(damaged_aero_moments_x, damaged_aero_moments_y, damaged_aero_moments_z);
    return dc;
}

// Methods related to the time data gathering
void MultirotorApiBase::setTimeInfoActivation(bool activation, float sample_rate)
{
    setTimeInfoAct(activation, sample_rate);
}

void MultirotorApiBase::cleanTimeInfoStoredData()
{
    cleanTimeInfoSD();
}

TimeInfoStoredData MultirotorApiBase::getTimeInfoStoredDataVec()
{
    std::vector<std::vector<float>> time_data = getTimeInfoStoredData();
    std::vector<float> time;
    std::vector<float> dt_real;
    std::vector<float> sampling_frequency;

    for (int i = 0; i < time_data.size(); i++)
    {
        time.push_back(time_data[i][0]);
        dt_real.push_back(time_data[i][1]);
        sampling_frequency.push_back(time_data[i][2]);
    }

    TimeInfoStoredData dc = TimeInfoStoredData(time, dt_real, sampling_frequency);
    return dc;
}

// Methods related to the camera data gathering
void MultirotorApiBase::setCameraActivation(bool activation, float sample_rate, const std::vector<ImageCaptureBase::ImageRequest>& request, VehicleSimApiBase* const& api)
{
    setCameraAct(activation, sample_rate, request, api);
}

void MultirotorApiBase::cleanCameraStoredData()
{
    cleanCameraSD();
}

void MultirotorApiBase::saveCameraStoredData(std::string path)
{
    saveCameraSD(path);
}

// Methods related to the IMU data gathering
void MultirotorApiBase::setImuActivation(bool activation, float sample_rate)
{
    setIMUAct(activation, sample_rate);
}

void MultirotorApiBase::cleanImuStoredData()
{
    cleanIMUSD();
}

IMUStoredData MultirotorApiBase::getImuStoredDataVec()
{
    std::vector<msr::airlib::ImuBase::Output> imu_data = getIMUStoredData();
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

    for (int i = 0; i < imu_data.size(); i++)
    {
        timestamps.push_back(imu_data[i].time_stamp);
        orientations_w.push_back(imu_data[i].orientation.w());
        orientations_x.push_back(imu_data[i].orientation.x());
        orientations_y.push_back(imu_data[i].orientation.y());
        orientations_z.push_back(imu_data[i].orientation.z());
        angular_velocity_x.push_back(imu_data[i].angular_velocity.x());
        angular_velocity_y.push_back(imu_data[i].angular_velocity.y());
        angular_velocity_z.push_back(imu_data[i].angular_velocity.z());
        linear_acceleration_x.push_back(imu_data[i].linear_acceleration.x());
        linear_acceleration_y.push_back(imu_data[i].linear_acceleration.y());
        linear_acceleration_z.push_back(imu_data[i].linear_acceleration.z());
    }

    IMUStoredData dc = IMUStoredData(timestamps, orientations_w, orientations_x, orientations_y, orientations_z, angular_velocity_x,
        angular_velocity_y, angular_velocity_z, linear_acceleration_x, linear_acceleration_y, linear_acceleration_z);
    return dc;
}

// Methods related to the PWMs data gathering
void MultirotorApiBase::setPWMActivation(bool activation, float sample_rate)
{
    setPWMAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPWMStoredData()
{
    cleanPWMSD();
}

PWMStoredData MultirotorApiBase::getPWMStoredDataVec()
{
    std::vector<std::vector<float>> PWM_data = getPWMStoredData();
    std::vector<float> PWM_1;
    std::vector<float> PWM_2;
    std::vector<float> PWM_3;
    std::vector<float> PWM_4;

    for (int i = 0; i < PWM_data.size(); i++)
    {
        PWM_1.push_back(PWM_data[i][0]);
        PWM_2.push_back(PWM_data[i][1]);
        PWM_3.push_back(PWM_data[i][2]);
        PWM_4.push_back(PWM_data[i][3]);
    }

    PWMStoredData dc = PWMStoredData(PWM_1, PWM_2, PWM_3, PWM_4);
    return dc;
}

// Methods related to the ground truth position data gathering
void MultirotorApiBase::setPositionActivation(bool activation, float sample_rate)
{
    setPositionAct(activation, sample_rate);
}

void MultirotorApiBase::cleanPositionStoredData()
{
    cleanPositionSD();
}

PositionStoredData MultirotorApiBase::getPositionStoredDataVec()
{
    std::vector<std::vector<float>> position_data = getPositionStoredData();
    std::vector<float> positions_x;
    std::vector<float> positions_y;
    std::vector<float> positions_z;

    for (int i = 0; i < position_data.size(); i++)
    {
        positions_x.push_back(position_data[i][0]);
        positions_y.push_back(position_data[i][1]);
        positions_z.push_back(position_data[i][2]);
    }

    PositionStoredData dc = PositionStoredData(positions_x, positions_y, positions_z);
    return dc;
}

// Methods related to the barometer data gathering
void MultirotorApiBase::setBarometerActivation(bool activation, float sample_rate)
{
    setBarometerAct(activation, sample_rate);
}

void MultirotorApiBase::cleanBarometerStoredData()
{
    cleanBarometerSD();
}

BarometerStoredData MultirotorApiBase::getBarometerStoredDataVec()
{
    std::vector<msr::airlib::BarometerBase::Output> barometer_data = getBarometerStoredData();
    std::vector<uint64_t> timestamps;
    std::vector<float> altitudes;
    std::vector<float> pressures;
    std::vector<float> qnhs;

    for (int i = 0; i < barometer_data.size(); i++)
    {
        timestamps.push_back(barometer_data[i].time_stamp);
        altitudes.push_back(barometer_data[i].altitude);
        pressures.push_back(barometer_data[i].pressure);
        qnhs.push_back(barometer_data[i].qnh);
    }

    BarometerStoredData dc = BarometerStoredData(timestamps, altitudes, pressures, qnhs);
    return dc;
}

// Methods related to the magnetometer data gathering
void MultirotorApiBase::setMagnetometerActivation(bool activation, float sample_rate)
{
    setMagnetometerAct(activation, sample_rate);
}

void MultirotorApiBase::cleanMagnetometerStoredData()
{
    cleanMagnetometerSD();
}

MagnetometerStoredData MultirotorApiBase::getMagnetometerStoredDataVec()
{
    std::vector<msr::airlib::MagnetometerBase::Output> magnetometer_data = getMagnetometerStoredData();
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

    for (int i = 0; i < magnetometer_data.size(); i++)
    {
        timestamps.push_back(magnetometer_data[i].time_stamp);
        magnetic_field_body_x.push_back(magnetometer_data[i].magnetic_field_body.x());
        magnetic_field_body_y.push_back(magnetometer_data[i].magnetic_field_body.y());
        magnetic_field_body_z.push_back(magnetometer_data[i].magnetic_field_body.z());
        if (magnetometer_data[i].magnetic_field_covariance.size() == 0)
        {
            magnetic_field_covariance_1.push_back(0);
            magnetic_field_covariance_2.push_back(0);
            magnetic_field_covariance_3.push_back(0);
            magnetic_field_covariance_4.push_back(0);
            magnetic_field_covariance_5.push_back(0);
            magnetic_field_covariance_6.push_back(0);
            magnetic_field_covariance_7.push_back(0);
            magnetic_field_covariance_8.push_back(0);
            magnetic_field_covariance_9.push_back(0);
        }
        else
        {
            magnetic_field_covariance_1.push_back(magnetometer_data[i].magnetic_field_covariance[0]);
            magnetic_field_covariance_2.push_back(magnetometer_data[i].magnetic_field_covariance[1]);
            magnetic_field_covariance_3.push_back(magnetometer_data[i].magnetic_field_covariance[2]);
            magnetic_field_covariance_4.push_back(magnetometer_data[i].magnetic_field_covariance[3]);
            magnetic_field_covariance_5.push_back(magnetometer_data[i].magnetic_field_covariance[4]);
            magnetic_field_covariance_6.push_back(magnetometer_data[i].magnetic_field_covariance[5]);
            magnetic_field_covariance_7.push_back(magnetometer_data[i].magnetic_field_covariance[6]);
            magnetic_field_covariance_8.push_back(magnetometer_data[i].magnetic_field_covariance[7]);
            magnetic_field_covariance_9.push_back(magnetometer_data[i].magnetic_field_covariance[8]);
        }
    }

    MagnetometerStoredData dc = MagnetometerStoredData(timestamps, magnetic_field_body_x, magnetic_field_body_y, magnetic_field_body_z, magnetic_field_covariance_1,
        magnetic_field_covariance_2, magnetic_field_covariance_3, magnetic_field_covariance_4, magnetic_field_covariance_5, magnetic_field_covariance_6,
        magnetic_field_covariance_7, magnetic_field_covariance_8, magnetic_field_covariance_9);
    return dc;
}

// Methods related to the GPS data gathering
void MultirotorApiBase::setGPSActivation(bool activation, float sample_rate)
{
    setGPSAct(activation, sample_rate);
}

void MultirotorApiBase::cleanGPSStoredData()
{
    cleanGPSSD();
}

GPSStoredData MultirotorApiBase::getGPSStoredDataVec()
{
    std::vector<msr::airlib::GpsBase::Output> GPS_data = getGPSStoredData();
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

    for (int i = 0; i < GPS_data.size(); i++)
    {
        timestamps.push_back(GPS_data[i].time_stamp);
        geo_point_altitude.push_back(GPS_data[i].gnss.geo_point.altitude);
        geo_point_latitude.push_back(GPS_data[i].gnss.geo_point.latitude);
        geo_point_longitude.push_back(GPS_data[i].gnss.geo_point.longitude);
        ephs.push_back(GPS_data[i].gnss.eph);
        epvs.push_back(GPS_data[i].gnss.epv);
        velocity_x.push_back(GPS_data[i].gnss.velocity.x());
        velocity_y.push_back(GPS_data[i].gnss.velocity.y());
        velocity_z.push_back(GPS_data[i].gnss.velocity.z());
        fix_types.push_back(GPS_data[i].gnss.fix_type);
        time_utcs.push_back(GPS_data[i].gnss.time_utc);
        is_valids.push_back(GPS_data[i].is_valid);
    }

    GPSStoredData dc = GPSStoredData(timestamps, geo_point_altitude, geo_point_latitude, geo_point_longitude, ephs,
        epvs, velocity_x, velocity_y, velocity_z, fix_types,
        time_utcs, is_valids);
    return dc;
}

// Methods related to the drone teleportation
void MultirotorApiBase::setTeleportYawRef(float yaw_angle_ref)
{
    setTeleportYawReference(yaw_angle_ref);
}

// Methods related to the drone failures
void MultirotorApiBase::setDamageCoefficientAdvanced(int propeller, int blade, float damage_coefficient, float start_angle)
{
    setDamageCoeffAdvanced(propeller, blade, damage_coefficient, start_angle);
}

void MultirotorApiBase::resetDamageCoefficientAdvanced()
{
    resetDamageCoeffAdvanced();
}

void MultirotorApiBase::setSwitchActivateBladeDamageAdvanced(bool switch_activate_blade_damage_advanced)
{
    setSwitchActBladeDamageAdvanced(switch_activate_blade_damage_advanced);
}

void MultirotorApiBase::setDamageCoefficients(float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4)
{
    float new_coeffs[4] = {new_coeff_1, new_coeff_2, new_coeff_3, new_coeff_4};

    setDamageCoeff(new_coeffs);
}

void MultirotorApiBase::setLockedProppellers(bool locked_1, bool locked_2, bool locked_3, bool locked_4)
{
    bool new_coeffs[4] = { locked_1, locked_2, locked_3, locked_4 };

    setLockedProp(new_coeffs);
}

void MultirotorApiBase::setLockedPropellerCoefficients(float new_coeff_1, float new_coeff_2, float new_coeff_3, float new_coeff_4)
{
    float new_coeffs[4] = { new_coeff_1, new_coeff_2, new_coeff_3, new_coeff_4 };

    setLockedPropellerCoeff(new_coeffs);
}

DamageCoefficients MultirotorApiBase::getDamageCoefficients()
{
    float* p;
    p = getDamageCoeff();
    DamageCoefficients dc = DamageCoefficients(*(p), *(p+1), *(p+2), *(p+3));
    return dc;
}

LockedPropellers MultirotorApiBase::getLockedPropellers()
{
    bool* p;
    p = getLockedProp();
    LockedPropellers dc = LockedPropellers(*(p), *(p + 1), *(p + 2), *(p + 3));
    return dc;
}

DamageCoefficients MultirotorApiBase::getLockedPropellerCoefficients()
{
    float* p;
    p = getLockedPropellerCoeff();
    DamageCoefficients dc = DamageCoefficients(*(p), *(p + 1), *(p + 2), *(p + 3));
    return dc;
}

DamageCoefficients MultirotorApiBase::getMotorPWMs()
{
    float* p;
    p = getPWMs();
    DamageCoefficients dc = DamageCoefficients(*(p), *(p + 1), *(p + 2), *(p + 3));
    return dc;
}
// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

bool MultirotorApiBase::moveByVelocityBodyFrame(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(getKinematicsEstimated().pose.orientation, pitch, roll, yaw);
    float vx_new = (vx * (float)std::cos(yaw)) - (vy * (float)std::sin(yaw));
    float vy_new = (vx * (float)std::sin(yaw)) + (vy * (float)std::cos(yaw));
    
    YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
    adjustYaw(vx_new, vy_new, drivetrain, adj_yaw_mode);

    return waitForFunction([&]() {
        moveByVelocityInternal(vx_new, vy_new, vz, adj_yaw_mode);
        return false; //keep moving until timeout
        }, duration).isTimeout();
}

bool MultirotorApiBase::moveByVelocityZBodyFrame(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    float pitch, roll, yaw;
    VectorMath::toEulerianAngle(getKinematicsEstimated().pose.orientation, pitch, roll, yaw);
    float vx_new = (vx * (float)std::cos(yaw)) - (vy * (float)std::sin(yaw));
    float vy_new = (vx * (float)std::sin(yaw)) + (vy * (float)std::cos(yaw));

    YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
    adjustYaw(vx_new, vy_new, drivetrain, adj_yaw_mode);

    return waitForFunction([&]() {
        moveByVelocityZInternal(vx_new, vy_new, z, adj_yaw_mode);
        return false; //keep moving until timeout
        }, duration).isTimeout();
}

bool MultirotorApiBase::moveByMotorPWMs(float front_right_pwm, float rear_left_pwm, float front_left_pwm, float rear_right_pwm, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        commandMotorPWMs(front_right_pwm, rear_left_pwm, front_left_pwm, rear_right_pwm);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}


bool MultirotorApiBase::moveByRollPitchYawZ(float roll, float pitch, float yaw, float z, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByRollPitchYawZInternal(roll, pitch, yaw, z);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByRollPitchYawThrottle(float roll, float pitch, float yaw, float throttle, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByRollPitchYawThrottleInternal(roll, pitch, yaw, throttle);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByRollPitchYawrateThrottle(float roll, float pitch, float yaw_rate, float throttle, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByRollPitchYawrateThrottleInternal(roll, pitch, yaw_rate, throttle);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByRollPitchYawrateZ(float roll, float pitch, float yaw_rate, float z, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByRollPitchYawrateZInternal(roll, pitch, yaw_rate, z);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByAngleRatesZ(float roll_rate, float pitch_rate, float yaw_rate, float z, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByAngleRatesZInternal(roll_rate, pitch_rate, yaw_rate, z);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByAngleRatesThrottle(float roll_rate, float pitch_rate, float yaw_rate, float throttle, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    return waitForFunction([&]() {
        moveByAngleRatesThrottleInternal(roll_rate, pitch_rate, yaw_rate, throttle);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByVelocity(float vx, float vy, float vz, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
    adjustYaw(vx, vy, drivetrain, adj_yaw_mode);

    return waitForFunction([&]() {
        moveByVelocityInternal(vx, vy, vz, adj_yaw_mode);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveByVelocityZ(float vx, float vy, float z, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return false;

    YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
    adjustYaw(vx, vy, drivetrain, adj_yaw_mode);

    return waitForFunction([&]() {
        moveByVelocityZInternal(vx, vy, z, adj_yaw_mode);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

bool MultirotorApiBase::moveOnPath(const vector<Vector3r>& path, float velocity, float timeout_sec, DrivetrainType drivetrain, const YawMode& yaw_mode,
    float lookahead, float adaptive_lookahead)
{
    SingleTaskCall lock(this);
    adaptive_lookahead_var = adaptive_lookahead;
    lookahead_var = lookahead;

    //validate path size
    if (path.size() == 0) {
        Utils::log("moveOnPath terminated because path has no points", Utils::kLogLevelWarn);
        return true;
    }

    //validate yaw mode
    if (drivetrain == DrivetrainType::ForwardOnly && yaw_mode.is_rate)
        throw std::invalid_argument("Yaw cannot be specified as rate if drivetrain is ForwardOnly");

    //validate and set auto-lookahead value
    float command_period_dist = velocity * getCommandPeriod();
    if (lookahead == 0)
        throw std::invalid_argument("lookahead distance cannot be 0"); //won't allow progress on path
    else if (lookahead > 0) {
        if (command_period_dist > lookahead)
            throw std::invalid_argument(Utils::stringf("lookahead value %f is too small for velocity %f. It must be at least %f", lookahead, velocity, command_period_dist));
        if (getDistanceAccuracy() > lookahead)
            throw std::invalid_argument(Utils::stringf("lookahead value %f is smaller than drone's distance accuracy %f.", lookahead, getDistanceAccuracy()));
    }
    else {
        //if auto mode requested for lookahead then calculate based on velocity
        lookahead = getAutoLookahead(velocity, adaptive_lookahead);
        Utils::log(Utils::stringf("lookahead = %f, adaptive_lookahead = %f", lookahead, adaptive_lookahead));        
    }

    //add current position as starting point
    vector<Vector3r> path3d;
    vector<PathSegment> path_segs;
    path3d.push_back(getKinematicsEstimated().pose.position);

    Vector3r point;
    float path_length = 0;

    //append the input path and compute segments
    // path_segs is a vector of path_segs that contain information about the origin and goal points of the segment, the desired flight velocity and the length of the segment
    for(uint i = 0; i < path.size(); ++i) {
        point = path.at(i);
        PathSegment path_seg(path3d.at(i), point, velocity, path_length);
        path_length += path_seg.seg_length;
        path_segs.push_back(path_seg);
        path3d.push_back(point);
    }
    //add last segment as zero length segment so we have equal number of segments and points. 
    //path_segs[i] refers to segment that starts at point i
    path_segs.push_back(PathSegment(point, point, velocity, path_length));

    //when path ends, we want to slow down
    float breaking_dist = 0;
    if (velocity > getMultirotorApiParams().breaking_vel) {
        breaking_dist = Utils::clip(velocity * getMultirotorApiParams().vel_to_breaking_dist, 
            getMultirotorApiParams().min_breaking_dist, getMultirotorApiParams().max_breaking_dist);
    }
    //else no need to change velocities for last segments

    //setup current position on path to 0 offset
    cur_path_loc_obj.seg_index = 0;
    cur_path_loc_obj.offset = 0;
    cur_path_loc_obj.position = path3d[0];

    float lookahead_error_increasing = 0;
    float lookahead_error = 0;
    Waiter waiter(getCommandPeriod(), timeout_sec, getCancelToken());

    //initialize next path position
    setNextPathPosition(path3d, path_segs, cur_path_loc_obj, lookahead + lookahead_error, next_path_loc_obj);
    float overshoot = 0;
    float goal_dist = 0;

    //until we are at the end of the path (last seg is always zero size)
    while (!waiter.isTimeout() && (next_path_loc_obj.seg_index < path_segs.size()-1 || goal_dist > 0)
        ) { //current position is approximately at the last end point

        float seg_velocity = path_segs.at(next_path_loc_obj.seg_index).seg_velocity;
        float path_length_remaining = path_length - path_segs.at(cur_path_loc_obj.seg_index).seg_path_length - cur_path_loc_obj.offset;
        if (seg_velocity > getMultirotorApiParams().min_vel_for_breaking && path_length_remaining <= breaking_dist) {
            seg_velocity = getMultirotorApiParams().breaking_vel;
            //Utils::logMessage("path_length_remaining = %f, Switched to breaking vel %f", path_length_remaining, seg_velocity);
        }

        //send drone command to get to next lookahead
        moveToPathPosition(next_path_loc_obj.position, seg_velocity, drivetrain, 
            yaw_mode, path_segs.at(cur_path_loc_obj.seg_index).start_z);

        //sleep for rest of the cycle
        if (!waiter.sleep())
            return false;

        /*  Below, P is previous position on path, N is next goal and C is our current position.

        N
        ^
        |
        |
        |
        C'|---C
        |  /
        | /
        |/
        P

        Note that PC could be at any angle relative to PN, including 0 or -ve. We increase lookahead distance
        by the amount of |PC|. For this, we project PC on to PN to get vector PC' and length of
        CC'is our adaptive lookahead error by which we will increase lookahead distance. 

        For next iteration, we first update our current position by goal_dist and then
        set next goal by the amount lookahead + lookahead_error.

        We need to take care of following cases:

        1. |PN| == 0 => lookahead_error = |PC|, goal_dist = 0
        2. |PC| == 0 => lookahead_error = 0, goal_dist = 0
        3. PC in opposite direction => lookahead_error = |PC|, goal_dist = 0

        One good test case is if C just keeps moving perpendicular to the path (instead of along the path).
        In that case, we expect next goal to come up and down by the amount of lookahead_error. However
        under no circumstances we should go back on the path (i.e. current pos on path can only move forward).
        */

        //how much have we moved towards last goal?
        // Goal vect is the distance between the next point and the previous point before it moved.
        const Vector3r& goal_vect = next_path_loc_obj.position - cur_path_loc_obj.position;

        if (!goal_vect.isZero()) { //goal can only be zero if we are at the end of path
            // actual_vect is the distance between the current location and the previous point before it moved
            const Vector3r& actual_vect = getPosition() - cur_path_loc_obj.position;

            //project actual vector on goal vector
            // goal normalized is the direction that the drone had to be followed from the previous point before it moved
            const Vector3r& goal_normalized = goal_vect.normalized();    

            // goal_dist is the distance that the drone has flown in the direction determined by the next point and the previous point before it moved
            goal_dist = actual_vect.dot(goal_normalized); //dist could be -ve if drone moves away from goal

            //if adaptive lookahead is enabled the calculate lookahead error (see above fig)
            if (adaptive_lookahead) {
                const Vector3r& actual_on_goal = goal_normalized * goal_dist;
                float error = (actual_vect - actual_on_goal).norm() * adaptive_lookahead;
                if (error > lookahead_error) {
                    lookahead_error_increasing++;
                    //TODO: below should be lower than 1E3 and configurable
                    //but lower values like 100 doesn't work for simple_flight + ScalableClock
                    if (lookahead_error_increasing > 1E5) {
                        throw std::runtime_error("lookahead error is continually increasing so we do not have safe control, aborting moveOnPath operation");
                    }
                }
                else { 
                    lookahead_error_increasing = 0; 
                }
                lookahead_error = error;
            }
        }
        else {
            lookahead_error_increasing = 0;
            goal_dist = 0;
            lookahead_error = 0; //this is not really required because we will exit
            waiter.complete();
        }

        // Utils::logMessage("PF: cur=%s, goal_dist=%f, cur_path_loc=%s, next_path_loc_obj=%s, lookahead_error=%f",
        //     VectorMath::toString(getPosition()).c_str(), goal_dist, VectorMath::toString(cur_path_loc.position).c_str(),
        //     VectorMath::toString(next_path_loc_obj.position).c_str(), lookahead_error);

        //if drone moved backward, we don't want goal to move backward as well
        //so only climb forward on the path, never back. Also note >= which means
        //we climb path even if distance was 0 to take care of duplicated points on path
        // This is done to update the current position of the object
        if (goal_dist >= 0) {
            overshoot = setNextPathPosition(path3d, path_segs, cur_path_loc_obj, goal_dist, cur_path_loc_obj);
            if (overshoot)
                Utils::log(Utils::stringf("overshoot=%f", overshoot));
        }

        //compute next target on path
        overshoot = setNextPathPosition(path3d, path_segs, cur_path_loc_obj, lookahead + lookahead_error, next_path_loc_obj);
    }

    return waiter.isComplete();
}

bool MultirotorApiBase::moveToPosition(float x, float y, float z, float velocity, float timeout_sec, DrivetrainType drivetrain,
    const YawMode& yaw_mode, float lookahead, float adaptive_lookahead)
{
    SingleTaskCall lock(this);

    vector<Vector3r> path{ Vector3r(x, y, z) };
    return moveOnPath(path, velocity, timeout_sec, drivetrain, yaw_mode, lookahead, adaptive_lookahead);
}

bool MultirotorApiBase::moveToZ(float z, float velocity, float timeout_sec, const YawMode& yaw_mode,
    float lookahead, float adaptive_lookahead)
{
    SingleTaskCall lock(this);

    Vector2r cur_xy(getPosition().x(), getPosition().y());
    vector<Vector3r> path { Vector3r(cur_xy.x(), cur_xy.y(), z) };
    return moveOnPath(path, velocity, timeout_sec, DrivetrainType::MaxDegreeOfFreedom, yaw_mode, lookahead, adaptive_lookahead);
}

bool MultirotorApiBase::moveByManual(float vx_max, float vy_max, float z_min, float duration, DrivetrainType drivetrain, const YawMode& yaw_mode)
{
    SingleTaskCall lock(this);

    const float kMaxMessageAge = 0.1f /* 0.1 sec */, kMaxRCValue = 10000;

    if (duration <= 0)
        return true;

    //freeze the quaternion
    Quaternionr starting_quaternion = getKinematicsEstimated().pose.orientation;

    Waiter waiter(getCommandPeriod(), duration, getCancelToken());
    do {

        RCData rc_data = getRCData();
        TTimeDelta age = clock()->elapsedSince(rc_data.timestamp);
        if (rc_data.is_valid && (rc_data.timestamp == 0 || age <= kMaxMessageAge)) { //if rc message timestamp is not set OR is not too old 
            if (rc_data_trims_.is_valid)
                rc_data.subtract(rc_data_trims_);

            //convert RC commands to velocity vector
            const Vector3r vel_word(rc_data.pitch * vy_max / kMaxRCValue, rc_data.roll  * vx_max / kMaxRCValue, 0);
            Vector3r vel_body = VectorMath::transformToBodyFrame(vel_word, starting_quaternion, true);

            //find yaw as per terrain and remote setting
            YawMode adj_yaw_mode(yaw_mode.is_rate, yaw_mode.yaw_or_rate);
            adj_yaw_mode.yaw_or_rate += rc_data.yaw * 100.0f / kMaxRCValue;
            adjustYaw(vel_body, drivetrain, adj_yaw_mode);

            //execute command
            try {
                float vz = (rc_data.throttle / kMaxRCValue) * z_min + getPosition().z();
                moveByVelocityZInternal(vel_body.x(), vel_body.y(), vz, adj_yaw_mode);
            }
            catch (const MultirotorApiBase::UnsafeMoveException& ex) {
                Utils::log(Utils::stringf("Safety violation: %s", ex.result.message.c_str()), Utils::kLogLevelWarn);
            }
        }
        else
            Utils::log(Utils::stringf("RCData had too old timestamp: %f", age));

    } while (waiter.sleep());

    //if timeout occurred then command completed successfully otherwise it was interrupted
    return waiter.isTimeout();
}

bool MultirotorApiBase::rotateToYaw(float yaw, float timeout_sec, float margin)
{
    SingleTaskCall lock(this);

    if (timeout_sec <= 0)
        return true;

    auto start_pos = getPosition();
    float yaw_target = VectorMath::normalizeAngle(yaw);
    YawMode move_yaw_mode(false, yaw_target);
    YawMode stop_yaw_mode(true, 0);

    return waitForFunction([&]() {
        if (isYawWithinMargin(yaw_target, margin)) { // yaw is within margin, then trying to stop rotation
            moveToPositionInternal(start_pos, stop_yaw_mode); // let yaw rate be zero
            auto yaw_rate = getKinematicsEstimated().twist.angular.z();
            if (abs(yaw_rate) <= approx_zero_angular_vel_) { // already sopped
                return true; //stop all for stably achieving the goal
            }
        }
        else { // yaw is not within margin, go on rotation
            moveToPositionInternal(start_pos, move_yaw_mode);
        }

        // yaw is not within margin
        return false; //keep moving until timeout
    }, timeout_sec).isComplete();
}

bool MultirotorApiBase::rotateByYawRate(float yaw_rate, float duration)
{
    SingleTaskCall lock(this);

    if (duration <= 0)
        return true;

    auto start_pos = getPosition();
    YawMode yaw_mode(true, yaw_rate);
    
    return waitForFunction([&]() {
        moveToPositionInternal(start_pos, yaw_mode);
        return false; //keep moving until timeout
    }, duration).isTimeout();
}

void MultirotorApiBase::setAngleLevelControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) 
{
    uint8_t controller_type = 2;
    setControllerGains(controller_type, kp, ki, kd);
}

void MultirotorApiBase::setAngleRateControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) 
{
    uint8_t controller_type = 3;
    setControllerGains(controller_type, kp, ki, kd);
}

void MultirotorApiBase::setVelocityControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) 
{
    uint8_t controller_type = 4;
    setControllerGains(controller_type, kp, ki, kd);
}

void MultirotorApiBase::setPositionControllerGains(const vector<float>& kp, const vector<float>& ki, const vector<float>& kd) 
{
    uint8_t controller_type = 5;
    setControllerGains(controller_type, kp, ki, kd);
}

bool MultirotorApiBase::hover()
{
    SingleTaskCall lock(this);

    return moveToZ(getPosition().z(), 0.5f, Utils::max<float>(), YawMode{ true,0 }, 1.0f, false);
}

void MultirotorApiBase::moveByRC(const RCData& rc_data)
{
    unused(rc_data);
    //by default we say that this command is not supported
    throw VehicleCommandNotImplementedException("moveByRC API is not implemented for this multirotor");
}

void MultirotorApiBase::moveByVelocityInternal(float vx, float vy, float vz, const YawMode& yaw_mode)
{
    if (safetyCheckVelocity(Vector3r(vx, vy, vz)))
        commandVelocity(vx, vy, vz, yaw_mode);
}

void MultirotorApiBase::moveByVelocityZInternal(float vx, float vy, float z, const YawMode& yaw_mode)
{
    if (safetyCheckVelocityZ(vx, vy, z))
        commandVelocityZ(vx, vy, z, yaw_mode);
}

void MultirotorApiBase::moveToPositionInternal(const Vector3r& dest, const YawMode& yaw_mode)
{
    if (safetyCheckDestination(dest))
        commandPosition(dest.x(), dest.y(), dest.z(), yaw_mode);
}

void MultirotorApiBase::moveByRollPitchYawZInternal(float roll, float pitch, float yaw, float z)
{
    if (safetyCheckVelocity(getVelocity()))
        commandRollPitchYawZ(roll, pitch, yaw, z);
}

void MultirotorApiBase::moveByRollPitchYawThrottleInternal(float roll, float pitch, float yaw, float throttle)
{
    if (safetyCheckVelocity(getVelocity()))
        commandRollPitchYawThrottle(roll, pitch, yaw, throttle);
}

void MultirotorApiBase::moveByRollPitchYawrateThrottleInternal(float roll, float pitch, float yaw_rate, float throttle)
{
    if (safetyCheckVelocity(getVelocity()))
        commandRollPitchYawrateThrottle(roll, pitch, yaw_rate, throttle);
}

void MultirotorApiBase::moveByRollPitchYawrateZInternal(float roll, float pitch, float yaw_rate, float z)
{
    if (safetyCheckVelocity(getVelocity()))
        commandRollPitchYawrateZ(roll, pitch, yaw_rate, z);
}

void MultirotorApiBase::moveByAngleRatesZInternal(float roll_rate, float pitch_rate, float yaw_rate, float z)
{
    if (safetyCheckVelocity(getVelocity()))
        commandAngleRatesZ(roll_rate, pitch_rate, yaw_rate, z);
}

void MultirotorApiBase::moveByAngleRatesThrottleInternal(float roll_rate, float pitch_rate, float yaw_rate, float throttle)
{
    if (safetyCheckVelocity(getVelocity()))
        commandAngleRatesThrottle(roll_rate, pitch_rate, yaw_rate, throttle);
}

//executes a given function until it returns true. Each execution is spaced apart at command period.
//return value is true if exit was due to given function returning true, otherwise false (due to timeout)
Waiter MultirotorApiBase::waitForFunction(WaitFunction function, float timeout_sec)
{
    Waiter waiter(getCommandPeriod(), timeout_sec, getCancelToken());
    if (timeout_sec <= 0)
        return waiter;

    do {
        if (function()) {
            waiter.complete();
            break;
        }
    }
    while (waiter.sleep());
    return waiter;
}

bool MultirotorApiBase::waitForZ(float timeout_sec, float z, float margin)
{
    float cur_z = 100000;
    return waitForFunction([&]() {
        cur_z = getPosition().z();
        return (std::abs(cur_z - z) <= margin);
    }, timeout_sec).isComplete();
}

void MultirotorApiBase::setSafetyEval(const shared_ptr<SafetyEval> safety_eval_ptr)
{
    SingleCall lock(this);
    safety_eval_ptr_ = safety_eval_ptr;
}

RCData MultirotorApiBase::estimateRCTrims(float trimduration, float minCountForTrim, float maxTrim)
{
    rc_data_trims_ = RCData();

    //get trims
    Waiter waiter_trim(getCommandPeriod(), trimduration, getCancelToken());
    uint count = 0;
    do {

        const RCData rc_data = getRCData();
        if (rc_data.is_valid) {
            rc_data_trims_.add(rc_data);
            count++;
        }

    } while (waiter_trim.sleep());

    rc_data_trims_.is_valid = true;


    if (count < minCountForTrim) {
        rc_data_trims_.is_valid = false;
        Utils::log("Cannot compute RC trim because too few readings received");
    }

    //take average
    rc_data_trims_.divideBy(static_cast<float>(count));
    if (rc_data_trims_.isAnyMoreThan(maxTrim)) {
        rc_data_trims_.is_valid = false;
        Utils::log(Utils::stringf("RC trims does not seem to be valid: %s", rc_data_trims_.toString().c_str()));
    }

    Utils::log(Utils::stringf("RCData Trims: %s", rc_data_trims_.toString().c_str()));

    return rc_data_trims_;
}

void MultirotorApiBase::moveToPathPosition(const Vector3r& dest, float velocity, DrivetrainType drivetrain, /* pass by value */ YawMode yaw_mode, float last_z)
{
    unused(last_z);
    //validate dest
    if (dest.hasNaN())
        throw std::invalid_argument(VectorMath::toString(dest, "dest vector cannot have NaN: "));

    //what is the distance we will travel at this velocity?
    float expected_dist = velocity * getCommandPeriod();

    //get velocity vector
    const Vector3r cur = getPosition();
    const Vector3r cur_dest = dest - cur;
    float cur_dest_norm = cur_dest.norm();

    //yaw for the direction of travel
    adjustYaw(cur_dest, drivetrain, yaw_mode);
    yaw_ref_deg = yaw_mode.yaw_or_rate;

    //find velocity vector
    Vector3r velocity_vect;
    if (cur_dest_norm < getDistanceAccuracy())  //our dest is approximately same as current
        velocity_vect = Vector3r::Zero();
    else if (cur_dest_norm >= expected_dist) {
        velocity_vect = (cur_dest / cur_dest_norm) * velocity;
    }
    else { //cur dest is too close than the distance we would travel
           //generate velocity vector that is same size as cur_dest_norm / command period
           //this velocity vect when executed for command period would yield cur_dest_norm
        Utils::log(Utils::stringf("Too close dest: cur_dest_norm=%f, expected_dist=%f", cur_dest_norm, expected_dist));
        velocity_vect = (cur_dest / cur_dest_norm) * (cur_dest_norm / getCommandPeriod());
    }

    //send commands
    //try to maintain altitude if path was in XY plan only, velocity based control is not as good
    if (std::abs(cur.z() - dest.z()) <= getDistanceAccuracy()) //for paths in XY plan current code leaves z untouched, so we can compare with strict equality
        moveByVelocityInternal(velocity_vect.x(), velocity_vect.y(), 0, yaw_mode);
    else
        moveByVelocityInternal(velocity_vect.x(), velocity_vect.y(), velocity_vect.z(), yaw_mode);
}

bool MultirotorApiBase::setSafety(SafetyEval::SafetyViolationType enable_reasons, float obs_clearance, SafetyEval::ObsAvoidanceStrategy obs_startegy,
    float obs_avoidance_vel, const Vector3r& origin, float xy_length, float max_z, float min_z)
{
    if (safety_eval_ptr_ == nullptr)
        throw std::invalid_argument("The setSafety call requires safety_eval_ptr_ to be set first");

    //default strategy is for move. In hover mode we set new strategy temporarily
    safety_eval_ptr_->setSafety(enable_reasons, obs_clearance, obs_startegy, origin, xy_length, max_z, min_z);

    obs_avoidance_vel_ = obs_avoidance_vel;
    Utils::log(Utils::stringf("obs_avoidance_vel: %f", obs_avoidance_vel_));

    return true;
}

bool MultirotorApiBase::emergencyManeuverIfUnsafe(const SafetyEval::EvalResult& result)
{
    if (!result.is_safe) {
        if (result.reason == SafetyEval::SafetyViolationType_::Obstacle) {
            //are we supposed to do EM?
            if (safety_eval_ptr_->getObsAvoidanceStrategy() != SafetyEval::ObsAvoidanceStrategy::RaiseException) {
                //get suggested velocity vector
                Vector3r avoidance_vel = getObsAvoidanceVelocity(result.cur_risk_dist, obs_avoidance_vel_) * result.suggested_vec;

                //use the unchecked command
                commandVelocityZ(avoidance_vel.x(), avoidance_vel.y(), getPosition().z(), YawMode::Zero());

                //tell caller not to execute planned command
                return false;
            }
            //other wise throw exception
        }
        //otherwise there is some other reason why we are in unsafe situation
        //send last command to come to full stop
        commandVelocity(0, 0, 0, YawMode::Zero());
        throw UnsafeMoveException(result);
    }
    //else no unsafe situation

    return true;
}

bool MultirotorApiBase::safetyCheckVelocity(const Vector3r& velocity)
{
    if (safety_eval_ptr_ == nullptr) //safety checks disabled
        return true;

    const auto& result = safety_eval_ptr_->isSafeVelocity(getPosition(), velocity, getOrientation());
    return emergencyManeuverIfUnsafe(result);
}
bool MultirotorApiBase::safetyCheckVelocityZ(float vx, float vy, float z)
{
    if (safety_eval_ptr_ == nullptr) //safety checks disabled
        return true;

    const auto& result = safety_eval_ptr_->isSafeVelocityZ(getPosition(), vx, vy, z, getOrientation());
    return emergencyManeuverIfUnsafe(result);
}
bool MultirotorApiBase::safetyCheckDestination(const Vector3r& dest_pos)
{
    if (safety_eval_ptr_ == nullptr) //safety checks disabled
        return true;

    const auto& result = safety_eval_ptr_->isSafeDestination(getPosition(), dest_pos, getOrientation());
    return emergencyManeuverIfUnsafe(result);
}    

float MultirotorApiBase::setNextPathPosition(const vector<Vector3r>& path, const vector<PathSegment>& path_segs,
    const PathPosition& cur_path_loc, float next_dist, PathPosition& next_path_loc)
{
    //note: cur_path_loc and next_path_loc may both point to same object
    uint i = cur_path_loc.seg_index;
    float offset = cur_path_loc.offset;
    while (i < path.size() - 1) {
        const PathSegment& seg = path_segs.at(i);
        if (seg.seg_length > 0 && //protect against duplicate points in path, normalized seg will have NaN
            seg.seg_length >= next_dist + offset) {

            next_path_loc.seg_index = i;
            next_path_loc.offset = next_dist + offset;  //how much total distance we will travel on this segment
            next_path_loc.position = path.at(i) + seg.seg_normalized * next_path_loc.offset;
            return 0;
        }
        //otherwise use up this segment, move on to next one
        next_dist -= seg.seg_length - offset;
        offset = 0;

        if (&cur_path_loc == &next_path_loc)
            Utils::log(Utils::stringf("segment %d done: x=%f, y=%f, z=%f", i, path.at(i).x(), path.at(i).y(), path.at(i).z()));

        ++i;
    }

    //if we are here then we ran out of segments
    //consider last segment as zero length segment
    next_path_loc.seg_index = i;
    next_path_loc.offset = 0;
    next_path_loc.position = path.at(i);
    return next_dist;
}

void MultirotorApiBase::adjustYaw(const Vector3r& heading, DrivetrainType drivetrain, YawMode& yaw_mode)
{
    //adjust yaw for the direction of travel in forward-only mode
    if (drivetrain == DrivetrainType::ForwardOnly && !yaw_mode.is_rate) {
        if (heading.norm() > getDistanceAccuracy()) {
            yaw_mode.yaw_or_rate = yaw_mode.yaw_or_rate + (std::atan2(heading.y(), heading.x()) * 180 / M_PIf);
            yaw_mode.yaw_or_rate = VectorMath::normalizeAngle(yaw_mode.yaw_or_rate);
        }
        else
            yaw_mode.setZeroRate(); //don't change existing yaw if heading is too small because that can generate random result
    }
    //else no adjustment needed
}

void MultirotorApiBase::adjustYaw(float x, float y, DrivetrainType drivetrain, YawMode& yaw_mode) {
    adjustYaw(Vector3r(x, y, 0), drivetrain, yaw_mode);
}

bool MultirotorApiBase::isYawWithinMargin(float yaw_target, float margin) const
{
    const float yaw_current = VectorMath::getYaw(getOrientation()) * 180 / M_PIf;
    return std::abs(yaw_current - yaw_target) <= margin;
}

float MultirotorApiBase::getAutoLookahead(float velocity, float adaptive_lookahead,
        float max_factor, float min_factor) const
{
    //if auto mode requested for lookahead then calculate based on velocity
    float command_period_dist = velocity * getCommandPeriod();
    float lookahead = command_period_dist * (adaptive_lookahead > 0 ? min_factor : max_factor);
    lookahead = std::max(lookahead, getDistanceAccuracy()*1.5f); //50% more than distance accuracy
    return lookahead;
}

float MultirotorApiBase::getObsAvoidanceVelocity(float risk_dist, float max_obs_avoidance_vel) const
{
    unused(risk_dist);
    return max_obs_avoidance_vel;
}

}} //namespace
#endif
