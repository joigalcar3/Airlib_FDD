// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file ControllerDataScoping.hpp
 *
 * @brief Provides the ControllerDataScoping, class for the storage of data for scoping from the controller.
 * Its a file in progress since the end goal is to migrate all the scoping functions from SimpleFlightApi.hpp 
 * to the current file, "ControllerDataScoping.hpp". It is still unfinished and in the developer's TODO list.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: jialvear@hotmail.com
 *
 */

#ifndef msr_airlib_ControllerDataScoping_hpp
#define msr_airlib_ControllerDataScoping_hpp

#include "sensors/SensorCollection.hpp"
#include <Eigen/Dense>
#include "common/Common.hpp"
#include <math.h>  

#include "vehicles/multirotor/MultiRotorParams.hpp"



namespace msr {
    namespace airlib {
        class ControllerDataScoping {
        public:
            ControllerDataScoping() 
            {


            }

            // Function in charge of activating or deactivating the data gathering of a particular signal
            void setAct(bool activation, float sample_rate, std::string data_name)
            {
                if (data_name == "PosRef")
                {
                    setPosRefAct(activation, sample_rate);
                }
                else if (data_name == "PosError")
                {
                    setPosErrorAct(activation, sample_rate);
                }
                else if (data_name == "PosErrorDot")
                {
                    setPosErrorDotAct(activation, sample_rate);
                }
                else if (data_name == "VelReef")
                {
                    setVelRefAct(activation, sample_rate);
                }
                else if (data_name == "Vel")
                {
                    setVelAct(activation, sample_rate);
                }
                else if (data_name == "AccRef")
                {
                    setAccRefAct(activation, sample_rate);
                }
                else if (data_name == "YawTransferFcn")
                {
                    setYawTransferFcnAct(activation, sample_rate);
                }
                else if (data_name == "PqrRef")
                {
                    setPqrRefAct(activation, sample_rate);
                }
                else if (data_name == "Pqr")
                {
                    setPqrAct(activation, sample_rate);
                }
                else if (data_name == "ThrustRef")
                {
                    setThrustRefAct(activation, sample_rate);
                }
                else if (data_name == "Omegas")
                {
                    setOmegasAct(activation, sample_rate);
                }
                else if (data_name == "YawRef")
                {
                    setYawRefAct(activation, sample_rate);
                }
                else if (data_name == "Orientation")
                {
                    setOrientationAct(activation, sample_rate);
                }
                else if (data_name == "PositionIntegrator")
                {
                    setPositionIntegratorAct(activation, sample_rate);
                }
                else if (data_name == "ThrustPi")
                {
                    setThrustPiAct(activation, sample_rate);
                }
                else if (data_name == "DamagedMassForces")
                {
                    setDamagedMassForcesAct(activation, sample_rate);
                }
                else if (data_name == "DamagedMassMoments")
                {
                    setDamagedMassMomentsAct(activation, sample_rate);
                }
                else if (data_name == "DamagedAeroForces")
                {
                    setDamagedAeroForcesAct(activation, sample_rate);
                }
                else if (data_name == "DamagedAeroMoments")
                {
                    setDamagedAeroMomentsAct(activation, sample_rate);
                }
                else if (data_name == "TimeInfo")
                {
                    setTimeInfoAct(activation, sample_rate);
                }
            }

            // Function that calls all methods that collect data
            virtual void collectAllData()
            {
                //local_IMU_data = getImuData("");
                storePosRefData();
                storePosErrorData();
                storePosErrorDotData();
                storeVelRefData();
                storeVelData();
                storeYawTransferFcnData();
                storeAccRefData();
                storePqrRefData();
                storePqrData();
                storeThrustRefData();
                storeOmegasData();
                storeYawRefData();
                storeOrientationData();
                storePositionIntegratorData();
                storeThrustPiData();
                storeDamagedMassForcesData();
                storeDamagedMassMomentsData();
                storeDamagedAeroForcesData();
                storeDamagedAeroMomentsData();
                storeTimeInfoData();
            }

            // Methods related to the activation of general data collection for plotting
            void setPlotDataCollectionAct(bool activation) 
            {
                plot_data_collection_switch = activation;
            }

            // Methods related to the data gathering of position reference
            void storePosRefData() 
            {
                if (pos_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_pos_ref_data = { current_pos_ref.x(), current_pos_ref.y(), current_pos_ref.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pos_ref_threshold = UE4_second / pos_ref_sample_rate;
                    uint64_t time_threshold = pos_ref_time_old + pos_ref_threshold;
                    if (time_new >= time_threshold) {
                        pos_ref_time_old = time_new;
                        pos_ref_data.push_back(local_pos_ref_data);
                    }
                }
            }

            void setPosRefAct(bool activation, float sample_rate) 
            {
                pos_ref_activate_store = activation;
                pos_ref_sample_rate = sample_rate;
            }

            void cleanPosRefSD() 
            {
                pos_ref_activate_store = false;
                pos_ref_data.clear();
            }

            std::vector<std::vector<float>> getPosRefStoredData() 
            {
                return pos_ref_data;
            }

            // Methods related to the data gathering of position reference
            void storePosErrorData() 
            {
                if (pos_error_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_pos_error_data = { current_pos_error.x(), current_pos_error.y(), current_pos_error.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pos_error_threshold = UE4_second / pos_error_sample_rate;
                    uint64_t time_threshold = pos_error_time_old + pos_error_threshold;
                    if (time_new >= time_threshold) {
                        pos_error_time_old = time_new;
                        pos_error_data.push_back(local_pos_error_data);
                    }
                }
            }

            void setPosErrorAct(bool activation, float sample_rate) 
            {
                pos_error_activate_store = activation;
                pos_error_sample_rate = sample_rate;
            }

            void cleanPosErrorSD() 
            {
                pos_error_activate_store = false;
                pos_error_data.clear();
            }

            std::vector<std::vector<float>> getPosErrorStoredData() 
            {
                return pos_error_data;
            }

            // Methods related to the data gathering of position error derivative
            void storePosErrorDotData() 
            {
                if (pos_error_dot_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_pos_error_dot_data = { current_pos_error_dot.x(), current_pos_error_dot.y(), current_pos_error_dot.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pos_error_dot_threshold = UE4_second / pos_error_dot_sample_rate;
                    uint64_t time_threshold = pos_error_dot_time_old + pos_error_dot_threshold;
                    if (time_new >= time_threshold) {
                        pos_error_dot_time_old = time_new;
                        pos_error_dot_data.push_back(local_pos_error_dot_data);
                    }
                }
            }

            void setPosErrorDotAct(bool activation, float sample_rate) 
            {
                pos_error_dot_activate_store = activation;
                pos_error_dot_sample_rate = sample_rate;
            }

            void cleanPosErrorDotSD() 
            {
                pos_error_dot_activate_store = false;
                pos_error_dot_data.clear();
            }

            std::vector<std::vector<float>> getPosErrorDotStoredData() 
            {
                return pos_error_dot_data;
            }

            // Methods related to the data gathering of velocity reference
            void storeVelRefData() 
            {
                if (vel_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_vel_ref_data = { current_vel_ref.x(), current_vel_ref.y(), current_vel_ref.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int vel_ref_threshold = UE4_second / vel_ref_sample_rate;
                    uint64_t time_threshold = vel_ref_time_old + vel_ref_threshold;
                    if (time_new >= time_threshold) {
                        vel_ref_time_old = time_new;
                        vel_ref_data.push_back(local_vel_ref_data);
                    }
                }
            }

            void setVelRefAct(bool activation, float sample_rate) 
            {
                vel_ref_activate_store = activation;
                vel_ref_sample_rate = sample_rate;
            }

            void cleanVelRefSD() 
            {
                vel_ref_activate_store = false;
                vel_ref_data.clear();
            }

            std::vector<std::vector<float>> getVelRefStoredData() 
            {
                return vel_ref_data;
            }

            // Methods related to the data gathering of velocity
            void storeVelData() 
            {
                if (vel_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_vel_data = { current_vel.x(), current_vel.y(), current_vel.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int vel_threshold = UE4_second / vel_sample_rate;
                    uint64_t time_threshold = vel_time_old + vel_threshold;
                    if (time_new >= time_threshold) {
                        vel_time_old = time_new;
                        vel_data.push_back(local_vel_data);
                    }
                }
            }

            void setVelAct(bool activation, float sample_rate) 
            {
                vel_activate_store = activation;
                vel_sample_rate = sample_rate;
            }

            void cleanVelSD() 
            {
                vel_activate_store = false;
                vel_data.clear();
            }

            std::vector<std::vector<float>> getVelStoredData() 
            {
                return vel_data;
            }

            // Methods related to the data gathering of reference acceleration
            void storeAccRefData() 
            {
                if (acc_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_acc_ref_data = { current_acc_ref.x(), current_acc_ref.y(), current_acc_ref.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int acc_ref_threshold = UE4_second / acc_ref_sample_rate;
                    uint64_t time_threshold = acc_ref_time_old + acc_ref_threshold;
                    if (time_new >= time_threshold) {
                        acc_ref_time_old = time_new;
                        acc_ref_data.push_back(local_acc_ref_data);
                    }
                }
            }

            void setAccRefAct(bool activation, float sample_rate) 
            {
                acc_ref_activate_store = activation;
                acc_ref_sample_rate = sample_rate;
            }

            void cleanAccRefSD() 
            {
                acc_ref_activate_store = false;
                acc_ref_data.clear();
            }

            std::vector<std::vector<float>> getAccRefStoredData() 
            {
                return acc_ref_data;
            }

            // Methods related to the data gathering of yaw transfer functions
            void storeYawTransferFcnData() 
            {
                if (yaw_transfer_fcn_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_yaw_transfer_fcn_data = { stored_yaw_transfer_fcn_3, stored_yaw_transfer_fcn_1 , stored_yaw_transfer_fcn_1_1 };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int yaw_transfer_fcn_threshold = UE4_second / yaw_transfer_fcn_sample_rate;
                    uint64_t time_threshold = yaw_transfer_fcn_time_old + yaw_transfer_fcn_threshold;
                    if (time_new >= time_threshold) {
                        yaw_transfer_fcn_time_old = time_new;
                        yaw_transfer_fcn_data.push_back(local_yaw_transfer_fcn_data);
                    }
                }
            }

            void setYawTransferFcnAct(bool activation, float sample_rate) 
            {
                yaw_transfer_fcn_activate_store = activation;
                yaw_transfer_fcn_sample_rate = sample_rate;
            }

            void cleanYawTransferFcnSD() 
            {
                yaw_transfer_fcn_activate_store = false;
                yaw_transfer_fcn_data.clear();
            }

            std::vector<std::vector<float>> getYawTransferFcnStoredData() 
            {
                return yaw_transfer_fcn_data;
            }


            // Methods related to the data gathering of reference rotational rates 
            void storePqrRefData() 
            {
                if (pqr_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_pqr_ref_data = { current_pqr_ref.x(), current_pqr_ref.y(), current_pqr_ref.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pqr_ref_threshold = UE4_second / pqr_ref_sample_rate;
                    uint64_t time_threshold = pqr_ref_time_old + pqr_ref_threshold;
                    if (time_new >= time_threshold) {
                        pqr_ref_time_old = time_new;
                        pqr_ref_data.push_back(local_pqr_ref_data);
                    }
                }
            }

            void setPqrRefAct(bool activation, float sample_rate) 
            {
                pqr_ref_activate_store = activation;
                pqr_ref_sample_rate = sample_rate;
            }

            void cleanPqrRefSD() 
            {
                pqr_ref_activate_store = false;
                pqr_ref_data.clear();
            }

            std::vector<std::vector<float>> getPqrRefStoredData() 
            {
                return pqr_ref_data;
            }

            // Methods related to the data gathering of rotational rates 
            void storePqrData() 
            {
                if (pqr_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_pqr_data = { current_pqr.x(), current_pqr.y(), current_pqr.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pqr_threshold = UE4_second / pqr_sample_rate;
                    uint64_t time_threshold = pqr_time_old + pqr_threshold;
                    if (time_new >= time_threshold) {
                        pqr_time_old = time_new;
                        pqr_data.push_back(local_pqr_data);
                    }
                }
            }

            void setPqrAct(bool activation, float sample_rate) 
            {
                pqr_activate_store = activation;
                pqr_sample_rate = sample_rate;
            }

            void cleanPqrSD() 
            {
                pqr_activate_store = false;
                pqr_data.clear();
            }

            std::vector<std::vector<float>> getPqrStoredData() 
            {
                return pqr_data;
            }

            // Methods related to the data gathering of reference thrust 
            void storeThrustRefData() 
            {
                if (pqr_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_thrust_ref_data = { current_thrust_ref_fb, current_thrust_ref_ff };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int thrust_ref_threshold = UE4_second / thrust_ref_sample_rate;
                    uint64_t time_threshold = thrust_ref_time_old + thrust_ref_threshold;
                    if (time_new >= time_threshold) {
                        thrust_ref_time_old = time_new;
                        thrust_ref_data.push_back(local_thrust_ref_data);
                    }
                }
            }

            void setThrustRefAct(bool activation, float sample_rate) 
            {
                thrust_ref_activate_store = activation;
                thrust_ref_sample_rate = sample_rate;
            }

            void cleanThrustRefSD() 
            {
                thrust_ref_activate_store = false;
                thrust_ref_data.clear();
            }

            std::vector<std::vector<float>> getThrustRefStoredData() 
            {
                return thrust_ref_data;
            }

            // Methods related to the data gathering of reference thrust 
            void storeOmegasData() 
            {
                if (pqr_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_omegas_data = current_omegas;
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int omegas_threshold = UE4_second / omegas_sample_rate;
                    uint64_t time_threshold = omegas_time_old + omegas_threshold;
                    if (time_new >= time_threshold) {
                        omegas_time_old = time_new;
                        omegas_data.push_back(local_omegas_data);
                    }
                }
            }

            void setOmegasAct(bool activation, float sample_rate) 
            {
                omegas_activate_store = activation;
                omegas_sample_rate = sample_rate;
            }

            void cleanOmegasSD() 
            {
                omegas_activate_store = false;
                omegas_data.clear();
            }

            std::vector<std::vector<float>> getOmegasStoredData() 
            {
                return omegas_data;
            }

            // Methods related to the data gathering of yaw reference
            void storeYawRefData() 
            {
                if (yaw_ref_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_yaw_ref_data = { current_yaw_ref, current_corrected_yaw_ref };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int yaw_ref_threshold = UE4_second / yaw_ref_sample_rate;
                    uint64_t time_threshold = yaw_ref_time_old + yaw_ref_threshold;
                    if (time_new >= time_threshold) {
                        yaw_ref_time_old = time_new;
                        yaw_ref_data.push_back(local_yaw_ref_data);
                    }
                }
            }

            void setYawRefAct(bool activation, float sample_rate) 
            {
                yaw_ref_activate_store = activation;
                yaw_ref_sample_rate = sample_rate;
            }

            void cleanYawRefSD() 
            {
                yaw_ref_activate_store = false;
                yaw_ref_data.clear();
            }

            std::vector<std::vector<float>> getYawRefStoredData() 
            {
                return yaw_ref_data;
            }

            // Methods related to the data gathering of orientation data (phi, theta and psi)
            void storeOrientationData() 
            {
                if (orientation_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_orientation_ref_data = { current_orientation.x(),  current_orientation.y(), current_orientation.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int orientation_threshold = UE4_second / orientation_sample_rate;
                    uint64_t time_threshold = orientation_time_old + orientation_threshold;
                    if (time_new >= time_threshold) {
                        orientation_time_old = time_new;
                        orientation_data.push_back(local_orientation_ref_data);
                    }
                }
            }

            void setOrientationAct(bool activation, float sample_rate) 
            {
                orientation_activate_store = activation;
                orientation_sample_rate = sample_rate;
            }

            void cleanOrientationSD() 
            {
                orientation_activate_store = false;
                orientation_data.clear();
            }

            std::vector<std::vector<float>> getOrientationStoredData() 
            {
                return orientation_data;
            }

            // Methods related to the data gathering of the position integrator within the first subsystem of the primary axis INDI controller
            void storePositionIntegratorData() 
            {
                if (position_integrator_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_position_integrator_data = { current_position_integrator.x(),  current_position_integrator.y(), current_position_integrator.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int position_integrator_threshold = UE4_second / position_integrator_sample_rate;
                    uint64_t time_threshold = position_integrator_time_old + position_integrator_threshold;
                    if (time_new >= time_threshold) {
                        position_integrator_time_old = time_new;
                        position_integrator_data.push_back(local_position_integrator_data);
                    }
                }
            }

            void setPositionIntegratorAct(bool activation, float sample_rate) 
            {
                position_integrator_activate_store = activation;
                position_integrator_sample_rate = sample_rate;
            }

            void cleanPositionIntegratorSD() 
            {
                position_integrator_activate_store = false;
                position_integrator_data.clear();
            }

            std::vector<std::vector<float>> getPositionIntegratorStoredData() 
            {
                return position_integrator_data;
            }

            // Methods related to the data gathering of the PI thrust controller within the first subsystem of the primary axis INDI controller
            void storeThrustPiData() 
            {
                if (thrust_PI_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_thrust_PI_data = { current_thrust_P,  current_thrust_I };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int thrust_PI_threshold = UE4_second / thrust_PI_sample_rate;
                    uint64_t time_threshold = thrust_PI_time_old + thrust_PI_threshold;
                    if (time_new >= time_threshold) {
                        thrust_PI_time_old = time_new;
                        thrust_PI_data.push_back(local_thrust_PI_data);
                    }
                }
            }

            void setThrustPiAct(bool activation, float sample_rate) 
            {
                thrust_PI_activate_store = activation;
                thrust_PI_sample_rate = sample_rate;
            }

            void cleanThrustPiSD() 
            {
                thrust_PI_activate_store = false;
                thrust_PI_data.clear();
            }

            std::vector<std::vector<float>> getThrustPiStoredData() 
            {
                return thrust_PI_data;
            }

            // Methods related to the data gathering of the mass forces due to the blade damage
            void storeDamagedMassForcesData() 
            {
                if (damaged_mass_forces_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_damaged_mass_forces_data = { current_damaged_mass_forces.x(),  current_damaged_mass_forces.y(), current_damaged_mass_forces.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int damaged_mass_forces_threshold = UE4_second / damaged_mass_forces_sample_rate;
                    uint64_t time_threshold = damaged_mass_forces_time_old + damaged_mass_forces_threshold;
                    if (time_new >= time_threshold) {
                        damaged_mass_forces_time_old = time_new;
                        damaged_mass_forces_data.push_back(local_damaged_mass_forces_data);
                    }
                }
            }

            void setDamagedMassForcesAct(bool activation, float sample_rate) 
            {
                damaged_mass_forces_activate_store = activation;
                damaged_mass_forces_sample_rate = sample_rate;
            }

            void cleanDamagedMassForcesSD() 
            {
                damaged_mass_forces_activate_store = false;
                damaged_mass_forces_data.clear();
            }

            std::vector<std::vector<float>> getDamagedMassForcesStoredData() 
            {
                return damaged_mass_forces_data;
            }

            // Methods related to the data gathering of the mass moments due to the blade damage
            void storeDamagedMassMomentsData() 
            {
                if (damaged_mass_moments_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_damaged_mass_moments_data = { current_damaged_mass_moments.x(),  current_damaged_mass_moments.y(), current_damaged_mass_moments.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int damaged_mass_moments_threshold = UE4_second / damaged_mass_moments_sample_rate;
                    uint64_t time_threshold = damaged_mass_moments_time_old + damaged_mass_moments_threshold;
                    if (time_new >= time_threshold) {
                        damaged_mass_moments_time_old = time_new;
                        damaged_mass_moments_data.push_back(local_damaged_mass_moments_data);
                    }
                }
            }

            void setDamagedMassMomentsAct(bool activation, float sample_rate) 
            {
                damaged_mass_moments_activate_store = activation;
                damaged_mass_moments_sample_rate = sample_rate;
            }

            void cleanDamagedMassMomentsSD() 
            {
                damaged_mass_moments_activate_store = false;
                damaged_mass_moments_data.clear();
            }

            std::vector<std::vector<float>> getDamagedMassMomentsStoredData() 
            {
                return damaged_mass_moments_data;
            }

            // Methods related to the data gathering of the aero forces due to the blade damage
            void storeDamagedAeroForcesData() 
            {
                if (damaged_aero_forces_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_damaged_aero_forces_data = { current_damaged_aero_forces.x(),  current_damaged_aero_forces.y(), current_damaged_aero_forces.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int damaged_aero_forces_threshold = UE4_second / damaged_aero_forces_sample_rate;
                    uint64_t time_threshold = damaged_aero_forces_time_old + damaged_aero_forces_threshold;
                    if (time_new >= time_threshold) {
                        damaged_aero_forces_time_old = time_new;
                        damaged_aero_forces_data.push_back(local_damaged_aero_forces_data);
                    }
                }
            }

            void setDamagedAeroForcesAct(bool activation, float sample_rate) 
            {
                damaged_aero_forces_activate_store = activation;
                damaged_aero_forces_sample_rate = sample_rate;
            }

            void cleanDamagedAeroForcesSD() 
            {
                damaged_aero_forces_activate_store = false;
                damaged_aero_forces_data.clear();
            }

            std::vector<std::vector<float>> getDamagedAeroForcesStoredData() 
            {
                return damaged_aero_forces_data;
            }

            // Methods related to the data gathering of the aero moments due to the blade damage
            void storeDamagedAeroMomentsData() 
            {
                if (damaged_aero_moments_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_damaged_aero_moments_data = { current_damaged_aero_moments.x(),  current_damaged_aero_moments.y(), current_damaged_aero_moments.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int damaged_aero_moments_threshold = UE4_second / damaged_aero_moments_sample_rate;
                    uint64_t time_threshold = damaged_aero_moments_time_old + damaged_aero_moments_threshold;
                    if (time_new >= time_threshold) {
                        damaged_aero_moments_time_old = time_new;
                        damaged_aero_moments_data.push_back(local_damaged_aero_moments_data);
                    }
                }
            }

            void setDamagedAeroMomentsAct(bool activation, float sample_rate) 
            {
                damaged_aero_moments_activate_store = activation;
                damaged_aero_moments_sample_rate = sample_rate;
            }

            void cleanDamagedAeroMomentsSD() 
            {
                damaged_aero_moments_activate_store = false;
                damaged_aero_moments_data.clear();
            }

            std::vector<std::vector<float>> getDamagedAeroMomentsStoredData() 
            {
                return damaged_aero_moments_data;
            }

            // Methods related to the data gathering of the sampling frequency or the frequency at which the controller is called
            void storeTimeInfoData() 
            {
                if (time_info_activate_store && plot_data_collection_switch) {
                    std::vector<float> local_time_info_data = { current_time,  current_dt_real, 1.0f / current_dt_real };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int time_info_threshold = UE4_second / time_info_sample_rate;
                    uint64_t time_threshold = time_info_time_old + time_info_threshold;
                    if (time_new >= time_threshold) {
                        time_info_time_old = time_new;
                        time_info_data.push_back(local_time_info_data);
                    }
                }
            }

            void setTimeInfoAct(bool activation, float sample_rate) 
            {
                time_info_activate_store = activation;
                time_info_sample_rate = sample_rate;
            }

            void cleanTimeInfoSD() 
            {
                time_info_activate_store = false;
                time_info_data.clear();
            }

            std::vector<std::vector<float>> getTimeInfoStoredData() 
            {
                return time_info_data;
            }

            //// Methods related to the Camera data gathering
            //void storeCameraData() 
            //{
            //    if (Camera_activate_store) {
            //        uint64_t time_new = clock()->nowNanos();
            //        int Camera_threshold = UE4_second / Camera_sample_rate;
            //        uint64_t time_threshold = Camera_time_old + Camera_threshold;
            //        if (time_new >= time_threshold) {
            //            const auto& local_Camera_data = Camera_api->getImages(Camera_request);
            //            Camera_time_old = time_new;
            //            Camera_data.push_back(local_Camera_data);
            //        }
            //    }
            //}

            //void setCameraAct(bool activation, float sample_rate, const std::vector<ImageCaptureBase::ImageRequest>& request, VehicleSimApiBase* const& api) 
            //{
            //    Camera_activate_store = activation;
            //    Camera_sample_rate = sample_rate;
            //    //unused(request);
            //    Camera_request = request;
            //    Camera_api = api;
            //}

            //void cleanCameraSD() 
            //{
            //    Camera_activate_store = false;
            //    Camera_data.clear();
            //}

            //void saveCameraSD(std::string path) 
            //{
            //    unused(path);
            //}

            //// Methods related to the IMU data gathering
            //void storeIMUData() 
            //{
            //    if (IMU_activate_store) {
            //        uint64_t time_new = local_IMU_data.time_stamp;
            //        int IMU_threshold = UE4_second / IMU_sample_rate;
            //        uint64_t time_threshold = IMU_time_old + IMU_threshold;
            //        if (time_new >= time_threshold) {
            //            IMU_time_old = time_new;
            //            IMU_data.push_back(local_IMU_data);
            //        }
            //    }
            //}

            //void setIMUAct(bool activation, float sample_rate) 
            //{
            //    IMU_activate_store = activation;
            //    IMU_sample_rate = sample_rate;
            //}

            //void cleanIMUSD() 
            //{
            //    IMU_activate_store = false;
            //    IMU_data.clear();
            //}

            //std::vector<msr::airlib::ImuBase::Output> getIMUStoredData() 
            //{
            //    return IMU_data;
            //}

            //// Methods related to the PWMs data gathering
            //void storePWMData() 
            //{
            //    if (PWM_activate_store) {
            //        PWMs[0] = board_->getMotorControlSignal(0);
            //        PWMs[1] = board_->getMotorControlSignal(1);
            //        PWMs[2] = board_->getMotorControlSignal(2);
            //        PWMs[3] = board_->getMotorControlSignal(3);
            //        std::vector<float> local_PWM_data = { PWMs[0], PWMs[1], PWMs[2], PWMs[3] };
            //        uint64_t time_new = local_IMU_data.time_stamp;
            //        int PWM_threshold = UE4_second / PWM_sample_rate;
            //        uint64_t time_threshold = PWM_time_old + PWM_threshold;
            //        if (time_new >= time_threshold) {
            //            PWM_time_old = time_new;
            //            PWM_data.push_back(local_PWM_data);
            //        }
            //    }
            //}

            //void setPWMAct(bool activation, float sample_rate) 
            //{
            //    PWM_activate_store = activation;
            //    PWM_sample_rate = sample_rate;
            //}

            //void cleanPWMSD() 
            //{
            //    PWM_activate_store = false;
            //    PWM_data.clear();
            //}

            //std::vector<std::vector<float>> getPWMStoredData() 
            //{
            //    return PWM_data;
            //}

            //// Methods related to the ground truth position data gathering
            //void storePositionData() 
            //{
            //    if (position_activate_store) {
            //        std::vector<float> local_position_data = { current_pos.x(), current_pos.y(), current_pos.z() };
            //        uint64_t time_new = local_IMU_data.time_stamp;
            //        int position_threshold = UE4_second / position_sample_rate;
            //        uint64_t time_threshold = position_time_old + position_threshold;
            //        if (time_new >= time_threshold) {
            //            position_time_old = time_new;
            //            position_data.push_back(local_position_data);
            //        }
            //    }
            //}

            //void setPositionAct(bool activation, float sample_rate) 
            //{
            //    position_activate_store = activation;
            //    position_sample_rate = sample_rate;
            //}

            //void cleanPositionSD() 
            //{
            //    position_activate_store = false;
            //    position_data.clear();
            //}

            //std::vector<std::vector<float>> getPositionStoredData() 
            //{
            //    return position_data;
            //}

            //// Methods related to the barometer data gathering
            //void storeBarometerData() 
            //{
            //    if (barometer_activate_store) {
            //        auto local_barometer_data = getBarometerData("");
            //        uint64_t time_new = local_barometer_data.time_stamp;
            //        int barometer_threshold = UE4_second / barometer_sample_rate;
            //        uint64_t time_threshold = barometer_time_old + barometer_threshold;
            //        if (time_new >= time_threshold) {
            //            barometer_time_old = time_new;
            //            barometer_data.push_back(local_barometer_data);
            //        }
            //    }
            //}

            //void setBarometerAct(bool activation, float sample_rate) 
            //{
            //    barometer_activate_store = activation;
            //    barometer_sample_rate = sample_rate;
            //}

            //void cleanBarometerSD() 
            //{
            //    barometer_activate_store = false;
            //    barometer_data.clear();
            //}

            //std::vector<msr::airlib::BarometerBase::Output> getBarometerStoredData() 
            //{
            //    return barometer_data;
            //}

            //// Methods related to the magnetometer data gathering
            //void storeMagnetometerData() 
            //{
            //    if (magnetometer_activate_store) {
            //        auto local_magnetometer_data = getMagnetometerData("");
            //        uint64_t time_new = local_magnetometer_data.time_stamp;
            //        int magnetometer_threshold = UE4_second / magnetometer_sample_rate;
            //        uint64_t time_threshold = magnetometer_time_old + magnetometer_threshold;
            //        if (time_new >= time_threshold) {
            //            magnetometer_time_old = time_new;
            //            magnetometer_data.push_back(local_magnetometer_data);
            //        }
            //    }
            //}

            //void setMagnetometerAct(bool activation, float sample_rate) 
            //{
            //    magnetometer_activate_store = activation;
            //    magnetometer_sample_rate = sample_rate;
            //}

            //void cleanMagnetometerSD() 
            //{
            //    magnetometer_activate_store = false;
            //    magnetometer_data.clear();
            //}

            //std::vector<msr::airlib::MagnetometerBase::Output> getMagnetometerStoredData() 
            //{
            //    return magnetometer_data;
            //}

            //// Methods related to the GPS data gathering
            //void storeGPSData() 
            //{
            //    if (GPS_activate_store) {
            //        auto local_GPS_data = getGpsData("");
            //        uint64_t time_new = local_GPS_data.time_stamp;
            //        int GPS_threshold = UE4_second / GPS_sample_rate;
            //        uint64_t time_threshold = GPS_time_old + GPS_threshold;
            //        if (time_new >= time_threshold) {
            //            GPS_time_old = time_new;
            //            GPS_data.push_back(local_GPS_data);
            //        }
            //    }
            //}

            //void setGPSAct(bool activation, float sample_rate)
            //{
            //    GPS_activate_store = activation;
            //    GPS_sample_rate = sample_rate;
            //}

            //void cleanGPSSD()
            //{
            //    GPS_activate_store = false;
            //    GPS_data.clear();
            //}

            //std::vector<msr::airlib::GpsBase::Output> getGPSStoredData()
            //{
            //    return GPS_data;
            //}



        private:
            // General variables
            int UE4_second = 1000000000;
            ImuBase::Output local_IMU_data;

            // Variables related to the general activation of data gathering
            bool plot_data_collection_switch = false;

            // Variables related to reference position data gathering
            Vector3r current_pos_ref;
            bool pos_ref_activate_store = false;
            float pos_ref_sample_rate = 1000;
            uint64_t pos_ref_time_old = 0;
            std::vector<std::vector<float>> pos_ref_data;

            // Variables related to position error data gathering
            Vector3r current_pos_error;
            bool pos_error_activate_store = false;
            float pos_error_sample_rate = 1000;
            uint64_t pos_error_time_old = 0;
            std::vector<std::vector<float>> pos_error_data;

            // Variables related to derivative of the position error data gathering
            Vector3r current_pos_error_dot;
            bool pos_error_dot_activate_store = false;
            float pos_error_dot_sample_rate = 1000;
            uint64_t pos_error_dot_time_old = 0;
            std::vector<std::vector<float>> pos_error_dot_data;

            // Variables related to reference velocity data gathering
            Vector3r current_vel_ref;
            bool vel_ref_activate_store = false;
            float vel_ref_sample_rate = 1000;
            uint64_t vel_ref_time_old = 0;
            std::vector<std::vector<float>> vel_ref_data;

            // Variables related to velocity data gathering
            Vector3r current_vel;
            bool vel_activate_store = false;
            float vel_sample_rate = 1000;
            uint64_t vel_time_old = 0;
            std::vector<std::vector<float>> vel_data;

            // Variables related to reference acceleration data gathering
            Vector3r current_acc_ref;
            bool acc_ref_activate_store = false;
            float acc_ref_sample_rate = 1000;
            uint64_t acc_ref_time_old = 0;
            std::vector<std::vector<float>> acc_ref_data;

            // Variables related to yaw transfer function data gathering
            float stored_yaw_transfer_fcn_3;
            float stored_yaw_transfer_fcn_1;
            float stored_yaw_transfer_fcn_1_1;
            bool yaw_transfer_fcn_activate_store = false;
            float yaw_transfer_fcn_sample_rate = 1000;
            uint64_t yaw_transfer_fcn_time_old = 0;
            std::vector<std::vector<float>> yaw_transfer_fcn_data;


            // Variables related to rotational rates data gathering
            Vector3r current_pqr_ref;
            bool pqr_ref_activate_store = false;
            float pqr_ref_sample_rate = 1000;
            uint64_t pqr_ref_time_old = 0;
            std::vector<std::vector<float>> pqr_ref_data;

            // Variables related to rotational rates data gathering
            Vector3r current_pqr;
            bool pqr_activate_store = false;
            float pqr_sample_rate = 1000;
            uint64_t pqr_time_old = 0;
            std::vector<std::vector<float>> pqr_data;

            // Variables related to reference thrust data gathering
            float current_thrust_ref_fb;
            float current_thrust_ref_ff;
            bool thrust_ref_activate_store = false;
            float thrust_ref_sample_rate = 1000;
            uint64_t thrust_ref_time_old = 0;
            std::vector<std::vector<float>> thrust_ref_data;

            // Variables related to motor commanded rotations data gathering
            std::vector<float> current_omegas;
            bool omegas_activate_store = false;
            float omegas_sample_rate = 1000;
            uint64_t omegas_time_old = 0;
            std::vector<std::vector<float>> omegas_data;

            // Variables related to reference yaw data gathering
            float current_yaw_ref;
            float current_corrected_yaw_ref;
            bool yaw_ref_activate_store = false;
            float yaw_ref_sample_rate = 1000;
            uint64_t yaw_ref_time_old = 0;
            std::vector<std::vector<float>> yaw_ref_data;

            // Variables related to orientation data gathering
            Vector3r current_orientation;
            bool orientation_activate_store = false;
            float orientation_sample_rate = 1000;
            uint64_t orientation_time_old = 0;
            std::vector<std::vector<float>> orientation_data;

            // Variables related to position integrator data gathering
            Vector3r current_position_integrator;
            bool position_integrator_activate_store = false;
            float position_integrator_sample_rate = 1000;
            uint64_t position_integrator_time_old = 0;
            std::vector<std::vector<float>> position_integrator_data;

            // Variables related to thrust PI data gathering
            float current_thrust_P;
            float current_thrust_I;
            bool thrust_PI_activate_store = false;
            float thrust_PI_sample_rate = 1000;
            uint64_t thrust_PI_time_old = 0;
            std::vector<std::vector<float>> thrust_PI_data;

            // Variables related to damaged mass forces data gathering
            Vector3r current_damaged_mass_forces;
            bool damaged_mass_forces_activate_store = false;
            float damaged_mass_forces_sample_rate = 1000;
            uint64_t damaged_mass_forces_time_old = 0;
            std::vector<std::vector<float>> damaged_mass_forces_data;

            // Variables related to damaged mass moments data gathering
            Vector3r current_damaged_mass_moments;
            bool damaged_mass_moments_activate_store = false;
            float damaged_mass_moments_sample_rate = 1000;
            uint64_t damaged_mass_moments_time_old = 0;
            std::vector<std::vector<float>> damaged_mass_moments_data;

            // Variables related to damaged aero forces data gathering
            Vector3r current_damaged_aero_forces;
            bool damaged_aero_forces_activate_store = false;
            float damaged_aero_forces_sample_rate = 1000;
            uint64_t damaged_aero_forces_time_old = 0;
            std::vector<std::vector<float>> damaged_aero_forces_data;

            // Variables related to damaged aero moments data gathering
            Vector3r current_damaged_aero_moments;
            bool damaged_aero_moments_activate_store = false;
            float damaged_aero_moments_sample_rate = 1000;
            uint64_t damaged_aero_moments_time_old = 0;
            std::vector<std::vector<float>> damaged_aero_moments_data;

            // Variables related to sampling frequency data gathering
            float current_dt_real;
            float current_time;
            bool time_info_activate_store = false;
            float time_info_sample_rate = 1000;
            uint64_t time_info_time_old = 0;
            std::vector<std::vector<float>> time_info_data;

            // Variables related to Camera data gathering
            bool Camera_activate_store = false;
            int Camera_sample_rate = 60;
            uint64_t Camera_time_old = 0;
            std::vector<vector<ImageCaptureBase::ImageResponse>> Camera_data;
            VehicleSimApiBase* Camera_api;
            std::vector<ImageCaptureBase::ImageRequest> Camera_request = { ImageCaptureBase::ImageRequest("0", ImageCaptureBase::ImageType::Scene), ImageCaptureBase::ImageRequest("0", ImageCaptureBase::ImageType::Scene) };

            // Variables related to IMU data gathering
            bool IMU_activate_store = false;
            float IMU_sample_rate = 1000;
            uint64_t IMU_time_old = 0;
            std::vector<msr::airlib::ImuBase::Output> IMU_data;

            // Variables related to PWMs data gathering
            bool PWM_activate_store = false;
            float PWM_sample_rate = 1000;
            uint64_t PWM_time_old = 0;
            std::vector<std::vector<float>> PWM_data;

            // Variables related to ground truth position data gathering
            Vector3r current_pos;
            bool position_activate_store = false;
            float position_sample_rate = 1000;
            uint64_t position_time_old = 0;
            std::vector<std::vector<float>> position_data;

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

        };
    };
}



#endif // !physics_DamagedPropeller_hpp
