// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef msr_airlib_SimpleFlightDroneController_hpp
#define msr_airlib_SimpleFlightDroneController_hpp

//#include "vehicles/multirotor/firmwares/simple_flight/firmware/interfaces/IPidIntegrator.hpp"
//#include "vehicles/multirotor/firmwares/simple_flight/firmware/StdPidIntegrator.hpp"
#include "common/FirstOrderFilter.hpp"
#include <Eigen/Dense>
#include <Eigen/QR>

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

namespace msr {
    namespace airlib {

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

                // initialise variables for integration
                // last integration time
                last_time_integral = clock()->nowNanos();

                // Low pass filter for pos ref variables in order to avoid drastic changes
                real_T ps_ref_controller_lp_time_constant = 0.3f;    //time constant for low pass filter
                pos_ref_low_pass_filter_x.initialize(ps_ref_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_ref_low_pass_filter_x.reset();                                     
                pos_ref_low_pass_filter_y.initialize(ps_ref_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_ref_low_pass_filter_y.reset();                                  
                pos_ref_low_pass_filter_z.initialize(ps_ref_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_ref_low_pass_filter_z.reset();

                // Low pass filter for pos error variables in order to avoid drastic changes
                real_T ps_error_controller_lp_time_constant = 0.1f;    //time constant for low pass filter
                pos_error_low_pass_filter_x.initialize(ps_error_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_error_low_pass_filter_x.reset();                                         
                pos_error_low_pass_filter_y.initialize(ps_error_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_error_low_pass_filter_y.reset();                           
                pos_error_low_pass_filter_z.initialize(ps_error_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pos_error_low_pass_filter_z.reset();

                // Low pass filter TransferFcn3
                real_T heading_controller_lp_1_time_constant = 0.3f;    //time constant for low pass filter
                heading_low_pass_filter_1.initialize(heading_controller_lp_1_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                heading_low_pass_filter_1.reset();

                // Low pass filter TransferFcn1
                heading_low_pass_filter_2.initialize(heading_controller_lp_1_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                heading_low_pass_filter_2.reset();

                // Low pass filter TransferFcn1
                real_T heading_controller_lp_3_time_constant = 0.01f;    //time constant for low pass filter
                heading_low_pass_filter_3.initialize(heading_controller_lp_3_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                heading_low_pass_filter_3.reset();

                // Low pass filter for attitude controller
                real_T attitude_controller_lp_time_constant = 0.05f;
                attitude_controller_low_pass_filter_x.initialize(attitude_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                attitude_controller_low_pass_filter_x.reset();
                attitude_controller_low_pass_filter_y.initialize(attitude_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                attitude_controller_low_pass_filter_y.reset();
                attitude_controller_low_pass_filter_z.initialize(attitude_controller_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                attitude_controller_low_pass_filter_z.reset();

                // Low pass filter for pq_des
                real_T pq_des_lp_time_constant = 0.05f;
                pq_des_low_pass_filter_1.initialize(pq_des_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pq_des_low_pass_filter_1.reset();
                pq_des_low_pass_filter_2.initialize(pq_des_lp_time_constant, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pq_des_low_pass_filter_2.reset();

                // Low pass filter for pqr
                pqr_low_pass_filter_x.initialize(t_indi, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pqr_low_pass_filter_x.reset();
                pqr_low_pass_filter_y.initialize(t_indi, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pqr_low_pass_filter_y.reset();
                pqr_low_pass_filter_z.initialize(t_indi, 0.0f, 0.0f, 0.0f, 0.0f, integration_method);
                pqr_low_pass_filter_z.reset();

                // Initialising the dulast matrix
                du_last << 0.0f,
                    0.0f,
                    0.0f,
                    0.0f;
                w_f << 0.0f,
                    0.0f,
                    0.0f,
                    0.0f;

                // Low pass filter for the actuator dynamics
                actuator_low_pass_filter_1.initialize(t_w, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                actuator_low_pass_filter_1.reset();  
                actuator_low_pass_filter_2.initialize(t_w, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                actuator_low_pass_filter_2.reset(); 
                actuator_low_pass_filter_3.initialize(t_w, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                actuator_low_pass_filter_3.reset();
                actuator_low_pass_filter_4.initialize(t_w, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                actuator_low_pass_filter_4.reset();

                // Low pass filter for w_obs
                real_T w_obs_lp_time_constant = 0.4f;
                w_obs_low_pass_filter_1.initialize(w_obs_lp_time_constant, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                w_obs_low_pass_filter_1.reset();                      
                w_obs_low_pass_filter_2.initialize(w_obs_lp_time_constant, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                w_obs_low_pass_filter_2.reset();                  
                w_obs_low_pass_filter_3.initialize(w_obs_lp_time_constant, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                w_obs_low_pass_filter_3.reset();                         
                w_obs_low_pass_filter_4.initialize(w_obs_lp_time_constant, 695.0f, 695.0f, 0.0f, 0.0f, integration_method);
                w_obs_low_pass_filter_4.reset();
            }

        public: //VehicleApiBase implementation
            virtual void resetImplementation() override
            {
                MultirotorApiBase::resetImplementation();

                // Resetting all the created parameters for the INDI controller
                // Dummy parameters
                dummy = false;
                dummy_yaw_ref = 0.0f;
                dummy_x = 0.0f;
                dummy_y = 0.0f;
                dummy_z = -3.0f;

                // Integrators
                Vector3r pos_ref = getPosRef();
                last_time_integral = clock()->nowNanos();
                integrator_pos_x = 0.0f;
                integrator_pos_x_dot_current = 0.0f;
                integrator_pos_x_dot_previous = 0.0f;
                integrator_pos_y = 0.0f;
                integrator_pos_y_dot_current = 0.0f;
                integrator_pos_y_dot_previous = 0.0f;
                integrator_pos_z = 0.0f;
                integrator_pos_z_dot_current = 0.0f;
                integrator_pos_z_dot_previous = 0.0f;
                integrator_thrust_ref = 0.0f;
                integrator_thrust_ref_dot_current = 0.0f;
                integrator_thrust_ref_dot_previous = 0.0f;

                // Parameters for the yawing
                n_half_rotations = 0.0f;
                previous_yaw_sign = 0.0f;
                previous_yaw_angle = 0.0f;
                infinite_yaw_angle = 0.0f;

                // First order time filters
                pos_ref_low_pass_filter_x.resetImplementation();
                pos_ref_low_pass_filter_y.resetImplementation();
                pos_ref_low_pass_filter_z.resetImplementation();
                pos_error_low_pass_filter_x.resetImplementation();
                pos_error_low_pass_filter_y.resetImplementation();
                pos_error_low_pass_filter_z.resetImplementation();
                heading_low_pass_filter_1.resetImplementation();
                heading_low_pass_filter_2.resetImplementation();
                heading_low_pass_filter_3.resetImplementation();
                attitude_controller_low_pass_filter_x.resetImplementation();
                attitude_controller_low_pass_filter_y.resetImplementation();
                attitude_controller_low_pass_filter_z.resetImplementation();
                pq_des_low_pass_filter_1.resetImplementation();
                pq_des_low_pass_filter_2.resetImplementation();
                pqr_low_pass_filter_x.resetImplementation();
                pqr_low_pass_filter_y.resetImplementation();
                pqr_low_pass_filter_z.resetImplementation();
                actuator_low_pass_filter_1.resetImplementation();
                actuator_low_pass_filter_2.resetImplementation();
                actuator_low_pass_filter_3.resetImplementation();
                actuator_low_pass_filter_4.resetImplementation();
                w_obs_low_pass_filter_1.resetImplementation();
                w_obs_low_pass_filter_2.resetImplementation();
                w_obs_low_pass_filter_3.resetImplementation();
                w_obs_low_pass_filter_4.resetImplementation();

                pos_error_x_last = 0.0f;
                pos_error_y_last = 0.0f;
                pos_error_z_last = 0.0f;
                v_error_x_last = 0.0f;
                v_error_y_last = 0.0f;
                v_error_z_last = 0.0f;
                filtered_yaw_cmd_2_last = 0.0f;
                filtered_r_cmd_last = 0.0f;
                filtered_ndi_x_last = 0.0f;
                filtered_ndi_y_last = 0.0f;
                filtered_ndi_z_last = 0.0f;
                filtered_pq_des_1_last = 0.0f;
                filtered_pq_des_2_last = 0.0f;
                filtered_pqr_x_last = 0.0f;
                filtered_pqr_y_last = 0.0f;
                filtered_pqr_z_last = 0.0f;

                // Variables for the function within the yaw command block
                theta_dot = 0.0f;

                // Variables for the attitude controller block
                pq_des_dot_1 = 0.0f;
                pq_des_dot_2 = 0.0f;

                // Variables of the INDI Allocator
                omega_sum = 0.0f;
                du_last << 0.0f,
                    0.0f,
                    0.0f,
                    0.0f;
                w_f << 0.0f,
                    0.0f,
                    0.0f,
                    0.0f;

                firmware_->reset();
            }

            virtual void droneTeleportReset() override
            {
                // Resetting all the created parameters for the INDI controller
                // Dummy parameters
                dummy = false;
                dummy_yaw_ref = 0;
                dummy_x = 0;
                dummy_y = 0;
                dummy_z = -3;

                // Integrators
                last_time_integral = clock()->nowNanos();
                integrator_pos_x = 0;
                integrator_pos_x_dot_current = 0;
                integrator_pos_x_dot_previous = 0;
                integrator_pos_y = 0;
                integrator_pos_y_dot_current = 0;
                integrator_pos_y_dot_previous = 0;
                integrator_pos_z = 0;
                integrator_pos_z_dot_current = 0;
                integrator_pos_z_dot_previous = 0;
                integrator_thrust_ref = 0;
                integrator_thrust_ref_dot_current = 0;
                integrator_thrust_ref_dot_previous = 0;

                // Parameters for the yawing
                setYawRef(teleport_yaw_deg);
                n_half_rotations = 0;
                previous_yaw_sign = 0;
                previous_yaw_angle = 0;
                infinite_yaw_angle = 0;

                // First order time filters
                pos_ref_low_pass_filter_x.resetImplementation();
                pos_ref_low_pass_filter_y.resetImplementation();
                pos_ref_low_pass_filter_z.resetImplementation();
                Vector3r pos_ref = getPosRef();
                pos_ref_low_pass_filter_x.setOutput(pos_ref.x());
                pos_ref_low_pass_filter_y.setOutput(pos_ref.y());
                pos_ref_low_pass_filter_z.setOutput(pos_ref.z());

                pos_error_low_pass_filter_x.resetImplementation();
                pos_error_low_pass_filter_y.resetImplementation();
                pos_error_low_pass_filter_z.resetImplementation();

                heading_low_pass_filter_1.resetImplementation();
                heading_low_pass_filter_1.setOutput(teleport_yaw_deg * M_PIf / 180.0f);

                heading_low_pass_filter_2.resetImplementation();
                heading_low_pass_filter_2.setOutput(teleport_yaw_deg * M_PIf / 180.0f);

                heading_low_pass_filter_3.resetImplementation();
                attitude_controller_low_pass_filter_x.resetImplementation();
                attitude_controller_low_pass_filter_y.resetImplementation();
                attitude_controller_low_pass_filter_z.resetImplementation();
                pq_des_low_pass_filter_1.resetImplementation();
                pq_des_low_pass_filter_2.resetImplementation();
                pqr_low_pass_filter_x.resetImplementation();
                pqr_low_pass_filter_y.resetImplementation();
                pqr_low_pass_filter_z.resetImplementation();
                actuator_low_pass_filter_1.resetImplementation();
                actuator_low_pass_filter_2.resetImplementation();
                actuator_low_pass_filter_3.resetImplementation();
                actuator_low_pass_filter_4.resetImplementation();
                w_obs_low_pass_filter_1.resetImplementation();
                w_obs_low_pass_filter_2.resetImplementation();
                w_obs_low_pass_filter_3.resetImplementation();
                w_obs_low_pass_filter_4.resetImplementation();

                pos_error_x_last = pos_ref.x();
                pos_error_y_last = pos_ref.y();
                pos_error_z_last = pos_ref.z();
                v_error_x_last = 0;
                v_error_y_last = 0;
                v_error_z_last = 0;
                filtered_yaw_cmd_2_last = teleport_yaw_deg * M_PIf / 180.0f;
                filtered_r_cmd_last = 0;
                filtered_ndi_x_last = 0;
                filtered_ndi_y_last = 0;
                filtered_ndi_z_last = 0;
                filtered_pq_des_1_last = 0;
                filtered_pq_des_2_last = 0;
                filtered_pqr_x_last = 0;
                filtered_pqr_y_last = 0;
                filtered_pqr_z_last = 0;

                // Variables for the function within the yaw command block
                theta_dot = 0;

                // Variables for the attitude controller block
                pq_des_dot_1 = 0;
                pq_des_dot_2 = 0;

                // Variables of the INDI Allocator
                omega_sum = 0;
                du_last << 0,
                    0,
                    0,
                    0;
                w_f << 0,
                    0,
                    0,
                    0;
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

        public:

            virtual real_T getActuation(unsigned int rotor_index) override
            {
                local_IMU_data = getImuData("");
                storePosRefData();
                storePosErrorData();
                storePosErrorDotData();
                storeVelRefData();
                storeVelData();
                storeAccRefData();
                storePqrRefData();
                storePqrData();
                storeThrustRefData();
                storeOmegasData();
                storeYawRefData();
                storeOrientationData();
                storePositionIntegratorData();
                storeThrustPiData();
                storeIMUData();
                storePWMData();
                storePositionData();
                storeBarometerData();
                storeMagnetometerData();
                storeGPSData();
                //storeCameraData();
                //auto control_signal_old = board_->getMotorControlSignal(rotor_index);
                //real_T control_signal;
                //if (locked_propeller[rotor_index] == true) {
                //    control_signal = lock_coefficients[rotor_index];
                //}
                //else {
                //    real_T prop_damage = propeller_damage_coefficients[rotor_index];
                //    control_signal = control_signal_old * prop_damage;
                //}
                real_T control_signal;
                if (w_f(rotor_index) < 50)
                {
                    control_signal = 0.0f;
                }
                else 
                {
                    control_signal = w_f(rotor_index) / w_max;
                }
                return control_signal;
            }

            // Implementing the INDI controller
            // Structs used as the output of some methods
            struct a_thrust_ref {
                Vector3r a_ref;
                real_T thrust_ref_fb;
            };
            struct ndi_omegasumref {
                Vector3r nd_i;
                real_T omega_sum_ref;
            };
            struct u_pqr_omega_ref {
                real_T omega_sum_ref;
                Vector3r u_pqr;
            };

            // Helper function
            real_T filter_derivative(FirstOrderFilter<real_T>& low_pass_filter, real_T& filter_input, real_T& filtered_input_last)
            {
                low_pass_filter.setInput(filter_input);
                low_pass_filter.update();
                real_T filtered_input = low_pass_filter.getOutput();
                real_T filtered_input_dot = (filtered_input - filtered_input_last) / dt_real_integration;
                filtered_input_last = filtered_input;
                return filtered_input_dot;
            }

            real_T only_filter(FirstOrderFilter<real_T>& low_pass_filter, real_T& filter_input)
            {
                low_pass_filter.setInput(filter_input);
                low_pass_filter.update();
                real_T filtered_input = low_pass_filter.getOutput();
                return filtered_input;
            }

            real_T apply_integral(real_T integrator_current, real_T& x_dot_previous, real_T& x_dot_current, real_T& x_dot_next, real_T dt_real)
            {
                if (integration_method == 0)  // Verlet algorithm
                {
                    integrator_current = integrator_current + (x_dot_current + x_dot_next) * (0.5f * dt_real);

                    x_dot_current = x_dot_next;
                }
                else if (integration_method == 1)  // Verlet algorithm AirSim
                {
                    integrator_current = integrator_current + dt_real * x_dot_next;
                }
                else if (integration_method == 2)  // Adams-Bashfort 2-step method
                {
                    integrator_current = integrator_current + 3 / 2.0f * x_dot_next * dt_real - x_dot_current * (0.5f * dt_real);

                    x_dot_current = x_dot_next;
                }
                else if (integration_method == 3)  // Beeman and Schofield
                {
                    integrator_current = integrator_current + 1 / 6.0f * (2 * x_dot_next + 5 * x_dot_current - x_dot_previous) * dt_real;

                    x_dot_previous = x_dot_current;

                    x_dot_current = x_dot_next;
                }
                return integrator_current;
            }

            void compute_infinite_yaw(real_T yaw)
            {
                int yaw_sign = yaw>=0;
                if (previous_yaw_sign != yaw_sign)
                {
                    if (previous_yaw_sign == 1 && previous_yaw_angle > (M_PIf / 2.0f))
                    {
                        if (n_half_rotations >= 0)
                        {
                            n_half_rotations += 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations + (M_PIf + yaw);
                        }
                        else if (n_half_rotations < 0)
                        {
                            n_half_rotations += 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                        }
                    }
                    else if (previous_yaw_sign == 1 && previous_yaw_angle < (M_PIf / 2.0f) && n_half_rotations != 0)
                    {
                        if (n_half_rotations > 0)
                        {
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                            n_half_rotations -= 1;
                        }
                        else if (n_half_rotations < 0)
                        {
                            n_half_rotations -= 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                        }
                    }
                    else if (previous_yaw_sign == 0 && -previous_yaw_angle > (M_PIf / 2.0f))
                    {
                        if (n_half_rotations > 0)
                        {
                            n_half_rotations -= 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                        }
                        else if (n_half_rotations <= 0)
                        {
                            n_half_rotations -= 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations - (M_PIf - yaw);
                        }
                    }
                    else if (previous_yaw_sign == 0 && -previous_yaw_angle < (M_PIf / 2.0f) && n_half_rotations != 0)
                    {
                        if (n_half_rotations > 0)
                        {
                            n_half_rotations += 1;
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                        }
                        else
                        {
                            infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                            n_half_rotations += 1;
                        }
                    }
                    else
                    {
                        infinite_yaw_angle = yaw;
                    }
                }
                else if (n_half_rotations > 0)
                {
                    if (yaw_sign == 1)
                    {
                        infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                    }
                    else if (yaw_sign == 0)
                    {
                        infinite_yaw_angle = M_PIf * n_half_rotations + (M_PIf + yaw);
                    }
                }
                else if (n_half_rotations < 0)
                {
                    if (yaw_sign == 1)
                    {
                        infinite_yaw_angle = M_PIf * n_half_rotations - (M_PIf - yaw);
                    }
                    else if (yaw_sign == 0)
                    {
                        infinite_yaw_angle = M_PIf * n_half_rotations + yaw;
                    }
                }
                else if (n_half_rotations == 0)
                {
                    infinite_yaw_angle = yaw;

                }
                previous_yaw_sign = yaw_sign;
                previous_yaw_angle = yaw;
            }

            real_T gaussian_noise(real_T clean_number, real_T mean, real_T variance)
            {
                real_T standard_deviation = sqrt(variance);
                std::normal_distribution<double> distribution(mean, standard_deviation);
                real_T dirty_number = clean_number + distribution(generator);
                return dirty_number;
            }

            Vector3r gaussian_noise(Vector3r clean_numbers, Vector3r means, Vector3r variances)
            {
                real_T dirty_number_x = gaussian_noise(clean_numbers.x(), means.x(), variances.x());
                real_T dirty_number_y = gaussian_noise(clean_numbers.y(), means.y(), variances.y());
                real_T dirty_number_z = gaussian_noise(clean_numbers.z(), means.z(), variances.z());
                Vector3r dirty_number(dirty_number_x, dirty_number_y, dirty_number_z);
                return dirty_number;
            }

            real_T apply_measurement_noise(real_T clean_number, real_T mean, real_T variance)
            {
                if (activate_measurement_noise)
                {
                    real_T dirty_number = gaussian_noise(clean_number, mean, variance);
                    return dirty_number;
                }
                else
                {
                    return clean_number;
                }
            }

            Vector3r apply_measurement_noise(Vector3r clean_number, Vector3r mean, Vector3r variance)
            {
                if (activate_measurement_noise)
                {
                    Vector3r dirty_number = gaussian_noise(clean_number, mean, variance);
                    return dirty_number;
                }
                else
                {
                    return clean_number;
                }
            }

            // Obtain all the data with Gaussian noise
            void apply_measurement_noise_states()
            {
                Vector3r position;
                Vector3r velocity;
                Vector3r attitude;
                Vector3r pqr;
                if (one_step_back_dyn)
                {
                    real_T pitch, roll, yaw;
                    VectorMath::toEulerianAngle(previous_.pose.orientation, pitch, roll, yaw);
                    position = previous_.pose.position;
                    velocity = previous_.twist.linear;
                    attitude = Vector3r(roll, pitch, yaw);
                    pqr = previous_.twist.angular;
                }
                else
                {
                    real_T pitch, roll, yaw;
                    VectorMath::toEulerianAngle(getOrientation(), pitch, roll, yaw);
                    position = getKinematicsEstimated().pose.position;
                    velocity = getKinematicsEstimated().twist.linear;
                    attitude = Vector3r(roll, pitch, yaw);
                    pqr = getKinematicsEstimated().twist.angular;
                }

                if (activate_measurement_noise)
                {
                    xyz_states = apply_measurement_noise(position, mean_xyz, cov_xyz);
                    vxyz_states = apply_measurement_noise(velocity, mean_vxyz, cov_vxyz);
                    att_states = apply_measurement_noise(attitude, mean_att, cov_att);
                    pqr_states = apply_measurement_noise(pqr, mean_pqr, cov_pqr);
                }
                else
                {
                    xyz_states = position;
                    vxyz_states = velocity;
                    att_states = attitude;
                    pqr_states = pqr;
                }   
            }

            // Method which describes the first block from which the acceleration and thrust reference values are obtained
            a_thrust_ref position_velocity_pid()
            {
                // Obtaining passed time
                TTimeDelta dt = clock()->updateSince(last_time_integral);
                dt_real_integration = static_cast<real_T>(dt);

                // Apply measurement noise to the states
                apply_measurement_noise_states();

                // Retrieving block inputs
                Vector3r position = xyz_states;
                Vector3r velocity = vxyz_states;
                Vector3r pos_ref = getPosRef();
                if (dummy)
                {
                    pos_ref.x() = dummy_x;
                    pos_ref.y() = dummy_y;
                    pos_ref.z() = dummy_z;
                }
                
                if (correct_start)
                {
                    real_T error_x = abs(pos_ref.x() - position.x());
                    real_T alpha_x = 1.0f / (1.0f + alpha_tuning * error_x);
                    real_T v_x = v_tuning_1 * (1.0f / (1.0f + std::exp(-v_tuning_2 * abs(velocity.x()))) - v_tuning_3);
                    int sign_x = (0 < (pos_ref.x() - position.x())) - ((pos_ref.x() - position.x()) < 0);
                    pos_ref.x() = position.x() + sign_x * (alpha_x * error_x + (1 - alpha_x) * error_x * v_x);

                    real_T error_y = abs(pos_ref.y() - position.y());
                    real_T alpha_y = 1.0f / (1.0f + alpha_tuning * error_y);
                    real_T v_y = v_tuning_1 * (1.0f / (1.0f + std::exp(-v_tuning_2 * abs(velocity.y()))) - v_tuning_3);
                    int sign_y = (0 < (pos_ref.y() - position.y())) - ((pos_ref.y() - position.y()) < 0);
                    pos_ref.y() = position.y() + sign_y * (alpha_y * error_y + (1 - alpha_y) * error_y * v_y);

                    real_T error_z = abs(pos_ref.z() - position.z());
                    real_T alpha_z = 1.0f / (1.0f + alpha_tuning * error_z);
                    real_T v_z = v_tuning_1 * (1.0f / (1.0f + std::exp(-v_tuning_2 * abs(velocity.z()))) - v_tuning_3);
                    int sign_z = (0 < (pos_ref.z() - position.z())) - ((pos_ref.z() - position.z()) < 0);
                    pos_ref.z() = position.z() + sign_z * (alpha_z * error_z + (1 - alpha_z) * error_z * v_z);

                    //pos_ref.x() = abs(pos_ref.x() - position.x()) * std::max((1 - 1 / (velocity.x() * 10 + 0.0000001)), 0.25) + position.x();
                    //pos_ref.y() = (pos_ref.y() - position.y()) * std::max((1 - 1 / (velocity.y() * 10 + 0.0000001)), 0.25) + position.y();
                    //pos_ref.z() = (pos_ref.z() - position.z()) * std::max((1 - 1 / (velocity.z() * 10 + 0.0000001)), 0.25) + position.z();
                }
                

                //// Filtered position reference
                pos_ref.x() = only_filter(pos_ref_low_pass_filter_x, pos_ref.x());
                pos_ref.y() = only_filter(pos_ref_low_pass_filter_y, pos_ref.y());
                pos_ref.z() = only_filter(pos_ref_low_pass_filter_z, pos_ref.z());

                // Computing position error
                Vector3r pos_error = pos_ref - position;
                //pos_error.x() = only_filter(pos_error_low_pass_filter_x, pos_error.x());
                //pos_error.y() = only_filter(pos_error_low_pass_filter_y, pos_error.y());
                //pos_error.z() = only_filter(pos_error_low_pass_filter_z, pos_error.z());

                // Checking whether the oscillations are caused by decimal problems: float is not enough
                //pos_error.x() = (float)std::floor((pos_ref.x() - position.x()) * 1000000) / 1000000;
                //pos_error.y() = (float)std::floor((pos_ref.y() - position.y()) * 1000000) / 1000000;
                //pos_error.z() = (float)std::floor((pos_ref.z() - position.z()) * 1000000) / 1000000;
                //auto potato = std::floor((pos_ref.x() - position.x()) * 1000000) / 1000000;
                //auto tomato = std::floor(((float)pos_ref.x() - (float)position.x()) * 1000000) / 1000000;

                // Computing the position derivative
                real_T pos_error_x_dot = (pos_error.x() - pos_error_x_last) / dt_real_integration;
                pos_error_x_last = pos_error.x();
                real_T pos_error_y_dot = (pos_error.y() - pos_error_y_last) / dt_real_integration;
                pos_error_y_last = pos_error.y();
                real_T pos_error_z_dot = (pos_error.z() - pos_error_z_last) / dt_real_integration;
                pos_error_z_last = pos_error.z();
                Vector3r pos_error_dot(pos_error_x_dot, pos_error_y_dot, pos_error_z_dot);

                // Computing reference velocity
                Vector3r v_ref = pos_P * pos_error + pos_D * pos_error_dot;

                // Computing velocity error and its derivative
                Vector3r v_error = v_ref - velocity;
                real_T v_error_x_dot = (v_error.x() - v_error_x_last) / dt_real_integration;
                v_error_x_last = v_error.x();
                real_T v_error_y_dot = (v_error.y() - v_error_y_last) / dt_real_integration;
                v_error_y_last = v_error.y();
                real_T v_error_z_dot = (v_error.z() - v_error_z_last) / dt_real_integration;
                v_error_z_last = v_error.z();
                Vector3r v_error_dot(v_error_x_dot, v_error_y_dot, v_error_z_dot);

                // Computing the integration of the position error
                integrator_pos_x = apply_integral(integrator_pos_x, integrator_pos_x_dot_previous, integrator_pos_x_dot_current, pos_error.x(), dt_real_integration);
                integrator_pos_y = apply_integral(integrator_pos_y, integrator_pos_y_dot_previous, integrator_pos_y_dot_current, pos_error.y(), dt_real_integration);
                integrator_pos_z = apply_integral(integrator_pos_z, integrator_pos_z_dot_previous, integrator_pos_z_dot_current, pos_error.z(), dt_real_integration);

                Vector3r pos_i_term(pos_I * integrator_pos_x, pos_I * integrator_pos_y, 0 * integrator_pos_z);

                // Computing the reference acceleration
                Vector3r a_ref = v_ref + pos_i_term + vel_P * v_error + vel_D * v_error_dot;

                // Computing the reference thrust value
                integrator_thrust_ref = apply_integral(integrator_thrust_ref, integrator_thrust_ref_dot_previous, integrator_thrust_ref_dot_current, pos_error.z(), dt_real_integration);
                real_T thrust_ref_fb = thrust_P * pos_error.z() + thrust_I * integrator_thrust_ref;

                // Storing values for debugging and plotting
                current_pos_ref = pos_ref;
                current_pos = position;
                current_pos_error = pos_error;
                current_pos_error_dot = pos_error_dot;
                current_vel_ref = v_ref;
                current_vel = velocity;
                current_acc_ref = a_ref;
                current_position_integrator.x() = pos_I * integrator_pos_x;
                current_position_integrator.y() = pos_I * integrator_pos_y;
                current_position_integrator.z() = 0.0f * integrator_pos_z;
                current_thrust_P = thrust_P * pos_error.z();
                current_thrust_I = thrust_I * integrator_thrust_ref;

                return a_thrust_ref{ a_ref , thrust_ref_fb };
            }

            // Method used to obtain the ndi and omega_sum reference values
            ndi_omegasumref ndiDirectlyFromA()
            {
                a_thrust_ref position_velocity_pid_input = position_velocity_pid();
                Vector3r a_ref = position_velocity_pid_input.a_ref;
                real_T thrust_ref_fb = position_velocity_pid_input.thrust_ref_fb;
                real_T roll = att_states.x();
                real_T pitch = att_states.y();
                real_T yaw = att_states.z();
                compute_infinite_yaw(yaw);
                yaw = infinite_yaw_angle;

                // Computing nd_i
                Vector3r gravity_vector(0.0f, 0.0f, gravity_Delft);
                Vector3r nd_i = (a_ref - gravity_vector).normalized();

                // Computing omega_sum_ref
                real_T thrust_ref_ff = (a_ref.z() - gravity_Delft) / (cos(roll) * cos(pitch));
                real_T omega_sum_ref = sqrt(std::max(-(thrust_ref_ff + thrust_ref_fb), 0.0f) * specific_thrust_2_omegasqure) * rpm2rad * 4.0f;

                // Thrust_fb is related to the error in position whereas thrust_ff is related to the acceleration reference
                current_thrust_ref_fb = thrust_ref_fb;
                current_thrust_ref_ff = thrust_ref_ff;
                return ndi_omegasumref{ nd_i , omega_sum_ref };
            }

            // Method that implements the Matlab block of heading controller_NDI
            real_T headingControllerNDI()
            {
                // Obtaining block inputs
                // The next line obtain yaw_ref from the AirSim outer loop
                real_T yaw_ref = getYawRef() * M_PIf / 180.0f;

                // The following lines compute the yaw ref using the position and reference location
                //const Vector3r cur = getKinematicsEstimated().pose.position;
                //const Vector3r heading = current_pos_ref - cur;
                //real_T yaw_ref;
                //if (heading.norm() > getDistanceAccuracy()) {
                //    yaw_ref = std::atan2(heading.y(), heading.x()) * 180 / M_PIf;
                //    yaw_ref = VectorMath::normalizeAngle(yaw_ref);
                //}
                //else
                //    yaw_ref = 0;
                //yaw_ref = yaw_ref * M_PIf / 180;

                real_T roll = att_states.x();
                real_T pitch = att_states.y();
                real_T yaw = att_states.z();
                real_T r = pqr_states.z();
                Vector3r position = xyz_states;
                Vector3r pos_ref = getPosRef();

                current_yaw_ref = yaw_ref;
                //real_T xy_distance = sqrt((pos_ref.x() - position.x()) * (pos_ref.x() - position.x()) + (pos_ref.y() - position.y()) * (pos_ref.y() - position.y()));
                //if (xy_distance < 0.1 && !(pos_ref.x()==0 && pos_ref.y() == 0 && yaw_ref == 0))
                //{
                //    yaw_ref = yaw;
                //}

                real_T yaw_ref_yaw_diff = 0.0f;
                if (abs(yaw_ref-yaw) > M_PIf)
                {
                    if (yaw >= 0)
                    {
                        yaw_ref_yaw_diff = 2.0f * M_PIf - abs(yaw) - abs(yaw_ref);
                    }
                    else
                    {
                        yaw_ref_yaw_diff = - (2.0f * M_PIf - abs(yaw) - abs(yaw_ref));
                    }
                }
                else
                {
                    yaw_ref_yaw_diff = yaw_ref - yaw;
                }

                yaw = infinite_yaw_angle;
                if (yaw_ref_yaw_diff >= 0)
                {
                    yaw_ref = yaw + std::min(abs(yaw_ref_yaw_diff), maximum_yaw_dif);
                }
                else
                {
                    yaw_ref = yaw - std::min(abs(yaw_ref_yaw_diff), maximum_yaw_dif);
                }

                if (dummy)
                {
                    yaw_ref = dummy_yaw_ref;
                }
                current_corrected_yaw_ref = yaw_ref;
                current_orientation.x() = roll;
                current_orientation.y() = pitch;
                current_orientation.z() = yaw;

                // TransferFcn3
                real_T filtered_yaw_cmd_1 = only_filter(heading_low_pass_filter_1, yaw_ref);

                // TransferFcn1
                real_T filtered_yaw_cmd_2_dot = filter_derivative(heading_low_pass_filter_2, filtered_yaw_cmd_1, filtered_yaw_cmd_2_last);

                // Obtaining psi_dot_cmd
                real_T psi_dot_cmd = (filtered_yaw_cmd_1 - yaw) * psi_P + filtered_yaw_cmd_2_dot;

                // Obtaining r_ref
                real_T r_ref = psi_dot_cmd * cos(roll) * cos(pitch) - sin(roll) * theta_dot;

                // TransferFcn2
                real_T filtered_r_cmd_dot = filter_derivative(heading_low_pass_filter_3, r_ref, filtered_r_cmd_last);

                // Obtaining r_dot_comd
                real_T r_dot_cmd = (r_ref - r) * r_P + filtered_r_cmd_dot;

                return r_dot_cmd;
            }

            u_pqr_omega_ref attitude_controller()
            {
                // Setting up the inputs
                real_T roll = att_states.x();
                real_T pitch = att_states.y();
                real_T yaw = att_states.z();
                Quaternionr orientation = VectorMath::toQuaternion(pitch, roll, yaw);
                ndi_omegasumref ndiDirectlyFromA_input = ndiDirectlyFromA();
                Vector3r nd_i = ndiDirectlyFromA_input.nd_i;
                real_T omega_sum_ref = ndiDirectlyFromA_input.omega_sum_ref;
                real_T r_dot_cmd = headingControllerNDI();
                Vector3r pqr = pqr_states;
                current_pqr = pqr;

                // Obtaining ndi_dot
                real_T filtered_nid_x_dot = filter_derivative(attitude_controller_low_pass_filter_x, nd_i.x(), filtered_ndi_x_last);
                real_T filtered_nid_y_dot = filter_derivative(attitude_controller_low_pass_filter_y, nd_i.y(), filtered_ndi_y_last);
                real_T filtered_nid_z_dot = filter_derivative(attitude_controller_low_pass_filter_z, nd_i.z(), filtered_ndi_z_last);

                Vector3r ndi_dot(filtered_nid_x_dot, filtered_nid_y_dot, filtered_nid_z_dot);

                // ind projection on b frame, states of the controller (orientation reversed)
                Vector3r h = VectorMath::transformToBodyFrame(nd_i, orientation);
                Vector3r ndi_dot_b = VectorMath::transformToBodyFrame(ndi_dot, orientation);

                //compute command p and q based on NDI
                real_T nxd = primary_axis.x();
                real_T nyd = primary_axis.y();

                real_T nxdot_cmd = kx * (h.x() - nxd);
                real_T nydot_cmd = ky * (h.y() - nyd);

                real_T pq_des_1 = (nydot_cmd + h.x() * pqr.z() + ndi_dot_b.y()) / h.z();
                real_T pq_des_2 = -(nxdot_cmd - h.y() * pqr.z() + ndi_dot_b.x()) / h.z();

                // compute command M based on NDI
                real_T p_dot_cmd = pq_des_dot_1 + kp_gain * (pqr.x() - pq_des_1);
                real_T q_dot_cmd = pq_des_dot_2 + kq_gain * (pqr.y() - pq_des_2);

                // Compute the pq_des_dot from the comptued pq_des
                pq_des_dot_1 = filter_derivative(pq_des_low_pass_filter_1, pq_des_1, filtered_pq_des_1_last);
                pq_des_dot_2 = filter_derivative(pq_des_low_pass_filter_2, pq_des_2, filtered_pq_des_2_last);

                Vector3r u_pqr(p_dot_cmd, q_dot_cmd, r_dot_cmd);
                current_pqr_ref = u_pqr;
                return u_pqr_omega_ref{ omega_sum_ref, u_pqr };;
            }

            Eigen::Matrix<real_T, 4, 1> INDIAllocator()
            {
                // Retrieve all the inputs
                Vector3r pqr = pqr_states;
                u_pqr_omega_ref attitude_controller_input = attitude_controller();
                Vector3r pqr_dot_cmd = attitude_controller_input.u_pqr;
                real_T omega_sum_ref = attitude_controller_input.omega_sum_ref;

                // Low pass filter and derivative of pqr
                real_T filtered_pqr_x_dot = filter_derivative(pqr_low_pass_filter_x, pqr.x(), filtered_pqr_x_last);
                real_T filtered_pqr_y_dot = filter_derivative(pqr_low_pass_filter_y, pqr.y(), filtered_pqr_y_last);
                real_T filtered_pqr_z_dot = filter_derivative(pqr_low_pass_filter_z, pqr.z(), filtered_pqr_z_last);

                Vector3r Omega_f_dot(filtered_pqr_x_dot, filtered_pqr_y_dot, filtered_pqr_z_dot);

                Eigen::Matrix<real_T, 4, 4> G;
                G << 15.0f, -15.0f, -15.0f, 15.0f,
                    14.0f, 14.0f, -14.0f, -14.0f,
                    -0.25, 0.25, -0.25, 0.25,
                    1.0f, 1.0f, 1.0f, 1.0f;

                G.row(2) = signr * G.row(2);

                Eigen::Matrix<real_T, 4, 1> nu;
                nu << pqr_dot_cmd.x(),
                    pqr_dot_cmd.y(),
                    pqr_dot_cmd.z(),
                    omega_sum_ref;

                Eigen::Matrix<real_T, 4, 1> nu_omega;
                nu_omega << filtered_pqr_x_dot,
                    filtered_pqr_y_dot,
                    filtered_pqr_z_dot,
                    omega_sum;
                Eigen::Matrix<real_T, 4, 1> dnu;
                Eigen::Matrix<real_T, 4, 1> du;
                Eigen::Matrix<real_T, 4, 4> G2;
                dnu = nu - nu_omega;

                if (failed == 0)
                {
                    // In the case that the propeller is not broken
                    G2 << 0.0f, 0.0f, 0.0f, 0.0f,
                        0.0f, 0.0f, 0.0f, 0.0f,
                        signr * -0.0612093, signr * 0.0653670, signr * -0.0657419, signr * 0.0654516,
                        0.0f, 0.0f, 0.0f, 0.0f;
                    du = (G / 1000.0f + G2).inverse() * (dnu + G2 * du_last) * 2.0f * M_PIf / 60.0f;
                }
                else
                {
                    // In the case that a propeller is broken
                    Eigen::Matrix<real_T, 4, 4> G_shrink;
                    Eigen::Matrix<real_T, 4, 1> broken_prop_col;
                    Eigen::Matrix<real_T, 1, 4> broken_prop_row;
                    broken_prop_col << 0,
                        0.0f,
                        0.0f,
                        0.0f;
                    broken_prop_row << 0.0f, 0.0f, 0.0f, 0.0f;
                    G_shrink = G;
                    G_shrink.col(failed - 1) = broken_prop_col;
                    G_shrink.row(2) = broken_prop_row;
                    du = G_shrink.completeOrthogonalDecomposition().pseudoInverse() * dnu * 1000.0f * 2.0f * M_PIf / 60.0f;
                }

                // Variables used for debugging
                real_T du1 = du(0);
                real_T du2 = du(1);
                real_T du3 = du(2);
                real_T du4 = du(3);

                // Obtaning the final motor rotations
                Eigen::Matrix<real_T, 4, 1> w_cmd;
                w_cmd = w_f + du;


                // Setting up the commanded rotations of the propellers by saturating their values
                for (int i = 0; i < 4; i++)
                {
                    w_cmd(i) = std::max(std::min(w_cmd(i), w_max), w_min);
                    du(i) = w_cmd(i) - w_f(i);
                }
                du_last = du;

                // Variables used for debugging
                du1 = du(0);
                du2 = du(1);
                du3 = du(2);
                du4 = du(3);
                real_T w_cmd_1 = w_cmd(0);
                real_T w_cmd_2 = w_cmd(1);
                real_T w_cmd_3 = w_cmd(2);
                real_T w_cmd_4 = w_cmd(3);


                if (failed != 0)
                {
                    w_cmd(failed-1) = 0.0;
                }

                return w_cmd;
            }

            // Actuator dynamics
            virtual std::vector<real_T> actuator_dyn(Kinematics::State previous) override
            {
                previous_ = previous;
                Eigen::Matrix<real_T, 4, 1> omega_cmd;
                omega_cmd = INDIAllocator();

                // Applying the low_pass_filter
                real_T filtered_actuator_1 = only_filter(actuator_low_pass_filter_1, omega_cmd(0));
                real_T filtered_actuator_2 = only_filter(actuator_low_pass_filter_2, omega_cmd(1));
                real_T filtered_actuator_3 = only_filter(actuator_low_pass_filter_3, omega_cmd(2));
                real_T filtered_actuator_4 = only_filter(actuator_low_pass_filter_4, omega_cmd(3));

                std::vector<real_T> omega = { filtered_actuator_1 , filtered_actuator_2, filtered_actuator_3, filtered_actuator_4 };

                for (int i = 0; i < 4; i++)
                {
                    if (locked_propeller[i] == true) {
                        omega[i] = lock_coefficients[i];
                    }
                    else {
                        real_T prop_damage = propeller_damage_coefficients[i];
                        omega[i] = omega[i] * prop_damage;
                    }
                }

                // Obtaining w_obs
                real_T filtered_w_obs_1 = only_filter(w_obs_low_pass_filter_1, filtered_actuator_1);
                real_T filtered_w_obs_2 = only_filter(w_obs_low_pass_filter_2, filtered_actuator_2);
                real_T filtered_w_obs_3 = only_filter(w_obs_low_pass_filter_3, filtered_actuator_3);
                real_T filtered_w_obs_4 = only_filter(w_obs_low_pass_filter_4, filtered_actuator_4);

                // Updating the class members of w_obs and omega_sum
                w_f(0) = filtered_w_obs_1;
                w_f(1) = filtered_w_obs_2;
                w_f(2) = filtered_w_obs_3;
                w_f(3) = filtered_w_obs_4;
                omega_sum = filtered_w_obs_1 + filtered_w_obs_2 + filtered_w_obs_3 + filtered_w_obs_4;
                current_omegas = omega;
                return omega;
            }

            // Methods related to the data gathering of position reference
            void storePosRefData() override
            {
                if (pos_ref_activate_store) {
                    std::vector<float> local_pos_ref_data = { current_pos_ref.x(), current_pos_ref.y(), current_pos_ref.z()};
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int pos_ref_threshold = UE4_second / pos_ref_sample_rate;
                    uint64_t time_threshold = pos_ref_time_old + pos_ref_threshold;
                    if (time_new >= time_threshold) {
                        pos_ref_time_old = time_new;
                        pos_ref_data.push_back(local_pos_ref_data);
                    }
                }
            }

            void setPosRefAct(bool activation, float sample_rate) override
            {
                pos_ref_activate_store = activation;
                pos_ref_sample_rate = sample_rate;
            }

            void cleanPosRefSD() override
            {
                pos_ref_activate_store = false;
                pos_ref_data.clear();
            }

            std::vector<std::vector<float>> getPosRefStoredData() override
            {
                return pos_ref_data;
            }

            // Methods related to the data gathering of position reference
            void storePosErrorData() override
            {
                if (pos_error_activate_store) {
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

            void setPosErrorAct(bool activation, float sample_rate) override
            {
                pos_error_activate_store = activation;
                pos_error_sample_rate = sample_rate;
            }

            void cleanPosErrorSD() override
            {
                pos_error_activate_store = false;
                pos_error_data.clear();
            }

            std::vector<std::vector<float>> getPosErrorStoredData() override
            {
                return pos_error_data;
            }

            // Methods related to the data gathering of position error derivative
            void storePosErrorDotData() override
            {
                if (pos_error_dot_activate_store) {
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

            void setPosErrorDotAct(bool activation, float sample_rate) override
            {
                pos_error_dot_activate_store = activation;
                pos_error_dot_sample_rate = sample_rate;
            }

            void cleanPosErrorDotSD() override
            {
                pos_error_dot_activate_store = false;
                pos_error_dot_data.clear();
            }

            std::vector<std::vector<float>> getPosErrorDotStoredData() override
            {
                return pos_error_dot_data;
            }

            // Methods related to the data gathering of velocity reference
            void storeVelRefData() override
            {
                if (vel_ref_activate_store) {
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

            void setVelRefAct(bool activation, float sample_rate) override
            {
                vel_ref_activate_store = activation;
                vel_ref_sample_rate = sample_rate;
            }

            void cleanVelRefSD() override
            {
                vel_ref_activate_store = false;
                vel_ref_data.clear();
            }

            std::vector<std::vector<float>> getVelRefStoredData() override
            {
                return vel_ref_data;
            }

            // Methods related to the data gathering of velocity
            void storeVelData() override
            {
                if (vel_activate_store) {
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

            void setVelAct(bool activation, float sample_rate) override
            {
                vel_activate_store = activation;
                vel_sample_rate = sample_rate;
            }

            void cleanVelSD() override
            {
                vel_activate_store = false;
                vel_data.clear();
            }

            std::vector<std::vector<float>> getVelStoredData() override
            {
                return vel_data;
            }

            // Methods related to the data gathering of reference acceleration
            void storeAccRefData() override
            {
                if (acc_ref_activate_store) {
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

            void setAccRefAct(bool activation, float sample_rate) override
            {
                acc_ref_activate_store = activation;
                acc_ref_sample_rate = sample_rate;
            }

            void cleanAccRefSD() override
            {
                acc_ref_activate_store = false;
                acc_ref_data.clear();
            }

            std::vector<std::vector<float>> getAccRefStoredData() override
            {
                return acc_ref_data;
            }

            // Methods related to the data gathering of reference rotational rates 
            void storePqrRefData() override
            {
                if (pqr_ref_activate_store) {
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

            void setPqrRefAct(bool activation, float sample_rate) override
            {
                pqr_ref_activate_store = activation;
                pqr_ref_sample_rate = sample_rate;
            }

            void cleanPqrRefSD() override
            {
                pqr_ref_activate_store = false;
                pqr_ref_data.clear();
            }

            std::vector<std::vector<float>> getPqrRefStoredData() override
            {
                return pqr_ref_data;
            }

            // Methods related to the data gathering of rotational rates 
            void storePqrData() override
            {
                if (pqr_activate_store) {
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

            void setPqrAct(bool activation, float sample_rate) override
            {
                pqr_activate_store = activation;
                pqr_sample_rate = sample_rate;
            }

            void cleanPqrSD() override
            {
                pqr_activate_store = false;
                pqr_data.clear();
            }

            std::vector<std::vector<float>> getPqrStoredData() override
            {
                return pqr_data;
            }

            // Methods related to the data gathering of reference thrust 
            void storeThrustRefData() override
            {
                if (pqr_ref_activate_store) {
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

            void setThrustRefAct(bool activation, float sample_rate) override
            {
                thrust_ref_activate_store = activation;
                thrust_ref_sample_rate = sample_rate;
            }

            void cleanThrustRefSD() override
            {
                thrust_ref_activate_store = false;
                thrust_ref_data.clear();
            }

            std::vector<std::vector<float>> getThrustRefStoredData() override
            {
                return thrust_ref_data;
            }

            // Methods related to the data gathering of reference thrust 
            void storeOmegasData() override
            {
                if (pqr_ref_activate_store) {
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

            void setOmegasAct(bool activation, float sample_rate) override
            {
                omegas_activate_store = activation;
                omegas_sample_rate = sample_rate;
            }

            void cleanOmegasSD() override
            {
                omegas_activate_store = false;
                omegas_data.clear();
            }

            std::vector<std::vector<float>> getOmegasStoredData() override
            {
                return omegas_data;
            }

            // Methods related to the data gathering of yaw reference
            void storeYawRefData() override
            {
                if (yaw_ref_activate_store) {
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

            void setYawRefAct(bool activation, float sample_rate) override
            {
                yaw_ref_activate_store = activation;
                yaw_ref_sample_rate = sample_rate;
            }

            void cleanYawRefSD() override
            {
                yaw_ref_activate_store = false;
                yaw_ref_data.clear();
            }

            std::vector<std::vector<float>> getYawRefStoredData() override
            {
                return yaw_ref_data;
            }

            // Methods related to the data gathering of orientation data (phi, theta and psi)
            void storeOrientationData() override
            {
                if (orientation_activate_store) {
                    std::vector<float> local_orientation_ref_data = { current_orientation.x(),  current_orientation.y(), current_orientation.z()};
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int orientation_threshold = UE4_second / orientation_sample_rate;
                    uint64_t time_threshold = orientation_time_old + orientation_threshold;
                    if (time_new >= time_threshold) {
                        orientation_time_old = time_new;
                        orientation_data.push_back(local_orientation_ref_data);
                    }
                }
            }

            void setOrientationAct(bool activation, float sample_rate) override
            {
                orientation_activate_store = activation;
                orientation_sample_rate = sample_rate;
            }

            void cleanOrientationSD() override
            {
                orientation_activate_store = false;
                orientation_data.clear();
            }

            std::vector<std::vector<float>> getOrientationStoredData() override
            {
                return orientation_data;
            }

            // Methods related to the data gathering of the position integrator within the first subsystem of the primary axis INDI controller
            void storePositionIntegratorData() override
            {
                if (position_integrator_activate_store) {
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

            void setPositionIntegratorAct(bool activation, float sample_rate) override
            {
                position_integrator_activate_store = activation;
                position_integrator_sample_rate = sample_rate;
            }

            void cleanPositionIntegratorSD() override
            {
                position_integrator_activate_store = false;
                position_integrator_data.clear();
            }

            std::vector<std::vector<float>> getPositionIntegratorStoredData() override
            {
                return position_integrator_data;
            }

            // Methods related to the data gathering of the PI thrust controller within the first subsystem of the primary axis INDI controller
            void storeThrustPiData() override
            {
                if (thrust_PI_activate_store) {
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

            void setThrustPiAct(bool activation, float sample_rate) override
            {
                thrust_PI_activate_store = activation;
                thrust_PI_sample_rate = sample_rate;
            }

            void cleanThrustPiSD() override
            {
                thrust_PI_activate_store = false;
                thrust_PI_data.clear();
            }

            std::vector<std::vector<float>> getThrustPiStoredData() override
            {
                return thrust_PI_data;
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

            // Methods related to the PWMs data gathering
            void storePWMData() override
            {
                if (PWM_activate_store) {
                    PWMs[0] = board_->getMotorControlSignal(0);
                    PWMs[1] = board_->getMotorControlSignal(1);
                    PWMs[2] = board_->getMotorControlSignal(2);
                    PWMs[3] = board_->getMotorControlSignal(3);
                    std::vector<float> local_PWM_data = { PWMs[0], PWMs[1], PWMs[2], PWMs[3] };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int PWM_threshold = UE4_second / PWM_sample_rate;
                    uint64_t time_threshold = PWM_time_old + PWM_threshold;
                    if (time_new >= time_threshold) {
                        PWM_time_old = time_new;
                        PWM_data.push_back(local_PWM_data);
                    }
                }
            }

            void setPWMAct(bool activation, float sample_rate) override
            {
                PWM_activate_store = activation;
                PWM_sample_rate = sample_rate;
            }

            void cleanPWMSD() override
            {
                PWM_activate_store = false;
                PWM_data.clear();
            }

            std::vector<std::vector<float>> getPWMStoredData() override
            {
                return PWM_data;
            }

            // Methods related to the ground truth position data gathering
            void storePositionData() override
            {
                if (position_activate_store) {
                    std::vector<float> local_position_data = { current_pos.x(), current_pos.y(), current_pos.z() };
                    uint64_t time_new = local_IMU_data.time_stamp;
                    int position_threshold = UE4_second / position_sample_rate;
                    uint64_t time_threshold = position_time_old + position_threshold;
                    if (time_new >= time_threshold) {
                        position_time_old = time_new;
                        position_data.push_back(local_position_data);
                    }
                }
            }

            void setPositionAct(bool activation, float sample_rate) override
            {
                position_activate_store = activation;
                position_sample_rate = sample_rate;
            }

            void cleanPositionSD() override
            {
                position_activate_store = false;
                position_data.clear();
            }

            std::vector<std::vector<float>> getPositionStoredData() override
            {
                return position_data;
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

            // Methods related to the drone teleportation
            void setTeleportYawReference(float yaw_angle_ref) override
            {
                teleport_yaw_deg = yaw_angle_ref;
            }

            // Methods related to the drone failures
            void setDamageCoeff(float new_coeffs[4]) override
            {
                propeller_damage_coefficients[0] = new_coeffs[0];
                propeller_damage_coefficients[1] = new_coeffs[1];
                propeller_damage_coefficients[2] = new_coeffs[2];
                propeller_damage_coefficients[3] = new_coeffs[3];
                if (propeller_damage_coefficients[0] < 1)
                {
                    failed = 1;
                }
                else if (propeller_damage_coefficients[1] < 1)
                {
                    failed = 2;
                }
                else if (propeller_damage_coefficients[2] < 1)
                {
                    failed = 3;
                }
                else if (propeller_damage_coefficients[3] < 1)
                {
                    failed = 4;
                }
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

                switch (controller_type_enum) {
                    // roll gain, pitch gain, yaw gain, and no gains in throttle / z axis
                case simple_flight::GoalModeType::AngleRate:
                    kp_axis4 = { kp[0], kp[1], kp[2], 1.0 };
                    ki_axis4 = { ki[0], ki[1], ki[2], 0.0 };
                    kd_axis4 = { kd[0], kd[1], kd[2], 0.0 };
                    params_.angle_rate_pid.p.setValues(kp_axis4);
                    params_.angle_rate_pid.i.setValues(ki_axis4);
                    params_.angle_rate_pid.d.setValues(kd_axis4);
                    params_.gains_changed = true;
                    break;
                case simple_flight::GoalModeType::AngleLevel:
                    kp_axis4 = { kp[0], kp[1], kp[2], 1.0 };
                    ki_axis4 = { ki[0], ki[1], ki[2], 0.0 };
                    kd_axis4 = { kd[0], kd[1], kd[2], 0.0 };
                    params_.angle_level_pid.p.setValues(kp_axis4);
                    params_.angle_level_pid.i.setValues(ki_axis4);
                    params_.angle_level_pid.d.setValues(kd_axis4);
                    params_.gains_changed = true;
                    break;
                case simple_flight::GoalModeType::VelocityWorld:
                    kp_axis4 = { kp[1], kp[0], 0.0, kp[2] };
                    ki_axis4 = { ki[1], ki[0], 0.0, ki[2] };
                    kd_axis4 = { kd[1], kd[0], 0.0, kd[2] };
                    params_.velocity_pid.p.setValues(kp_axis4);
                    params_.velocity_pid.i.setValues(ki_axis4);
                    params_.velocity_pid.d.setValues(kd_axis4);
                    params_.gains_changed = true;
                    break;
                case simple_flight::GoalModeType::PositionWorld:
                    kp_axis4 = { kp[1], kp[0], 0.0, kp[2] };
                    ki_axis4 = { ki[1], ki[0], 0.0, ki[2] };
                    kd_axis4 = { kd[1], kd[0], 0.0, kd[2] };
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
            ImuBase::Output local_IMU_data;

            // Variables related to potential failures
            float propeller_damage_coefficients[4] = { 1.0, 1.0, 1.0, 1.0 };
            bool locked_propeller[4] = { false, false, false, false };
            float lock_coefficients[4] = { 1.0, 1.0, 1.0, 1.0 };
            float PWMs[4] = { 1.0, 1.0, 1.0, 1.0 };
            int failed = 0;

            // Dummy variables
            bool dummy = false;
            bool correct_start = false;
            real_T dummy_yaw_ref = 0;
            real_T dummy_x = 0;
            real_T dummy_y = 0;
            real_T dummy_z = -3;
            bool activate_rounding = false;
            int activate_correct_G = 0;
            bool divide_1000 = false;

            // Variables used for the Gaussian noise
            std::default_random_engine generator;
            bool activate_measurement_noise = false;
            Vector3r cov_pqr = 0.7 * Vector3r(0.8599, 0.0250, 0.0861);
            Vector3r cov_att = 0.7 * Vector3r(0.0001, 0.0001, 0.0001);
            Vector3r cov_xyz = 0.7 * Vector3r(0.00001, 0.00001, 0.00001);
            Vector3r cov_vxyz = 0.7 * Vector3r(0.0002, 0.0002, 0.001);

            Vector3r mean_pqr = Vector3r(0, 0, 0);
            Vector3r mean_att = Vector3r(0, 0, 0);
            Vector3r mean_xyz = Vector3r(0, 0, 0);
            Vector3r mean_vxyz = Vector3r(0, 0, 0);

            // States
            bool one_step_back_dyn = true;
            Kinematics::State previous_;
            Vector3r pqr_states;
            Vector3r att_states;
            Vector3r xyz_states;
            Vector3r vxyz_states;

            // Variables for the tuning of the smoothing of the initial acceleration
            real_T alpha_tuning = 10;
            real_T v_tuning_1 = 2;
            real_T v_tuning_2 = 4;
            real_T v_tuning_3 = 0.5;

            // Members for fixing the controller
            // setting up all the integrators
            TTimePoint last_time_integral;
            real_T dt_real_integration;
            real_T integrator_pos_x = 0;
            real_T integrator_pos_x_dot_current = 0;
            real_T integrator_pos_x_dot_previous = 0;
            real_T integrator_pos_y = 0;
            real_T integrator_pos_y_dot_current = 0;
            real_T integrator_pos_y_dot_previous = 0;
            real_T integrator_pos_z = 0;
            real_T integrator_pos_z_dot_current = 0;
            real_T integrator_pos_z_dot_previous = 0;
            real_T integrator_thrust_ref = 0;
            real_T integrator_thrust_ref_dot_current = 0;
            real_T integrator_thrust_ref_dot_previous = 0;

            // Parameters for the PID block
            //real_T vel_P = 1.2;
            //real_T vel_D = 0.1;
            //real_T pos_P = 1.5;
            //real_T pos_I = 0;
            //real_T pos_D = 1.3;
            //real_T thrust_P = 8.;
            //real_T thrust_I = 0.5;
            //real_T psi_P = 20.;
            //real_T r_P = 40.;

            real_T vel_P = 1.5;
            real_T vel_D = 0;
            real_T pos_P = 0.5;
            real_T pos_I = 0;
            real_T pos_D = 0;
            real_T thrust_P = 8.;
            real_T thrust_I = 0.5;
            real_T psi_P = 20.;
            real_T r_P = 40.;

            // Parameters for the computation of nd_i directly from a
            real_T gravity_Delft = 9.8124;
            real_T specific_thrust_2_omegasqure = 4.7197e6 / 0.8;
            real_T rpm2rad = 2 * M_PIf / 60;

            // Parameters for the yawing
            real_T teleport_yaw_deg = 0;
            real_T n_half_rotations = 0;
            int previous_yaw_sign = 0;
            real_T previous_yaw_angle = 0;
            real_T infinite_yaw_angle = 0;
            //real_T maximum_yaw_dif = 0.5235987755982988;
            real_T maximum_yaw_dif = 10000000000;

            // Variables for the first order time filters
            int integration_method = 1;
            FirstOrderFilter<real_T> pos_ref_low_pass_filter_x;
            FirstOrderFilter<real_T> pos_ref_low_pass_filter_y;
            FirstOrderFilter<real_T> pos_ref_low_pass_filter_z;
            FirstOrderFilter<real_T> pos_error_low_pass_filter_x;
            FirstOrderFilter<real_T> pos_error_low_pass_filter_y;
            FirstOrderFilter<real_T> pos_error_low_pass_filter_z;
            FirstOrderFilter<real_T> heading_low_pass_filter_1;
            FirstOrderFilter<real_T> heading_low_pass_filter_2;
            FirstOrderFilter<real_T> heading_low_pass_filter_3;
            FirstOrderFilter<real_T> attitude_controller_low_pass_filter_x;
            FirstOrderFilter<real_T> attitude_controller_low_pass_filter_y;
            FirstOrderFilter<real_T> attitude_controller_low_pass_filter_z;
            FirstOrderFilter<real_T> pq_des_low_pass_filter_1;
            FirstOrderFilter<real_T> pq_des_low_pass_filter_2;
            FirstOrderFilter<real_T> pqr_low_pass_filter_x;
            FirstOrderFilter<real_T> pqr_low_pass_filter_y;
            FirstOrderFilter<real_T> pqr_low_pass_filter_z;
            FirstOrderFilter<real_T> actuator_low_pass_filter_1;
            FirstOrderFilter<real_T> actuator_low_pass_filter_2;
            FirstOrderFilter<real_T> actuator_low_pass_filter_3;
            FirstOrderFilter<real_T> actuator_low_pass_filter_4;
            FirstOrderFilter<real_T> w_obs_low_pass_filter_1;
            FirstOrderFilter<real_T> w_obs_low_pass_filter_2;
            FirstOrderFilter<real_T> w_obs_low_pass_filter_3;
            FirstOrderFilter<real_T> w_obs_low_pass_filter_4;
            real_T pos_error_x_last = 0;
            real_T pos_error_y_last = 0;
            real_T pos_error_z_last = 0;
            real_T v_error_x_last = 0;
            real_T v_error_y_last = 0;
            real_T v_error_z_last = 0;
            real_T filtered_yaw_cmd_2_last = 0;
            real_T filtered_r_cmd_last = 0;
            real_T filtered_ndi_x_last = 0;
            real_T filtered_ndi_y_last = 0;
            real_T filtered_ndi_z_last = 0;
            real_T filtered_pq_des_1_last = 0;
            real_T filtered_pq_des_2_last = 0;
            real_T filtered_pqr_x_last = 0;
            real_T filtered_pqr_y_last = 0;
            real_T filtered_pqr_z_last = 0;

            // Variables for the function within the yaw command block
            real_T theta_dot = 0;

            // Variables for the attitude controller block
            Vector3r primary_axis = Vector3r(0, 0, -1);
            real_T kx = -5;
            real_T ky = -5;
            real_T kp_gain = -20;
            real_T kq_gain = -20;
            real_T pq_des_dot_1 = 0;
            real_T pq_des_dot_2 = 0;

            // Variables of the INDI Allocator
            real_T w_max = 1256;
            real_T w_min = 300;
            real_T t_indi = 0.04;
            int signr = -1;
            real_T omega_sum = 0;
            Eigen::Matrix<real_T, 4, 1> du_last;
            Eigen::Matrix<real_T, 4, 1> w_f;

            // Variables of the actuator dynamics
            real_T t_w = 1.0 / 40;

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

            int remote_control_id_ = 0;
            simple_flight::Params params_;

            unique_ptr<AirSimSimpleFlightBoard> board_;
            unique_ptr<AirSimSimpleFlightCommLink> comm_link_;
            unique_ptr<AirSimSimpleFlightEstimator> estimator_;
            unique_ptr<simple_flight::IFirmware> firmware_;

            MultirotorApiParams safety_params_;

            RCData last_rcData_;
        };

    }
} //namespace
#endif
