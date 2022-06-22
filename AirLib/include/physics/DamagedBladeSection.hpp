// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file DamagedBladeSection.hpp
 *
 * @brief Provides the BladeSection, class for the aerodynamic model identification and computation of forces and moments.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: j.i.dealvearcardenas@student.tudelft.nl
 *
 */

#ifndef physics_DamagedBladeSection_hpp
#define physics_DamagedBladeSection_hpp

#include <Eigen/Dense>
#include "common/Common.hpp"
#include <numeric>

namespace msr {
    namespace airlib {
        class DamagedBladeSection {
        public:
            DamagedBladeSection(const int& section_number, const real_T& chord, const real_T& section_length, const real_T& average_twist, const real_T& initial_chord, const real_T& final_chord, const real_T& radius_hub,
                const int& rotation_direction, const real_T& air_density=1.225)
            {
                section_number_ = section_number;
                c_ = chord;
                dr_ = section_length;
                twist_ = average_twist;
                c1_ = initial_chord;
                c2_ = final_chord;
                radius_hub_ = radius_hub;
                rotation_direction_ = rotation_direction;
                rho_ = air_density;

                y_ = (section_number_ + 0.5f) * dr_ + radius_hub_;
                S_ = c_ * dr_;

            }
            //%% Structures for data return
            struct retVals_compute_velocity {        // Declare a local structure 
                real_T Vl, V_total;
            };

            struct retVals_LS_term_params {        // Declare a local structure 
                real_T V_total, aoa;
            };

            struct retVals_torque_force {        // Declare a local structure 
                real_T dQ;
                Vector3r dF;
            };

            struct retVals_thrust_moment {        // Declare a local structure 
                real_T dT;
                Vector3r dM;
            };

            //%% Class functions
            // Compute the angle of attack
            real_T computeAoa(const Vector3r& rotor_speed, const real_T& Vx, const real_T& vi) {
                // Computation of the angle of attack
                real_T Vz_bl = -rotor_speed.z() + vi;
                real_T velocity_angle = atan(Vz_bl / abs(Vx));

                // Situation when the velocity vector is coming from the back of the blade
                real_T aoa;
                if (-rotation_direction_ * Vx > 0) {
                    aoa = twist_ + M_PIf + velocity_angle;
                    if (aoa > M_PIf) {
                        aoa = 2 * M_PIf - aoa;
                    }  
                } else {
                    aoa = twist_ - velocity_angle;
                }

                if (aoa * rad2deg_factor_ < -25 || aoa * rad2deg_factor_ > 25) {
                    stall_ = true;
                }

                return aoa;
            }

            // Compute the velocity along the chord of the blade section
            retVals_compute_velocity computeVelocity(const real_T& omega, const real_T& position_rotor, const Vector3r& rotor_speed, const real_T& vi) {
                real_T Vr = rotation_direction_ * omega * y_;
                real_T Vx_bl = -(sin(position_rotor) * rotor_speed.x() - cos(position_rotor) * rotor_speed.y());

                // The velocity in the x - direction used for the force computation
                // is the sum of the air velocity due to the body displacementand the propeller rotation
                real_T Vl = Vr + Vx_bl;

                // The total velocity experienced by the blade cross section is the sum of the velocity components in the x
                // and z directions
                real_T V_z = -rotor_speed.z() + vi;
                real_T V_total = sqrt(powf(Vl, 2) + powf(V_z, 2));
                return retVals_compute_velocity{Vl, V_total};
            }

            // Method that computes the velocity and the angle of attack required for the computation of Least Squares
            retVals_LS_term_params computeLSTermParams(const real_T& omega, const real_T& position_rotor, const Vector3r& rotor_speed, 
                std::function<real_T(real_T, real_T)> inflow_data) {
                stall_ = false;

                // Compute the psi angle of the blade
                real_T Vxy_angle = atan2(rotor_speed.y(), rotor_speed.x());
                real_T inter_vector_angle = acos(std::max(std::min((rotor_speed.x() * cos(position_rotor) + rotor_speed.y() * sin(position_rotor)) / (sqrt(powf(rotor_speed.x(), 2) + powf(rotor_speed.y(), 2))), 1.0f), -1.0f));
                real_T blade_angle = position_rotor;
                if (blade_angle > M_PIf) {
                    blade_angle = -(2 * M_PIf - position_rotor);
                }

                if (blade_angle < Vxy_angle) {
                    inter_vector_angle = -inter_vector_angle;
                }

                real_T psi = M_PIf + rotation_direction_ * inter_vector_angle;

                // Computation of the induced velocity
                real_T vi = inflow_data(y_, psi);

                // Compute the velocities of the vehicle and the angle of attack
                retVals_compute_velocity velocities= computeVelocity(omega, position_rotor, rotor_speed, vi);
                real_T Vx = velocities.Vl;
                real_T V_total = velocities.V_total;
                real_T aoa = computeAoa(rotor_speed, Vx, vi);

                if (isnan(aoa) || isnan(V_total))
                {
                    int a = 1;
                }

                return retVals_LS_term_params{ V_total, aoa };
            }

            // Method that computes the torque produced by the blade section drag and the corresponding force in the x-y plane
            retVals_torque_force computeTorqueForce(const real_T& omega, const Vector3r& rotor_speed, const real_T& position_rotor, const vector<real_T>& cla_coeffs,
                const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {

                // Compute parameters
                retVals_LS_term_params BS_params = computeLSTermParams(omega, position_rotor, rotor_speed, inflow_data);
                real_T Vl = BS_params.V_total;
                real_T aoa = BS_params.aoa;

                // Computation of cl
                real_T cl = 0.0f;
                if (stall_ == false) {
                    for (int i = 0; i < cla_coeffs.size(); i++) {
                        cl += cla_coeffs[i] * powf(aoa, i);
                    }
                }

                // Computation of cd
                real_T cd = 0.0f;
                for (int i = 0; i < cda_coeffs.size(); i++) {
                    cd += cda_coeffs[i] * powf(aoa, i);
                }

                // Drag and lift equations for the blade section
                real_T dL = 0.5 * rho_ * S_ * powf(Vl, 2) * cl;
                real_T dD = 0.5 * rho_ * S_ * powf(Vl, 2) * cd;
                real_T dF_abs = dL * sin(twist_ - aoa) + dD * cos(twist_ - aoa);

                // Torque equation for the blade section
                real_T dQ = -rotation_direction_ * y_ * dF_abs;

                // Computation of the forces in the x - y direction due to the force creating the torque
                real_T angle = position_rotor - rotation_direction_ * M_PIf / 2.0f;
                Vector3r dF = Vector3r(dF_abs * cos(angle), dF_abs * sin(angle), 0);
                return retVals_torque_force{dQ, dF};
            }

            // Method that computes the thrust produced by the blade section and its corresponding moment about the center of
            // the propeller caused by the thrust force
            retVals_thrust_moment computeThrustMoment(const real_T& omega, const Vector3r& rotor_speed, const real_T& position_rotor, const vector<real_T>& cla_coeffs,
                const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {
                // Compute parameters
                retVals_LS_term_params BS_params = computeLSTermParams(omega, position_rotor, rotor_speed, inflow_data);
                real_T Vl = BS_params.V_total;
                real_T aoa = BS_params.aoa;

                // Computation of cl
                real_T cl = 0;
                if (stall_ == false) {
                    for (int i = 0; i < cla_coeffs.size(); i++) {
                        cl += cla_coeffs[i] * powf(aoa, i);
                    }
                }

                // Computation of cd
                real_T cd = 0;
                for (int i = 0; i < cda_coeffs.size(); i++) {
                    cd += cda_coeffs[i] * powf(aoa, i);
                }

                // Drag and lift equations for the blade section
                real_T dL = 0.5 * rho_ * S_ * powf(Vl, 2) * cl;
                real_T dD = 0.5 * rho_ * S_ * powf(Vl, 2) * cd;
                real_T dT = -(dL * cos(twist_ - aoa) - dD * sin(twist_ - aoa));   // It is negative because it is in the negative z - direction.

                // Computation of the moment generated by the lift force about the center of the propeller
                Vector3r r = Vector3r(y_ * cos(position_rotor), y_ * sin(position_rotor), 0);
                Vector3r F = Vector3r(0, 0, dT);
                Vector3r dM = r.cross(F);

                return retVals_thrust_moment{dT, dM};
            }

        private:
            // Parameters that will be initialised in __init__
            int section_number_;
            real_T c_;
            real_T dr_;
            real_T twist_;
            real_T c1_;
            real_T c2_;
            real_T radius_hub_;
            int rotation_direction_;
            real_T rho_;

            // Other parameters initialised outside of __init__
            bool stall_ = false;
            real_T S_;
            real_T y_;


            // Constant expressions
            static constexpr real_T rad2deg_factor_ = 180.0f / M_PIf;
            static constexpr real_T deg2rad_factor_ = M_PIf / 180.0f;
        };

    }
}



#endif // !physics_DamagedBladeSection_hpp
