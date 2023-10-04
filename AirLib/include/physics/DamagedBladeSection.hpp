// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file DamagedBladeSection.hpp
 *
 * @brief Provides the BladeSection, class for the aerodynamic model identification and computation of forces and moments.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: jialvear@hotmail.com
 *
 */

#ifndef physics_DamagedBladeSection_hpp
#define physics_DamagedBladeSection_hpp

#include <Eigen/Dense>
#include "common/Common.hpp"
#include <numeric>

namespace msr {
    namespace airlib {
        /// <summary>
        /// Provides the DamagedBladeSection, class for the aerodynamic model identification and computation of forces and moments.
        /// DamagedBladeSection holds all the information related to a single blade element according to BEM theory.It is used for the
        /// computation of the angle of attackand velocity seen by each BladeSection. Additionally, it computes the contribution
        /// of the BladeSection lift and drag to the thrust force and torque.
        /// </summary>
        class DamagedBladeSection {
        public:
            /// <summary>
            /// Initialize a Damaged Blade Section instance.
            /// </summary>
            /// <param name="section_number">the id number of the blade section within its parent Blade class</param>
            /// <param name="chord">the average chord of the blade section</param>
            /// <param name="section_length">the length of the blade section</param>
            /// <param name="average_twist">the average twist experienced along the blade section</param>
            /// <param name="initial_chord">the chord at the root of the blade section</param>
            /// <param name="final_chord">the chord at the tip of the blade section</param>
            /// <param name="radius_hub">the radius of the middle propeller hub</param>
            /// <param name="rotation_direction">the direction of rotation of the propeller</param>
            /// <param name="air_density">the density of the air</param>
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
            /// <summary>
            ///  Compute the angle of attack
            /// </summary>
            /// <param name="rotor_speed">velocity experienced by the complete motor due to the translation and rotation of the body</param> 
            /// <param name="Vx">x-component of the velocity experienced by the blade in the blade coordinate frame. This means that it is the velocity component perpendicular to the blade</param>
            /// <param name="vi">induced velocity</param>
            /// <returns>the blade section angle of attack</returns>
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

            /// <summary>
            /// Compute the velocity along the chord of the blade section
            /// </summary>
            /// <param name="omega">rotational velocity of the rotor</param>
            /// <param name="position_rotor">current rotation of the propeller relative to the body coordinate frame. When the rotor
            /// position is at an angle of 90 degrees, then the bodyand the rotor coordinate frames coincide.When the rotor
            /// angle is 0 degrees, then the x - axis of the propeller is pointing towards the negative body y - axis and the y - axis
            /// of the propeller is pointing towards the positive body x - axis.</param>
            /// <param name="rotor_speed">velocity experienced by the complete motor due to the translation and rotation of the body</param>
            /// <param name="vi">induced velocity</param>
            /// <returns>velocity perpendicular to the blade section's chord in the x-y propeller plane (Vl) and the total velocity
            /// experienced by the blade section along the plane perpendicular to its chord(V_total).</returns>
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

            /// <summary>
            /// Method that computes the velocity and the angle of attack required for the computation of Least Squares
            /// </summary>
            /// <param name="omega">rotational velocity of the rotor</param>
            /// <param name="position_rotor">current rotation of the blade relative to the body coordinate frame</param>
            /// <param name="rotor_speed">velocity experienced by the complete motor due to the translation and rotation of the body</param>
            /// <param name="inflow_data">data regarding the inflow field, namely the uniform induced inflow field, induced inflow
            /// velocityand a lambda function that computes the linear induced field depending on the blade element distance
            /// from the huband angle with respect to the inflow.["v0", "lambda_0", "induced_velocity_func"(r, psi), "R"]</param>
            /// <returns>the velocity experienced by the blade section used in the lift equation and the angle of attack</returns>
            retVals_LS_term_params computeLSTermParams(const real_T& omega, const real_T& position_rotor, const Vector3r& rotor_speed, 
                std::function<real_T(real_T, real_T)> inflow_data) {
                stall_ = false;

                // Compute the psi angle of the blade
                real_T Vxy_angle = atan2(rotor_speed.y(), rotor_speed.x());
                real_T inter_vector_angle = acos(std::max(std::min((rotor_speed.x() * cos(position_rotor) + rotor_speed.y() * sin(position_rotor)) / (sqrt(powf(rotor_speed.x(), 2) + powf(rotor_speed.y(), 2))), 1.0f), -1.0f));
                real_T blade_angle = position_rotor;
                if (blade_angle > M_PIf) {  //Keep the angle within [-pi, pi]
                    blade_angle = -(2 * M_PIf - position_rotor);
                }

                if (blade_angle < Vxy_angle) {  // # Transformation from blade angle coordinate frame to azimuth coordinate frame
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

            /// <summary>
            /// Method that computes the torque produced by the blade section drag and the corresponding force in the x-y plane
            /// </summary>
            /// <param name="omega">the speed at which the propeller is rotating [rad/s]</param>
            /// <param name="rotor_speed">speed at which the drone is flying</param>
            /// <param name="position_rotor">position of the propeller coordinate frame relative to the body frame. This information
            /// is necessary in order to understand how much of the air velocity is perpendicular to the blade[rad]</param>
            /// <param name="cla_coeffs">list of coefficients used for the computation of the cl given the angle of attack</param>
            /// <param name="cda_coeffs">list of coefficients used for the computation of the cd given the angle of attack</param>
            /// <param name="inflow_data">data regarding the inflow field, namely the uniform induced inflow field, induced inflow
            /// velocityand a lambda function that computes the linear induced field depending on the blade element distance
            /// from the huband angle with respect to the inflow.</param>
            /// <returns>blade section torque and forces in the x-y body plane</returns>
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

            /// <summary>
            /// Method that computes the thrust produced by the blade section and its corresponding moment about the center of
            /// the propeller caused by the thrust force
            /// </summary>
            /// <param name="omega">the speed at which the propeller is rotating [rad/s]</param>
            /// <param name="rotor_speed">speed at which the drone is flying</param>
            /// <param name="position_rotor">position of the propeller coordinate frame relative to the body frame. This information
            /// is necessary in order to understand how much of the air velocity is perpendicular to the blade[rad]</param>
            /// <param name="cla_coeffs">list of coefficients used for the computation of the cl given the angle of attack</param>
            /// <param name="cda_coeffs">list of coefficients used for the computation of the cd given the angle of attack</param>
            /// <param name="inflow_data">data regarding the inflow field, namely the uniform induced inflow field, induced inflow
            /// velocityand a lambda function that computes the linear induced field depending on the blade element distance
            /// from the huband angle with respect to the inflow.</param>
            /// <returns>blade section thrust and moments in the x-y body plane</returns>
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
