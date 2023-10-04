// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file DamagedBlade.hpp
 *
 * @brief Provides the Blade, class for the aerodynamic model identification and computation of forces and moments.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: jialvear@hotmail.com
 *
 */

#ifndef physics_DamagedBlade_hpp
#define physics_DamagedBlade_hpp

#include <Eigen/Dense>
#include "common/Common.hpp"
#include <numeric>
#include "DamagedBladeSection.hpp"

namespace msr {
    namespace airlib {
        /// <summary>
        /// Provides the Blade, class for the aerodynamic model identification and computation of forces and moments.
        /// 
        /// Blade holds all the information related to a single blade and contains a list with all the BladeSection objects that
        /// define the Blade. It is used for calling methods applicable to all the BladeSections which are required for the
        /// computation of the Blade center of gravity, blade area and mass, as well as the moments and forces generated by the
        /// Blade.
        /// 
        /// Additionally, it computes the contribution for the identification of the lift and drag coefficient relative to a
        /// single blade.
        /// </summary>
        class DamagedBlade {
        public:
            /// <summary>
            /// Function that computes a blade area and the location of its cg
            /// </summary>
            /// <param name="chords">the base and tip chords of all the trapezoids</param>
            /// <param name="hs">the height of all the trapezoids</param>
            /// <param name="start_twist">the twist angle at the root</param>
            /// <param name="final_twist">the twist angle at the tip</param>
            /// <param name="radius_hub">radius of the middle propeller hub</param>
            /// <param name="rotation_direction">direction of rotation of the propeller (CW or CCW)</param>
            /// <param name="initial_angle">initial angle of the blade with respect to the propeller at the start of the computations</param>
            /// <param name="broken_percentage">percentage of the blade that is broken from the tip</param>
            DamagedBlade(const vector<real_T>& chords, const vector<real_T>& hs, const real_T& start_twist, const real_T& final_twist, const real_T& radius_hub,
                const int& rotation_direction, const real_T& initial_angle=0, const real_T& broken_percentage = 0.0)
            {
                healthy_chords_ = chords;
                healthy_hs_ = hs;
                start_twist_ = start_twist;
                healthy_final_twist_ = final_twist;
                radius_hub_ = radius_hub;
                rotation_direction_ = rotation_direction;
                initial_angle_ = initial_angle;
                damaged_ = broken_percentage > 0.0001;
                broken_percentage_ = broken_percentage;

                // Compute the length of the healthy blade
                for (real_T& n : healthy_hs_)
                    healthy_h_ += n;

                if (damaged_) {
                    // Compute which length of the blade is not broken
                    damaged_h_ = healthy_h_ * (1 - broken_percentage_ / 100);

                    // Compute the number of hs trapezoids that are completely healthy
                    vector<real_T> accumulated_healthy_h;
                    cumulativeSum(healthy_hs_, accumulated_healthy_h);
                    int n_complete_trapezoids = 0;
                    for (real_T& n : accumulated_healthy_h) {
                        if (n < damaged_h_) {
                            n_complete_trapezoids++;
                        }
                    }

                    // Obtain the length of completely healthy trapezoids
                    vector<real_T> survived_h(healthy_hs_.begin(), healthy_hs_.begin() + n_complete_trapezoids);
                    hs_ = survived_h;

                    real_T length_complete_trapezoids = 0.0;
                    for (real_T& n : survived_h) {
                        length_complete_trapezoids += n;
                    }

                    // Add length of broken trapezoid to hs list
                    hs_.push_back(damaged_h_ - length_complete_trapezoids);

                    // Obtain the chords of healthy trapezoids
                    vector<real_T> new_chords(healthy_chords_.begin(), healthy_chords_.begin() + n_complete_trapezoids + 1);

                    // Obtain the last chord of the broken trapezoid
                    real_T c2 = new_chords.back() - (new_chords.back() - healthy_chords_[n_complete_trapezoids + 1]) / healthy_hs_[n_complete_trapezoids] * hs_.back();

                    // Create the chords vectors with the intact chords and the sliced new one
                    chords_ = new_chords;
                    chords_.push_back(c2);

                    // Obtain the twist at the end of the damaged blade
                    final_twist_ = (healthy_final_twist_ - start_twist_) / healthy_h_ * damaged_h_ + start_twist_;
                }
                else {
                    hs_ = healthy_hs_;
                    chords_ = healthy_chords_;
                    final_twist_ = healthy_final_twist_;
                }


            }
            //%% Structures for data return
            struct retVals_trapezoid_params {        // Declare a local structure 
                real_T area, y_bar;
            };

            struct retVals_average_chords {        // Declare a local structure 
                vector<real_T> average_chords, segment_chords;
            };

            struct retVals_thrust_moment_blade {        // Declare a local structure 
                real_T T_remaining, T_damaged;
                Vector3r M_remaining, M_damaged;
            };

            struct retVals_torque_force_blade {        // Declare a local structure 
                real_T Q_remaining, Q_damaged;
                Vector3r F_remaining, F_damaged;
            };

            //%% Class functions
            /// <summary>
            /// Function that computes the location of the center of gravity of the blade and its area
            /// </summary>
            void computeBladeParams() {
                vector<real_T> areas;
                real_T mass_moment = 0.0;
                real_T h0 = 0.0;
                for (int i = 0; i < hs_.size(); i++) {
                    real_T bc = chords_[i];
                    real_T tc = chords_[i + 1];
                    real_T h = hs_[i];

                    retVals_trapezoid_params trapezoid_params = trapezoidParams(bc, tc, h);
                    real_T area = trapezoid_params.area;
                    real_T y_bar = trapezoid_params.y_bar;
                    mass_moment += area * (h0 + y_bar);
                    areas.push_back(area);
                    h0 += hs_[i];
                }
                blade_area_ = std::accumulate(areas.begin(), areas.end(), 0.0f);
                y_cg_ = 0;
                if (blade_area_ != 0) {
                    y_cg_ = mass_moment / blade_area_;
                }
                
            }

            /// <summary>
            /// Function that computes the mass of the blade
            /// </summary>
            /// <param name="healthy_mass"mass of the blade when there is no damage></param>
            /// <returns>The mass of the blade, taking into account any damage. If the blade is damaged, the mass is calculated
            /// using the healthy massand the blade's relative area. If not damaged, the mass remains the same as the healthy
            /// mass.</returns>
            real_T computeBladeMass(const real_T& healthy_mass) {
                if (damaged_) {
                    computeHealthyBladeArea();
                    blade_mass_ = healthy_mass * (blade_area_ / healthy_blade_area_);
                }
                else {
                    healthy_blade_area_ = blade_area_;
                    blade_mass_ = healthy_mass;
                }

                return blade_mass_;
            }

            /// <summary>
            ///  Function that computes the area of the blade when there is no damage
            /// </summary>
            void computeHealthyBladeArea() {
                for (int i = 0; i < healthy_hs_.size(); i++) {
                    real_T bc = healthy_chords_[i];
                    real_T tc = healthy_chords_[i + 1];
                    real_T h = healthy_hs_[i];
                    retVals_trapezoid_params trapezoid_params = trapezoidParams(bc, tc, h);
                    real_T area = trapezoid_params.area;
                    healthy_blade_area_ += area;
                }
            }

            /// <summary>
            /// Method that creates the BladeSection objects that shape a blade.
            /// </summary>
            /// <param name="number_sections">the number of blade sections in which the blade should be split</param>
            void createBladeSections(const int& number_sections) {
                real_T dr = healthy_h_ / number_sections;
                retVals_average_chords average_chords_params = compute_average_chords(healthy_chords_, healthy_hs_, number_sections, dr);
                vector<real_T> average_chords = average_chords_params.average_chords;
                vector<real_T> segment_chords = average_chords_params.segment_chords;
                vector<real_T> twists_edges = linspace(start_twist_, healthy_final_twist_, number_sections + 1);
                vector<real_T> twist_sections;
                for (int i = 0; i < number_sections; i++) {
                    twist_sections.push_back((twists_edges[i] + twists_edges[i + 1]) / 2);
                }

                for (int i = 0; i < number_sections; i++) {
                    DamagedBladeSection blade_section = DamagedBladeSection(i, average_chords[i], dr, twist_sections[i], segment_chords[i],
                        segment_chords[i + 1], radius_hub_, rotation_direction_);
                    if (damaged_ && dr * i >= damaged_h_) {
                        damaged_blade_sections_.push_back(blade_section);
                    }
                    else {
                        blade_sections_.push_back(blade_section);
                    }
                }
            }

            /// <summary>
            /// Function that computes the thrust and the moment generated by the thrust force around the propeller hub. This
            /// is done for the complete bladeand the damaged component.
            /// </summary>
            /// <param name="number_sections">number of sections to split the blade</param>
            /// <param name="angle_propeller_rotation">the angle the propeller has rotated since the start</param>
            /// <param name="omega">the rotation rate of the propeller [rad/s]</param>
            /// <param name="propeller_speed">the 3D velocity vector of the propeller system</param>
            /// <param name="cla_coeffs">the coefficients used for the computation of the lift coefficient as a function of alpha</param>
            /// <param name="cda_coeffs">list of coefficients used for the computation of the cd given the angle of attack</param>
            /// <param name="inflow_data">data regarding the inflow field, namely the uniform induced inflow field, induced inflow
            /// velocityand a lambda function that computes the linear induced field depending on the blade element distance
            /// from the huband angle with respect to the inflow.</param>
            /// <returns></returns>
            retVals_thrust_moment_blade compute_thrust_moment(const real_T& number_sections, const real_T& angle_propeller_rotation, const real_T& omega, const Vector3r& propeller_speed,
                const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {
                real_T blade_angle = fmod(angle_propeller_rotation + initial_angle_, 2.0f * M_PIf);
                if (blade_sections_.empty() && damaged_blade_sections_.empty()) {
                    createBladeSections(number_sections);
                }

                real_T T_remaining = 0;
                Vector3r M_remaining = Vector3r::Zero();

                // Iterate for the damaged / lost blade sections
                real_T T_damaged = 0;
                Vector3r M_damaged = Vector3r{ 0, 0, 0 };
                for (DamagedBladeSection& DBS : damaged_blade_sections_) {
                    auto TM = DBS.computeThrustMoment(omega, propeller_speed, blade_angle, cla_coeffs, cda_coeffs, inflow_data);
                    real_T dT = TM.dT;
                    Vector3r dM = TM.dM;
                    T_damaged += dT;
                    M_damaged += dM;
                }

                return retVals_thrust_moment_blade{ T_remaining , T_damaged , M_remaining , M_damaged };
            }

            /// <summary>
            /// Function that computes the torque and force in the body x-y plane generated by the healthy part of the blade and
            /// the damaged sections.
            /// </summary>
            /// <param name="number_sections">number of sections to split the blade</param>
            /// <param name="angle_propeller_rotation">the angle the propeller has rotated since the start</param>
            /// <param name="omega">the rotation rate of the propeller [rad/s]</param>
            /// <param name="propeller_speed">the 3D velocity vector of the propeller system</param>
            /// <param name="cla_coeffs">the coefficients used for the computation of the lift coefficient as a function of alpha</param>
            /// <param name="cda_coeffs">list of coefficients used for the computation of the cd given the angle of attack</param>
            /// <param name="inflow_data">data regarding the inflow field, namely the uniform induced inflow field, induced inflow
            /// velocityand a lambda function that computes the linear induced field depending on the blade element distance
            /// from the huband angle with respect to the inflow.</param>
            /// <returns>the torque and the forces in the body x-y plane</returns>
            retVals_torque_force_blade compute_torque_force(const int& number_sections, const real_T& angle_propeller_rotation, const real_T& omega, const Vector3r& propeller_speed,
                const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {
                real_T blade_angle = fmod(angle_propeller_rotation + initial_angle_, 2.0f * M_PIf);
                if (blade_sections_.empty() && damaged_blade_sections_.empty()) {
                    createBladeSections(number_sections);
                }

                real_T Q_remaining = 0;
                Vector3r F_remaining = Vector3r{ 0, 0, 0 };

                // Iterate for the damaged/lost blade sections
                real_T Q_damaged = 0;
                Vector3r F_damaged = Vector3r{ 0, 0, 0 };
                for (DamagedBladeSection& DBS : damaged_blade_sections_) {
                    auto TM = DBS.computeTorqueForce(omega, propeller_speed, blade_angle, cla_coeffs, cda_coeffs, inflow_data);
                    real_T Qd = TM.dQ;
                    Vector3r Fd = TM.dF;
                    Q_damaged += Qd;
                    F_damaged += Fd;
                }
                return retVals_torque_force_blade{ Q_remaining , Q_damaged , F_remaining, F_damaged };
            }


            //%% Helping functions
            /// <summary>
            /// Computes an accumulative vector of the input. Each element of input with index m equals the sum of all elements of input till index m
            /// </summary>
            /// <param name="input">input vector</param>
            /// <param name="result">output vector</param>
            void cumulativeSum(const vector<real_T>& input, vector<real_T>& result) {
                result.push_back(input[0]);
                for (int i = 1; i < input.size(); i++) {
                    result.push_back(result[i - 1] + input[i]);
                }
            }

            /// <summary>
            /// Computed the area and the location of the center of gravity of a blade
            /// </summary>
            /// <param name="bc">base chord, closest to the hub</param>
            /// <param name="tc">tip chord</param>
            /// <param name="h">length of trapezoid</param>
            /// <returns></returns>
            retVals_trapezoid_params trapezoidParams(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T area = computeTrapezoidArea(bc, tc, h);
                real_T y_bar = computeTrapezoidCg(bc, tc, h);
                return retVals_trapezoid_params{ area , y_bar };
            }

            /// <summary>
            /// Compute the area of a trapezoid
            /// </summary>
            /// <param name="bc">the base chord</param>
            /// <param name="tc">the tip chord</param>
            /// <param name="h">the height of the trapezoid</param>
            /// <returns>the area of the trapezoid</returns>
            real_T computeTrapezoidArea(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T area = (tc + bc) * h / 2.0f;
                return area;
            }

            /// <summary>
            /// Compute the location of the centre of gravity of a trapezoid
            /// </summary>
            /// <param name="bc">the base chord</param>
            /// <param name="tc">the tip chord</param>
            /// <param name="h">the trapezoid length</param>
            /// <returns>the location along the h direction where the center of gravity is locate</returns>
            real_T computeTrapezoidCg(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T y_bar = (2.0 * tc + bc) / (tc + bc) * h / 3.0f;
                return y_bar;
            }

            /// <summary>
            /// Function that computes the chord of a trapezoid at a specific location along the h direction
            /// </summary>
            /// <param name="bc">chord at the base</param>
            /// <param name="tc">chord at the tip</param>
            /// <param name="h">length of the trapezoid</param>
            /// <param name="h0">position along the blade at which the trapezoid base is located</param>
            /// <param name="pos">position along the blade at which we would like to compute the chord</param>
            /// <returns></returns>
            real_T computeChordTrapezoid(const real_T& bc, const real_T& tc, const real_T& h, const real_T& h0, const real_T& pos) {
                real_T chord = bc - (bc - tc) / h * (pos - h0);
                return chord;
            }

            /// <summary>
            /// Function that retrieves the information related to a blade section
            /// </summary>
            /// <param name="counter">blade section index</param>
            /// <param name="h_origin">the location along the blade span of the root chord of the current blade section</param>
            /// <param name="current_h">current blade section span</param>
            /// <param name="current_bc">current blade section root chord</param>
            /// <param name="current_tc">current blade section tip chord</param>
            void updateChordsH(int& counter, real_T& h_origin, real_T& current_h, real_T& current_bc, real_T& current_tc) {
                if (counter >= 0) {
                    h_origin += healthy_hs_[counter];
                }

                counter += 1;
                current_h = healthy_hs_[counter];
                current_bc = healthy_chords_[counter];
                current_tc = healthy_chords_[counter + 1];
            }

            /// <summary>
            /// Function that computes the average chords of all the blade section
            /// </summary>
            /// <param name="chords">list with all the trapezoid sections' chords</param>
            /// <param name="hs">list with all the trapezoid sections' lengths</param>
            /// <param name="n_segments">number of blade sections</param>
            /// <param name="dr">blade section span</param>
            /// <returns>average chords of all the blade sections and the chords at the root and tip of the blade sections</returns>
            retVals_average_chords compute_average_chords(const vector<real_T>& chords, const vector<real_T>& hs, const int& n_segments, const real_T& dr) {
                int segment = 0;
                int trapezoid_count;
                real_T h0, h, bc, tc;
                trapezoid_count = -1;
                h0 = 0.0;
                updateChordsH(trapezoid_count, h0, h, bc, tc);
                vector<real_T> average_chords;
                vector<real_T> segment_chords;
                segment_chords.push_back(bc);
                for (int i = 0; i < n_segments; i++) {
                    real_T area_1 = 0;
                    real_T pos_c1 = segment * dr;  // location of the blade section's root chord along the blade
                    real_T pos_c2 = (segment + 1) * dr;  // location of the blade section's tip chord along the blade
                    real_T c1 = segment_chords[segment];  // chord of the blade section's root
                    if (pos_c2 > (h0 + hs[trapezoid_count])) {  // In the case that the blade section's tip is in the next trapezoid
                        // Compute the area of the part of the blade section in the first trapezoid
                        area_1 = computeTrapezoidArea(c1, tc, h0 + h - pos_c1);

                        // Retrieve the information from the second trapezoid
                        updateChordsH(trapezoid_count, h0, h, bc, tc);
                    }
                    // Compute the location of the blade section's tip chord given its location and the information of the trapezoid
                    // where it is located
                    real_T c2 = computeChordTrapezoid(bc, tc, h, h0, pos_c2);
                    real_T area;
                    if (area_1 == 0) {  // when the blade section is in a single trapezoid
                        area = computeTrapezoidArea(c1, c2, dr);
                    }
                    else {  // when it is found between two trapezoids
                        area = area_1 + computeTrapezoidArea(bc, c2, pos_c2 - h0);
                    }
                    real_T average_chord = area / dr;  // compute the average chord of the blade section
                    average_chords.push_back(average_chord);
                    segment_chords.push_back(c2);
                    segment++;
                }
                return retVals_average_chords{ average_chords, segment_chords };
            }

            /// <summary>
            /// Equivalent to np.linspace function
            /// </summary>
            /// <param name="a">start of range</param>
            /// <param name="b">end of range</param>
            /// <param name="N">number of section in the range determined by [a,b]</param>
            /// <returns>array of range sections</returns>
            std::vector<real_T> linspace(const real_T& a, const real_T& b, const int& N) {
                real_T h = (b - a) / static_cast<real_T>(N - 1);
                std::vector<real_T> xs(N);
                typename std::vector<real_T>::iterator x;
                real_T val;
                for (x = xs.begin(), val = a; x != xs.end(); ++x, val += h)
                    *x = val;
                return xs;
            }

            // Setter and getters
            real_T getInitialAngle() {
                return initial_angle_;
            }

            real_T getYCg() {
                return y_cg_;
            }

            real_T getBladeMass() {
                return blade_mass_;
            }

        private:
            // Parameters that will be initialised in __init__
            vector<real_T> healthy_chords_;
            vector<real_T> chords_;
            vector<real_T> healthy_hs_;
            vector<real_T> hs_;
            real_T start_twist_;
            real_T healthy_final_twist_;
            real_T final_twist_;
            real_T radius_hub_;
            int rotation_direction_;
            real_T initial_angle_;
            bool damaged_;
            real_T broken_percentage_;

            // Other parameters initialised outside of __init__
            real_T blade_area_;
            real_T healthy_blade_area_ = 0;
            real_T y_cg_;
            real_T blade_mass_;
            vector<DamagedBladeSection> blade_sections_;
            vector<DamagedBladeSection> damaged_blade_sections_;
            real_T healthy_h_ = 0;
            real_T damaged_h_ = -1;


            // Constant expressions
            static constexpr real_T rad2deg_factor_ = 180.0f / M_PIf;
            static constexpr real_T deg2rad_factor_ = M_PIf / 180.0f;
        };

    }
}



#endif // !physics_DamagedBlade_hpp
