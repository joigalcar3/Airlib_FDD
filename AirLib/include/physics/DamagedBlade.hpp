// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file DamagedBlade.hpp
 *
 * @brief Provides the Blade, class for the aerodynamic model identification and computation of forces and moments.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: j.i.dealvearcardenas@student.tudelft.nl
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
        class DamagedBlade {
        public:
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
            // Function that computes the location of the center of gravity of the blade and its area
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

            // Function that computes the mass of the blade
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

            // Function that computes the area of the blade when there is no damage
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

            // Method that creates the BladeSection objects that shape a blade.
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

            // Function that computes the thrust and the moment generated by the thrust force around the propeller hub. This
            // is done for the complete bladeand the damaged component.
            retVals_thrust_moment_blade compute_thrust_moment(const real_T& number_sections, const real_T& angle_propeller_rotation, const real_T& omega, const Vector3r& propeller_speed,
                const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {
                real_T blade_angle = fmod(angle_propeller_rotation + initial_angle_, 2.0f * M_PIf);
                if (blade_sections_.empty() && damaged_blade_sections_.empty()) {
                    createBladeSections(number_sections);
                }

                real_T T_remaining = 0;
                Vector3r M_remaining = Vector3r::Zero();
                //for (DamagedBladeSection& BS : blade_sections_) {
                //    auto TM = BS.computeThrustMoment(omega, propeller_speed, blade_angle, cla_coeffs, cda_coeffs, inflow_data);
                //    real_T Tr = TM.dT;
                //    Vector3r Mr = TM.dM;
                //    T_remaining += Tr;
                //    M_remaining += Mr;
                //}       

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

            // Function that computes the torque and force in the body x-y plane generated by the healthy part of the blade and
            // the damaged sections.
            retVals_torque_force_blade compute_torque_force(const int& number_sections, const real_T& angle_propeller_rotation, const real_T& omega, const Vector3r& propeller_speed,
                const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs, std::function<real_T(real_T, real_T)> inflow_data) {
                real_T blade_angle = fmod(angle_propeller_rotation + initial_angle_, 2.0f * M_PIf);
                if (blade_sections_.empty() && damaged_blade_sections_.empty()) {
                    createBladeSections(number_sections);
                }

                real_T Q_remaining = 0;
                Vector3r F_remaining = Vector3r{ 0, 0, 0 };
                //for (DamagedBladeSection& BS : blade_sections_) {
                //    auto TM = BS.computeTorqueForce(omega, propeller_speed, blade_angle, cla_coeffs, cda_coeffs, inflow_data);
                //    real_T Qr = TM.dQ;
                //    Vector3r Fr = TM.dF;
                //    Q_remaining += Qr;
                //    F_remaining += Fr;
                //}

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
            // Computes an accumulative vector of the input. Each element of input with index m equals the sum of all elements of input till index m
            void cumulativeSum(const vector<real_T>& input, vector<real_T>& result) {
                result.push_back(input[0]);
                for (int i = 1; i < input.size(); i++) {
                    result.push_back(result[i - 1] + input[i]);
                }
            }

            // Computed the area and the location of the center of gravity of a blade
            retVals_trapezoid_params trapezoidParams(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T area = computeTrapezoidArea(bc, tc, h);
                real_T y_bar = computeTrapezoidCg(bc, tc, h);
                return retVals_trapezoid_params{ area , y_bar };
            }

            // Compute the area of a trapezoid
            real_T computeTrapezoidArea(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T area = (tc + bc) * h / 2.0f;
                return area;
            }

            // Compute the location of the centre of gravity of a trapezoid
            real_T computeTrapezoidCg(const real_T& bc, const real_T& tc, const real_T& h) {
                real_T y_bar = (2.0 * tc + bc) / (tc + bc) * h / 3.0f;
                return y_bar;
            }

            // Function that computes the chord of a trapezoid at a specific location along the h direction
            real_T computeChordTrapezoid(const real_T& bc, const real_T& tc, const real_T& h, const real_T& h0, const real_T& pos) {
                real_T chord = bc - (bc - tc) / h * (pos - h0);
                return chord;
            }

            // Function that retrieves the information related to a blade section
            void updateChordsH(int& counter, real_T& h_origin, real_T& current_h, real_T& current_bc, real_T& current_tc) {
                if (counter >= 0) {
                    h_origin += healthy_hs_[counter];
                }

                counter += 1;
                current_h = healthy_hs_[counter];
                current_bc = healthy_chords_[counter];
                current_tc = healthy_chords_[counter + 1];
            }

            // Function that computes the average chords of all the blade section
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
                    real_T pos_c1 = segment * dr;
                    real_T pos_c2 = (segment + 1) * dr;
                    real_T c1 = segment_chords[segment];
                    if (pos_c2 > (h0 + hs[trapezoid_count])) {
                        area_1 = computeTrapezoidArea(c1, tc, h0 + h - pos_c1);
                        updateChordsH(trapezoid_count, h0, h, bc, tc);
                    }
                    real_T c2 = computeChordTrapezoid(bc, tc, h, h0, pos_c2);
                    real_T area;
                    if (area_1 == 0) {
                        area = computeTrapezoidArea(c1, c2, dr);
                    }
                    else {
                        area = area_1 + computeTrapezoidArea(bc, c2, pos_c2 - h0);
                    }
                    real_T average_chord = area / dr;
                    average_chords.push_back(average_chord);
                    segment_chords.push_back(c2);
                    segment++;
                }
                return retVals_average_chords{ average_chords, segment_chords };
            }

            // Equivalent to np.linspace function
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
