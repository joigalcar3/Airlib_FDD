// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

/**
 * @file DamagedPropeller.hpp
 *
 * @brief Provides the Propeller, class for the aerodynamic model identification and computation of forces and moments.
 *
 * @author Jose Ignacio de Alvear Cardenas
 * Contact: j.i.dealvearcardenas@student.tudelft.nl
 *
 */

#ifndef physics_DamagedPropeller_hpp
#define physics_DamagedPropeller_hpp

#include <Eigen/Dense>
#include "common/Common.hpp"
#include "DamagedBlade.hpp"
#include <math.h>  

namespace msr {
    namespace airlib {
        class DamagedPropeller {
        public:
            DamagedPropeller() {}

            DamagedPropeller(const int& propeller_number, const int& number_blades, const vector<real_T>& chords, const vector<real_T>& hs, const real_T& radius_hub, const real_T& healthy_propeller_mass,
                const real_T& percentage_hub_m, const real_T& angle_first_blade, const real_T& start_twist, const real_T& final_twist, const vector<real_T>& broken_percentage,
                const real_T& l = 0.0875, const real_T& b = 0.1150, const int& signr = -1, const real_T& g = 9.80665)
            {
                propeller_number_ = propeller_number;
                number_blades_ = number_blades;
                chords_ = chords;
                hs_ = hs;
                radius_hub_ = radius_hub;
                healthy_propeller_mass_ = healthy_propeller_mass;
                percentage_hub_m_ = percentage_hub_m;
                angle_first_blade_ = deg2rad_factor_ * angle_first_blade;
                start_twist_ = deg2rad_factor_ * start_twist;
                final_twist_ = deg2rad_factor_ * final_twist;
                broken_percentage_ = broken_percentage;
                l_ = l;
                b_ = b;
                signr_ = signr;
                g_ = g;

                healthy_blade_m_ = healthy_propeller_mass_ * (1 - percentage_hub_m_ / 100) / number_blades_;
                SN_ = { signr_, -signr_ , signr_ , -signr_ };
                d_ = { {l_, -b_, 0}, {l_, b_, 0}, {-l_, b_, 0}, {-l_, -b_, 0} };

                // Compute the length of the healthy blade
                for (real_T& n : hs_)
                    length_blade_ += n;
            }

            //%% Structures for data return
            struct retVals_compute_cg_forces_moments {        // Declare a local structure 
                Vector3r M_vector, F_vector;
            }; 

            struct retVals_thrust_moment_propeller {        // Declare a local structure 
                real_T T_remaining, T_damaged;
                Vector3r M_remaining, M_damaged;
            };

            struct retVals_torque_force_propeller {        // Declare a local structure 
                real_T Q_remaining, Q_damaged;
                Vector3r F_remaining, F_damaged;
            };

            struct retVals_mass_aero_FM_propeller {
                Vector3r delta_F, delta_M, mass_F, mass_M, aero_F, aero_M;
            };

            struct retVals_aero_FM_propeller {
                Vector3r M_vector, F_vector;
            };

            // Method to reset the propeller to the default values and change its broken degree
            void resetPropeller(const vector<real_T>& broken_percentage, const real_T& rotation_angle) {
                broken_percentage_ = broken_percentage;
                blades_.clear();
                cg_x_ = 0.0;
                cg_y_ = 0.0;
                cg_r_ = -1.0;
                propeller_mass_ = -1.0;
                propeller_velocity_ = Vector3r::Zero();
                omega_ = -1.0;
                rotation_angle_ = rotation_angle;
            }

            // Function that creates each of the blades objects that are part of a propeller
            void createBlades()
            {
                real_T current_angle = angle_first_blade_;
                real_T angle_step = 2 * M_PIf / number_blades_;

                for (int i = 0; i < number_blades_; i++) {
                    real_T bp = broken_percentage_[i];
                    DamagedBlade blade = DamagedBlade(chords_, hs_, start_twist_, final_twist_, radius_hub_, SN_[propeller_number_], 
                        current_angle, bp);
                    blades_.push_back(blade);
                    current_angle += angle_step;
                }    
            }

            //Function that computes the location of the cg, the area and the mass of each of the blades, as well as the mass
            // of the complete propeller by summing the hub mass with the computed blades' masses.
            void computeBladesParams() {
                if (blades_.empty()) {
                    createBlades();
                }
                real_T blades_mass = 0;
                for (int i = 0; i < number_blades_; i++) {
                    DamagedBlade& blade = blades_[i];
                    blade.computeBladeParams();
                    real_T blade_mass = blade.computeBladeMass(healthy_blade_m_);
                    blades_mass += blade_mass;
                }
                propeller_mass_ = blades_mass + healthy_propeller_mass_ * percentage_hub_m_ / 100.0f;
            }

            // Function that computes the location of the cg of the complete propeller
            void computeCgLocation() {
                if (propeller_mass_ < 0) {
                    computeBladesParams();
                }

                //Computation of overall cg location
                for (int i = 0; i < number_blades_; i++) {
                    DamagedBlade& blade = blades_[i];
                    real_T y_coord = (blade.getYCg() + radius_hub_) * sin(blade.getInitialAngle());
                    real_T x_coord = (blade.getYCg() + radius_hub_) * cos(blade.getInitialAngle());
                    cg_x_ += x_coord * blade.getBladeMass() / propeller_mass_;
                    cg_y_ += y_coord * blade.getBladeMass() / propeller_mass_;
                }
                cg_r_ = sqrt(powf(cg_x_, 2) + powf(cg_y_, 2));
            }

            // Method that computes the forces caused by the current location of the center of gravity
            retVals_compute_cg_forces_moments computeCgForcesMoments(const real_T& omega, const Quaternionr& attitude) {
                if (cg_r_ < 0) {
                    computeCgLocation();
                }
                
                // Centrifugal force
                real_T F_centrifugal = propeller_mass_ * powf(omega, 2) * cg_r_;
                real_T angle_cg = atan2(cg_y_, cg_x_) + rotation_angle_;
                real_T Fx_centrifugal = F_centrifugal * cos(angle_cg);
                real_T Fy_centrifugal = F_centrifugal * sin(angle_cg);
                Vector3r F_centrifugal_vector = Vector3r(Fx_centrifugal, Fy_centrifugal, 0 );

                // Moments caused by the shift in cg
                real_T M = propeller_mass_ * g_ * cg_r_;

                // Compute the vector of the gravity force passing through the current center of gravity in the body frame
                Vector3r Fg_I = Vector3r(0, 0, g_ * propeller_mass_);
                Vector3r Fg_b = VectorMath::transformToBodyFrame(Fg_I, attitude);

                // Compute the vector of the gravity force of the lost blade piece passing through the current center of gravity
                // in the body frame.
                Vector3r Fg_broken_segment_I = Vector3r(0, 0, -g_ * (healthy_propeller_mass_ - propeller_mass_));
                Vector3r Fg_broken_segment_b = VectorMath::transformToBodyFrame(Fg_broken_segment_I, attitude);

                // Compute the moment and force vectors that have to be added to the current forces and moments
                Vector3r r_vector = Vector3r(cos(angle_cg) * cg_r_, sin(angle_cg) * cg_r_, 0);

                Vector3r M_vector = r_vector.cross(Fg_b);
                Vector3r F_vector = F_centrifugal_vector + Fg_broken_segment_b;
                return retVals_compute_cg_forces_moments{ M_vector , F_vector };
            }

            // Method that computes the uniform and linear induced inflow
            std::function<real_T(real_T, real_T)> computeInducedInflow(const real_T& T, const real_T& rho, const real_T& omega) {
                real_T R = length_blade_ + radius_hub_;
                real_T A = M_PIf * R * R;
                real_T V_inf = propeller_velocity_.norm();
                real_T tpp_V_angle = 0.0f;
                if (V_inf != 0) {
                    tpp_V_angle = asin(std::max(std::min(-propeller_velocity_.z() / V_inf, 1.0f), -1.0f));
                }

                // Glauert equation
                std::function<real_T(const real_T&)> min_func = [=](const real_T& x) {
                    return abs(T - 2.0f * rho * A * x * sqrt(powf(V_inf * cos(tpp_V_angle), 2) + powf(V_inf * sin(tpp_V_angle) + x, 2))); 
                };

                // Glauert equation without the absolute value
                std::function<real_T(const real_T&)> min_func_2 = [=](const real_T& x) {
                    return T - 2.0f * rho * A * x * sqrt(powf(V_inf * cos(tpp_V_angle), 2) + powf(V_inf * sin(tpp_V_angle) + x, 2));
                };

                // Derivative of the Glauert equation
                std::function<real_T(const real_T&)> der_func = [=](const real_T& x) {
                    return -2.0f * rho * A * (sqrt(powf(V_inf * cos(tpp_V_angle), 2) + powf(V_inf * sin(tpp_V_angle) + x, 2)) +
                        (V_inf * sin(tpp_V_angle) + x) / (sqrt(powf(V_inf * cos(tpp_V_angle), 2) + powf(V_inf * sin(tpp_V_angle) + x, 2)))) *
                        min_func_2(x) / min_func(x);
                };

                real_T x0 = 4.5f;

                real_T v0 = optimizeGradientDescend(der_func, x0, min_func);

                real_T lambda_0 = v0 / (omega * R);

                // Create function for the computation of the linear induced inflow
                std::function<real_T(real_T, real_T)> induced_velocity_func = [=](real_T r, real_T psi) {
                    return v0;
                };

                if (V_inf != 0) {
                    // Compute wake skew angle
                    real_T mu_x = V_inf * cos(tpp_V_angle) / (omega * R);
                    real_T mu_z = V_inf * sin(tpp_V_angle) / (omega * R);
                    real_T Chi = atan(mu_x / (mu_z + lambda_0));

                    // Compute kxand ky weighting factors
                    real_T kx = 4.0f / 3.0f * ((1.0f - cos(Chi) - 1.8f * powf(mu_x, 2)) / sin(Chi));
                    real_T ky = -2.0f * mu_x;

                    // Create function for the computation of the linear induced inflow
                    induced_velocity_func = [=](real_T r, real_T psi) {
                        return lambda_0 * (1.0f + kx * r * cos(psi) + ky * r * sin(psi)) * omega * R;
                    };
                }

                return induced_velocity_func;
            }

            // Function that computes the lift of a propeller using the identified polynomials from the aerodynamic model.
            real_T computeLiftTorqueMatlab(const Vector3r& body_velocity, const Vector3r& pqr, const real_T& omega, const real_T& rho = 1.225f) {
                Vector3r d_local = Vector3r{ d_[propeller_number_][0], d_[propeller_number_][1], d_[propeller_number_][2] };
                propeller_velocity_ = pqr.cross(d_local) + body_velocity;
                real_T u = propeller_velocity_.x();
                real_T v = propeller_velocity_.y();
                real_T w = propeller_velocity_.z();
                real_T R = length_blade_ + radius_hub_;

                real_T va = sqrt(powf(u, 2) + powf(v, 2) + powf(w, 2));
                real_T vv;
                real_T alpha;
                if ((omega * R) == 0){
                    vv = 0.0f;
                }
                else {
                    vv = std::min(va / (omega * R), 0.6f);
                }
                if (sqrt(powf(u,2) + powf(v,2)) == 0) {
                    alpha = 0.0f;
                }
                else {
                    alpha = atan(w / sqrt(powf(u, 2) + powf(v, 2))) * rad2deg_factor_;
                }

                std::vector<real_T> P52_comp = computeP52(alpha, vv);
                std::vector<real_T> k_Ct0{ 0.0152457017219075f, -3.19835880466424e-05f, -0.0474659629880834f, -5.48089604291955e-08f, -0.000164550969624146f, 0.650877249185920f, 1.10477778832442e-08f, -9.76752919452344e-06f, 0.00859691522825337f, -2.20418122442645f, 3.27434126987218e-11f, -5.69117054658112e-08f, 2.32561854294217e-05f, -0.0116550184566165f, 3.04959484433102f, -6.00795185558617e-13f, 1.81690349314076e-10f, -4.63671043348055e-08f, -1.52454780569063e-05f, 0.00607313609112646f, -1.51563942225535f };
                real_T Ct = std::inner_product(P52_comp.begin(), P52_comp.end(), k_Ct0.begin(), 0.0f);

                real_T mu = sqrt(powf(u, 2) + powf(v, 2)) / (omega * R);
                real_T lc = w / (omega * R);
                real_T dCt;

                if (u == 0 && v == 0) {
                    dCt = 0.0f;
                }
                else {
                    real_T beta = computeBeta(u, v) * rad2deg_factor_;
                    real_T arm_angle = atan(l_ / b_) * rad2deg_factor_;
                    real_T psi_h = compute_psi(beta, arm_angle, propeller_number_);
                    std::vector<real_T> FN_comp = compute_Fn(psi_h, 5.0f, 1.0f, mu, lc);
                    std::vector<real_T> k_model_2{ 0.00274750362242118f, 0.0587325418628517f, 0.0291979006795257f, 0.155176381977433f, -0.848919654295447f, -2.85285652127970f, -16.6872606424138f, -25.3582092758054f, -8.21139486023900f, 0.000662666074942486f, -0.0137515544490184f, 0.0258468383560923f, 0.129354278831284f, 0.739022953068088f, -0.751589665006720f, 2.30225029828906f, 3.19387485842342f, -2.01400124775404f, -0.0327090226346993f, 0.00550048663001955f, 0.704747484311295f, 0.384042859177342f, 0.409107040799852f, -2.91693142809590f, -5.72731924259749f, -3.84424311448819f, 0.957068915766478f, 0.00798767042006989f, -0.0658174319226979f, -0.515362725845307f, 0.154017181898898f, 1.07229345471127f, 5.60834749404815f, 3.12341580631406f, 13.2751387862931f, 3.38340384818304f, -0.00871325200163225f, 0.0139319909808224f, 0.135829051260073f, 0.0724018634704221f, 0.462231305873754f, 1.07728548843851f, -2.92439099099261f, 2.07387265629944f, -1.76236683822441f, 0.00277901355913424f, 5.93712461960435e-05f, -0.0737682036851551f, 0.408392701436168f, 0.181780336855863f, -0.0914796558508702f, -5.33048488631146f, -11.6294693255163f, -4.72950404100762f, -0.00594871416216384f, -0.0162850806730608f, 0.173368295316786f, 0.186292675296392f, 0.225644067201894f, -0.688845939593434f, -6.49432628543192f, -7.80900137821226f, 0.415239218701371f, -0.00544216811616573f, 0.00518487316578840f, 0.0476580090813803f, -0.200801241660794f, -0.476117215479456f, -0.407991135460875f, -1.81735072025647f, 1.50472930028764f, 4.35662490484023f, -0.00159368739623987f, 0.000467723919419556f, 0.0129022985413385f, -0.142747208717601f, -0.286423056758624f, -0.233246678589007f, 5.27930446169201f, 6.06363387971617f, 3.14128857337644f, 0.00453268191002699f, -0.00474962613583822f, -0.180460224377998f, -0.0116017180130748f, 0.0192198318847662f, 1.17708508701190f, 0.0640467785184096f, 3.10723451211166f, 0.482465692101886f};
                    dCt = std::inner_product(FN_comp.begin(), FN_comp.end(), k_model_2.begin(), 0.0f);
                    if (dCt > 0.007f) { dCt = 0.007f; }
                    else if (dCt < -0.007f) { dCt = -0.007f; }
                    real_T vh = sqrt(u * u + v * v);
                    real_T one_exp = 1.0f / (1.0f + exp(-6.0f * (vh - 1.0f)));
                    dCt = one_exp * dCt;
                }

                real_T dynhead = rho * powf(omega, 2) * powf(R, 2);
                real_T area = M_PIf * powf(R, 2);
                real_T T = (Ct + dCt) * dynhead * area;
                return T;
            }

            // Method to compute the thrust and the corresponding moment around the propeller hub caused by this forces. This
            // is done for the remainingand damaged(flown away) blade sections.
            retVals_thrust_moment_propeller computeThrustMoment(const real_T& number_sections, const real_T& omega, const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs,
                const Vector3r& body_velocity, const Vector3r& pqr, std::function<real_T(real_T, real_T)> inflow_data, const real_T& rho = 1.225f) {
                if (blades_.empty()) {
                    createBlades();
                }
                // Compute the thrust of the propeller
                real_T T_remaining = 0;
                real_T T_damaged = 0;
                Vector3r M_remaining = Vector3r::Zero();
                Vector3r M_damaged = Vector3r::Zero();

                for (DamagedBlade& B : blades_) {
                    auto TM = B.compute_thrust_moment(number_sections, rotation_angle_, omega, propeller_velocity_, cla_coeffs, cda_coeffs, inflow_data);
                    real_T T_r = TM.T_remaining;
                    real_T T_d = TM.T_damaged;
                    Vector3r M_r = TM.M_remaining;
                    Vector3r M_d = TM.M_damaged;
                    T_remaining += T_r;
                    T_damaged += T_d;
                    M_remaining += M_r;
                    M_damaged += M_d;
                }

                return retVals_thrust_moment_propeller{ T_remaining, T_damaged, M_remaining, M_damaged };
            }

            // Method to compute the torque generated by the blade, as well as the corresponding force in the x-y plane. This
            // is done for the complete propellerand the damaged(flown away) sections.
            retVals_torque_force_propeller computeTorqueForce(const int& number_sections, const real_T& omega, const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs,
                const Vector3r& body_velocity, const Vector3r& pqr, std::function<real_T(real_T, real_T)> inflow_data, const real_T& rho = 1.225f) {
                if (blades_.empty()) {
                    createBlades();
                }
                // Compute the thrust of the propeller
                real_T Q_remaining = 0.0f;
                real_T Q_damaged = 0.0f;
                Vector3r F_remaining = Vector3r::Zero();
                Vector3r F_damaged = Vector3r::Zero();

                for (DamagedBlade& B : blades_) {
                    auto TM = B.compute_torque_force(number_sections, rotation_angle_, omega, propeller_velocity_, cla_coeffs, cda_coeffs, inflow_data);
                    real_T Q_r = TM.Q_remaining;
                    real_T Q_d = TM.Q_damaged;
                    Vector3r F_r = TM.F_remaining;
                    Vector3r F_d = TM.F_damaged;
                    Q_remaining += Q_r;
                    Q_damaged += Q_d;
                    F_remaining += F_r;
                    F_damaged += F_d;
                }

                return retVals_torque_force_propeller{ Q_remaining, Q_damaged, F_remaining, F_damaged };
            }

            // Method to compute the forces and moments caused by the aerodynamic changes
            retVals_aero_FM_propeller computeAeroFM(const int& number_sections, const real_T& omega, const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs,
                const Vector3r& body_velocity, const Vector3r& pqr, const real_T& rho = 1.225f) {
                // Obtain the aero related forces and moments
                if (blades_.empty()) {
                    createBlades();
                }
                real_T T = computeLiftTorqueMatlab(body_velocity, pqr, omega, rho);
                std::function<real_T(real_T, real_T)> inflow_data = computeInducedInflow(T, rho, omega);
                retVals_thrust_moment_propeller aeroFM_1 = computeThrustMoment(number_sections, omega, cla_coeffs, cda_coeffs, body_velocity, pqr, inflow_data, rho);
                retVals_torque_force_propeller aeroFM_2 = computeTorqueForce(number_sections, omega, cla_coeffs, cda_coeffs, body_velocity, pqr, inflow_data, rho);
                real_T aero_T1 = aeroFM_1.T_damaged;
                Vector3r aero_M1 = aeroFM_1.M_damaged;
                Vector3r aero_T2 = aeroFM_2.F_damaged;
                real_T aero_M2 = aeroFM_2.Q_damaged;

                aero_T2.z() += aero_T1;
                aero_M1.z() += aero_M2;

                return retVals_aero_FM_propeller{ aero_M1, aero_T2};
            }

            // Method to compute the forces and moments caused by the aerodynamic and mass changes
            retVals_mass_aero_FM_propeller computeMassAeroFM(const int& number_sections, const real_T& omega, const Quaternionr& attitude, const vector<real_T>& cla_coeffs, const vector<real_T>& cda_coeffs,
                const Vector3r& body_velocity, const Vector3r& pqr, const real_T& rho = 1.225f) {
                // Obtain the mass related forces and moments
                retVals_compute_cg_forces_moments mass_FM = computeCgForcesMoments(omega, attitude);
                Vector3r mass_F = mass_FM.F_vector;
                Vector3r mass_M = mass_FM.M_vector;

                // Obtain the aero related forces and moments
                retVals_aero_FM_propeller aero_FM = computeAeroFM(number_sections, omega, cla_coeffs, cda_coeffs, body_velocity, pqr, rho);
                Vector3r aero_F = aero_FM.F_vector;
                Vector3r aero_M = aero_FM.M_vector;

                // Add the mass and aero effects
                Vector3r delta_F = mass_F - aero_F;
                Vector3r delta_M = mass_M - aero_M;

                return retVals_mass_aero_FM_propeller{ delta_F, delta_M, mass_F, mass_M, aero_F, aero_M };
            }

            real_T get_propeller_lost_mass()
            {
                // Compute the lost propeller mass
                real_T lost_propeller_mass = (healthy_propeller_mass_ - propeller_mass_);
                return lost_propeller_mass;
            }

            //% Helper functions
            void updateRotationAngle(const real_T& omega, const real_T& delta_t) {
                omega_ = omega;
                rotation_angle_ += omega_ * delta_t * SN_[propeller_number_];
                if (rotation_angle_ < 0) {
                    rotation_angle_ += 2.0f * M_PIf;
                }
                rotation_angle_ = fmod(rotation_angle_, 2.0f * M_PIf);
            }

            void setRotationAngle(const real_T& rotation_angle) {
                rotation_angle_ = rotation_angle;
            }

            // Personal optimization (gradient descend) function to substitute the scipy.optimize.minimize Nelder_Mead function
            // used for the computation of the induced velocity.The function stops when the denominator becomes zero, when the
            // maximum number of iterations has been reached or when the optimization variable has barely changed in 20 iterations.
            // This function takes less time than the scipy counterpart and it can be implemented in C++.
            real_T optimizeGradientDescend(std::function<real_T(const real_T&)> func, const real_T& x0, std::function<real_T(const real_T&)> den) {
                real_T x = x0;
                real_T alpha = 0.5f;
                real_T th = 0.01f;
                int counter = 0;
                real_T previous_der = func(x);
                for (int i = 0; i < 10000; i++) {
                    if (den(x) < 1e-10f) {
                        return x;
                    }

                    // Compute and apply the gradient
                    real_T der = func(x);
                    real_T x_new = x - alpha * der;

                    if (der!=0) {
                        if (previous_der / der < 0) {
                            alpha = alpha / 2.f;
                        }
                    }

                    // If there has not been a change in x, return function
                    real_T step = x_new - x;
                    x = x_new;
                    previous_der = der;
                    if (abs(step) < th) {
                        counter += 1;
                        if (counter > 20) {
                            return x;
                        }
                    }
                    else {
                        counter = 0;
                    }
                }
                return x;
            }

            // Function to find the 5th degree polynomial with 2 parameters
            static std::vector<real_T> computeP52(const real_T& x1, const real_T& x2, const real_T& U = 1)
            {

                real_T A1 = U;
                real_T A2 = x1 * U;
                real_T A3 = x2 * U;
                real_T A4 = x2 * x2 * U;
                real_T A5 = x1 * x2 * U;
                real_T A6 = x2 * x2 * U;
                real_T A7 = x1 * x1 * x1 * U;
                real_T A8 = x1 * x1 * x2 * U;
                real_T A9 = x1 * x2 * x2 * U;
                real_T A10 = x2 * x2 * x2 * U;
                real_T A11 = x1 * x1 * x1 * x1 * U;
                real_T A12 = x1 * x1 * x1 * x2 * U;
                real_T A13 = x1 * x1 * x2 * x2 * U;
                real_T A14 = x1 * x2 * x2 * x2 * U;
                real_T A15 = x2 * x2 * x2 * x2 * U;
                real_T A16 = x1 * x1 * x1 * x1 * x1 * U;
                real_T A17 = x1 * x1 * x1 * x1 * x2 * U;
                real_T A18 = x1 * x1 * x1 * x2 * x2 * U;
                real_T A19 = x1 * x1 * x2 * x2 * x2 * U;
                real_T A20 = x1 * x2 * x2 * x2 * x2 * U;
                real_T A21 = x2 * x2 * x2 * x2 * x2 * U;

                std::vector<real_T> P52_output = { A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12, A13, A14, A15, A16, A17, A18, A19, A20, A21 };
                return P52_output;

            }

            static real_T computeBeta(const real_T& uy, const real_T& vx)
            {
                if (uy == 0 && vx == 0)
                {
                    return 0.0f;
                }

                real_T beta = atan2(uy, vx);

                if (beta < 0)
                {
                    beta = 2.0f * M_PIf + beta;
                }
                beta = beta - M_PIf / 2;
                if (beta < 0)
                {
                    beta = 2.0f * M_PIf + beta;
                }
                return beta;
            }

            // Function to compute the psi value from the aerodynamic Matlab model
            static real_T compute_psi(const real_T& beta, const real_T& arm_angle, const int& propeller_number) {
                real_T psi_h = 0.0f;
                if (propeller_number == 0) {
                    psi_h = beta - (360 + 90 - arm_angle);
                }
                else if (propeller_number == 1) {
                    psi_h = beta - (360 - 90 + arm_angle);
                }
                else if (propeller_number == 2) {
                    psi_h = beta - (180 + 90 - arm_angle);
                }
                else if (propeller_number == 3) {
                    psi_h = beta - (180 - 90 + arm_angle);
                }

                psi_h /= (180.0f / M_PIf);
                psi_h = fmod(psi_h, 2.0f * M_PIf);

                if (propeller_number == 1 || propeller_number == 3) {
                    psi_h = 2 * M_PIf - psi_h;
                }

                return psi_h;
            }

            static std::vector<real_T> compute_Fn(const real_T& x, const int& n = 1, const real_T& bias = 1.0f, const real_T& U = 1.0f, const real_T& alpha = 0.0f, const real_T& beta = 0.0f)
            {

                std::vector<real_T> A_F;
                if (bias != 0)
                {
                    A_F.push_back(1.0f);
                }

                for (int i = 0; i < n; i++)
                {
                    real_T sine_component = sin((i + 1.0f) * x) * U;
                    std::vector<real_T> Ai1 = compute_P32(alpha, beta, sine_component);

                    real_T cosine_component = cos((i + 1.0f) * x) * U;
                    std::vector<real_T> Ai2 = compute_P32(alpha, beta, cosine_component);

                    A_F.insert(A_F.end(), Ai1.begin(), Ai1.end());
                    A_F.insert(A_F.end(), Ai2.begin(), Ai2.end());
                }

                return A_F;
            }

            // regressor generator 3rd order polynomial function with 2 inputs
            static std::vector<real_T> compute_P32(const real_T& x1, const real_T& x2, const real_T& U = 1, const real_T& bias = -1)
            {
                real_T A2 = x1 * U;
                real_T A3 = x2 * U;
                real_T A4 = x1 * x1 * U;
                real_T A5 = x2 * x2 * U;
                real_T A6 = x1 * x2 * U;
                real_T A7 = x1 * x1 * x1 * U;
                real_T A8 = x2 * x2 * x2 * U;
                real_T A9 = x1 * x2 * x2 * U;
                real_T A10 = x1 * x1 * x2 * U;

                std::vector<real_T> P32_output;
                if (bias != -1)
                {
                    real_T A1 = 1.0f;
                    P32_output = { A1, A2, A3, A4, A5, A6, A7, A8, A9, A10 };
                }
                else
                {
                    P32_output = { A2, A3, A4, A5, A6, A7, A8, A9, A10 };
                }
                return P32_output;
            }

        private:
            // Parameters that will be initialised in __init__
            int propeller_number_;
            int number_blades_;
            vector<real_T> chords_;
            vector<real_T> hs_;
            real_T radius_hub_;
            real_T healthy_propeller_mass_;
            real_T percentage_hub_m_;
            real_T angle_first_blade_;
            real_T start_twist_;
            real_T final_twist_;
            vector<real_T> broken_percentage_;
            real_T l_;
            real_T b_;
            int signr_;
            real_T g_;
            vector<int> SN_;
            vector<vector<real_T>> d_;
            real_T length_blade_ = 0;

            // Parameters that will be initialised in other functions
            vector<DamagedBlade> blades_;
            real_T cg_x_ = 0.0;
            real_T cg_y_ = 0.0;
            real_T cg_r_ = -1.0;
            real_T healthy_blade_m_;
            real_T propeller_mass_ = -1.0;
            Vector3r propeller_velocity_ = Vector3r::Zero();
            real_T omega_ = -1.0;
            real_T rotation_angle_ = 0.0;

            // Constant expressions
            static constexpr real_T rad2deg_factor_ = 180.0f / M_PIf;
            static constexpr real_T deg2rad_factor_ = M_PIf / 180.0f;
        };

    }
}



#endif // !physics_DamagedPropeller_hpp
