// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#ifndef airsim_core_FastPhysicsEngine_hpp
#define airsim_core_FastPhysicsEngine_hpp

#include "common/FirstOrderFilter.hpp"
#include <numeric>
#include "DamagedPropeller.hpp"

#include "common/Common.hpp"
#include "physics/PhysicsEngineBase.hpp"
#include <iostream>
#include <sstream>
#include <fstream>
#include <memory>
#include "common/CommonStructs.hpp"
#include "common/SteppableClock.hpp"
#include <cinttypes>
#include <vector>
#include <cfloat>
#include <math.h>


namespace msr {
    namespace airlib {

        class FastPhysicsEngine : public PhysicsEngineBase {
        public:
            FastPhysicsEngine(bool enable_ground_lock = true, Vector3r wind = Vector3r::Zero())
                : enable_ground_lock_(enable_ground_lock), wind_(wind)
            {
                real_T propeller_mass_g = 5.07f;                 // [g] measured 5.07
                real_T propeller_mass = propeller_mass_g / 1000.0f;  // [kg]
                real_T blade_mass_g = 1.11f;                     // [g] measured 1.11.Mass of a single blade
                int n_blades = 3.0f;                            // [-] measured 3
                real_T percentage_hub_m = (propeller_mass_g - blade_mass_g * n_blades) / propeller_mass_g * 100.0f;
                real_T tip_chord = 0.008f;                 // [m] measured 0.008
                real_T largest_chord_length = 0.02f;       // [m] measured 0.02
                real_T second_segment_length = 0.032f;     // [m] measured 0.032
                real_T base_chord = 0.013f;                // [m] measured 0.013
                real_T length_blade_origin = 0.075f;       // [m] measured 0.076
                real_T radius_hub = 0.011f;                // [m] measured 0.012
                real_T start_twist = 27.0f;                  // [deg] measured 26.39[deg]
                real_T finish_twist = 5.0f;
                vector<real_T> chord_lengths_rt_lst { base_chord, largest_chord_length, tip_chord };
                real_T first_segment_length = length_blade_origin - radius_hub - second_segment_length;
                vector<real_T> length_trapezoids_rt_lst{ first_segment_length, second_segment_length };

                // User input
                vector<real_T> percentage_broken_blade_length{ 0, 0, 0 };   // [%]
                real_T angle_first_blade = 0;                 // [deg] angle of the first blade with respect to the propeller coord.frame

                for (int propeller_number = 0; propeller_number < 4; propeller_number++)
                {
                    DamagedPropeller propeller = DamagedPropeller(propeller_number, n_blades, chord_lengths_rt_lst, length_trapezoids_rt_lst, radius_hub, propeller_mass, percentage_hub_m,
                        angle_first_blade, start_twist, finish_twist, percentage_broken_blade_length);
                    damaged_propellers_.push_back(propeller);
                }

            }

            //*** Start: UpdatableState implementation ***//
            virtual void resetImplementation() override
            {
                for (PhysicsBody* body_ptr : *this) {
                    initPhysicsBody(body_ptr);
                }
            }

            virtual void insert(PhysicsBody* body_ptr) override
            {
                PhysicsEngineBase::insert(body_ptr);

                initPhysicsBody(body_ptr);
            }

            virtual void update() override
            {
                PhysicsEngineBase::update();

                for (PhysicsBody* body_ptr : *this) {
                    updatePhysics(*body_ptr);
                }
            }
            virtual void reportState(StateReporter& reporter) override
            {
                for (PhysicsBody* body_ptr : *this) {
                    reporter.writeValue("Phys", debug_string_.str());
                    reporter.writeValue("Is Grounded", body_ptr->isGrounded());
                    reporter.writeValue("Force (world)", body_ptr->getWrench().force);
                    reporter.writeValue("Torque (body)", body_ptr->getWrench().torque);
                }
                //call base
                UpdatableObject::reportState(reporter);
            }
            //*** End: UpdatableState implementation ***//

            // Set Wind, for API and Settings implementation
            void setWind(const Vector3r& wind) override
            {
                wind_ = wind;
            }

        private:
            void initPhysicsBody(PhysicsBody* body_ptr)
            {
                body_ptr->last_kinematics_time = clock()->nowNanos();
            }

            void updatePhysics(PhysicsBody& body)
            {
                TTimeDelta dt = clock()->updateSince(body.last_kinematics_time);

                body.lock();
                if (initial_time == -1)
                {
                    previous = body.getKinematics();
                }
                initial_time = 1;
                //get current kinematics state of the body - this state existed since last dt seconds
                const Kinematics::State& current = body.getKinematics();
                Kinematics::State next;
                Wrench next_wrench;

                //first compute the response as if there was no collision
                //this is necessary to take in to account forces and torques generated by body
                getNextKinematicsNoCollisionFM_BB2_dml_6DOF(dt, body, current, next, next_wrench, wind_, propeller_damage_coefficients);

                //if there is collision, see if we need collision response
                const CollisionInfo collision_info = body.getCollisionInfo();
                CollisionResponse& collision_response = body.getCollisionResponseInfo();
                //if collision was already responded then do not respond to it until we get updated information
                if (body.isGrounded() || (collision_info.has_collided && collision_response.collision_time_stamp != collision_info.time_stamp)) {
                    bool is_collision_response = getNextKinematicsOnCollision(dt, collision_info, body,
                        current, next, next_wrench, enable_ground_lock_);
                    updateCollisionResponseInfo(collision_info, next, is_collision_response, collision_response);
                    //throttledLogOutput("*** has collision", 0.1);
                }
                //else throttledLogOutput("*** no collision", 0.1);

                //Utils::log(Utils::stringf("T-VEL %s %" PRIu64 ": ", 
                //    VectorMath::toString(next.twist.linear).c_str(), clock()->getStepCount()));
                previous = current;
                body.setWrench(next_wrench);
                body.updateKinematics(next);
                body.unlock();
                //body.collectCameraData();


                //TODO: this is now being done in PawnSimApi::update. We need to re-think this sequence
                //with below commented out - Arducopter GPS may not work.
                //body.getEnvironment().setPosition(next.pose.position);
                //body.getEnvironment().update();

            }

            static void updateCollisionResponseInfo(const CollisionInfo& collision_info, const Kinematics::State& next,
                bool is_collision_response, CollisionResponse& collision_response)
            {
                collision_response.collision_time_stamp = collision_info.time_stamp;
                ++collision_response.collision_count_raw;

                //increment counter if we didn't collided with high velocity (like resting on ground)
                if (is_collision_response && next.twist.linear.squaredNorm() > kRestingVelocityMax * kRestingVelocityMax)
                    ++collision_response.collision_count_non_resting;

            }

            //return value indicates if collision response was generated
            static bool getNextKinematicsOnCollision(TTimeDelta dt, const CollisionInfo& collision_info, PhysicsBody& body,
                const Kinematics::State& current, Kinematics::State& next, Wrench& next_wrench, bool enable_ground_lock)
            {
                /************************* Collision response ************************/
                const real_T dt_real = static_cast<real_T>(dt);

                //are we going away from collision? if so then keep using computed next state
                if (collision_info.normal.dot(next.twist.linear) >= 0.0f)
                    return false;

                /********** Core collision response ***********/
                //get avg current velocity
                const Vector3r vcur_avg = current.twist.linear + current.accelerations.linear * dt_real;

                //get average angular velocity
                const Vector3r angular_avg = current.twist.angular + current.accelerations.angular * dt_real;

                //contact point vector
                Vector3r r = collision_info.impact_point - collision_info.position;

                //see if impact is straight at body's surface (assuming its box)
                const Vector3r normal_body = VectorMath::transformToBodyFrame(collision_info.normal, current.pose.orientation);
                const bool is_ground_normal = Utils::isApproximatelyEqual(std::abs(normal_body.z()), 1.0f, kAxisTolerance);
                bool ground_collision = false;
                const float z_vel = vcur_avg.z();
                const bool is_landing = z_vel > std::abs(vcur_avg.x()) && z_vel > std::abs(vcur_avg.y());

                real_T restitution = body.getRestitution();
                real_T friction = body.getFriction();

                if (is_ground_normal && is_landing
                    // So normal_body is the collision normal translated into body coords, why does an x==1 or y==1
                    // mean we are coliding with the ground???
                    // || Utils::isApproximatelyEqual(std::abs(normal_body.x()), 1.0f, kAxisTolerance) 
                    // || Utils::isApproximatelyEqual(std::abs(normal_body.y()), 1.0f, kAxisTolerance) 
                    ) {
                    // looks like we are coliding with the ground.  We don't want the ground to be so bouncy
                    // so we reduce the coefficient of restitution.  0 means no bounce.
                    // TODO: it would be better if we did this based on the material we are landing on.
                    // e.g. grass should be inelastic, but a hard surface like the road should be more bouncy.
                    restitution = 0;
                    // crank up friction with the ground so it doesn't try and slide across the ground
                    // again, this should depend on the type of surface we are landing on.
                    friction = 1;

                    //we have collided with ground straight on, we will fix orientation later
                    ground_collision = is_ground_normal;
                }

                //velocity at contact point
                const Vector3r vcur_avg_body = VectorMath::transformToBodyFrame(vcur_avg, current.pose.orientation);
                const Vector3r contact_vel_body = vcur_avg_body + angular_avg.cross(r);

                /*
                    GafferOnGames - Collision response with columb friction
                    http://gafferongames.com/virtual-go/collision-response-and-coulomb-friction/
                    Assuming collision is with static fixed body,
                    impulse magnitude = j = -(1 + R)V.N / (1/m + (I'(r X N) X r).N)
                    Physics Part 3, Collision Response, Chris Hecker, eq 4(a)
                    http://chrishecker.com/images/e/e7/Gdmphys3.pdf
                    V(t+1) = V(t) + j*N / m
                */
                const real_T impulse_mag_denom = 1.0f / body.getMass() +
                    (body.getInertiaInv() * r.cross(normal_body))
                    .cross(r)
                    .dot(normal_body);
                const real_T impulse_mag = -contact_vel_body.dot(normal_body) * (1 + restitution) / impulse_mag_denom;

                next.twist.linear = vcur_avg + collision_info.normal * (impulse_mag / body.getMass());
                next.twist.angular = angular_avg + r.cross(normal_body) * impulse_mag;

                //above would modify component in direction of normal
                //we will use friction to modify component in direction of tangent
                const Vector3r contact_tang_body = contact_vel_body - normal_body * normal_body.dot(contact_vel_body);
                const Vector3r contact_tang_unit_body = contact_tang_body.normalized();
                const real_T friction_mag_denom = 1.0f / body.getMass() +
                    (body.getInertiaInv() * r.cross(contact_tang_unit_body))
                    .cross(r)
                    .dot(contact_tang_unit_body);
                const real_T friction_mag = -contact_tang_body.norm() * friction / friction_mag_denom;

                const Vector3r contact_tang_unit = VectorMath::transformToWorldFrame(contact_tang_unit_body, current.pose.orientation);
                next.twist.linear += contact_tang_unit * friction_mag;
                next.twist.angular += r.cross(contact_tang_unit_body) * (friction_mag / body.getMass());

                //TODO: implement better rolling friction
                next.twist.angular *= 0.9f;

                // there is no acceleration during collision response, this is a hack, but without it the acceleration cancels
                // the computed impulse response too much and stops the vehicle from bouncing off the collided object.
                next.accelerations.linear = Vector3r::Zero();
                next.accelerations.angular = Vector3r::Zero();

                next.pose = current.pose;
                if (enable_ground_lock && ground_collision) {
                    float pitch, roll, yaw;
                    VectorMath::toEulerianAngle(next.pose.orientation, pitch, roll, yaw);
                    pitch = roll = 0;
                    next.pose.orientation = VectorMath::toQuaternion(pitch, roll, yaw);

                    //there is a lot of random angular velocity when vehicle is on the ground
                    next.twist.angular = Vector3r::Zero();

                    // also eliminate any linear velocity due to twist - since we are sitting on the ground there shouldn't be any.
                    next.twist.linear = Vector3r::Zero();
                    next.pose.position = collision_info.position;
                    body.setGrounded(true);

                    // but we do want to "feel" the ground when we hit it (we should see a small z-acc bump)
                    // equal and opposite our downward velocity.
                    next.accelerations.linear = -0.5f * body.getMass() * vcur_avg;

                    //throttledLogOutput("*** Triggering ground lock", 0.1);
                }
                else
                {
                    //else keep the orientation
                    next.pose.position = collision_info.position + (collision_info.normal * collision_info.penetration_depth) + next.twist.linear * (dt_real * kCollisionResponseCycles);
                }
                next_wrench = Wrench::zero();

                //Utils::log(Utils::stringf("*** C-VEL %s: ", VectorMath::toString(next.twist.linear).c_str()));

                return true;
            }

            /// <summary>
            /// Function to find the index of an element in a vector with the highest value but smaller than a specific threshold
            /// </summary>
            /// <param name="value">threshold</param>
            /// <param name="x">vector of elements</param>
            /// <returns>index of the largest element that is smaller than the threshold</returns>
            static int findNearestNeighbourIndex(float value, std::vector<float>& x)
            {
                float dist = FLT_MAX;
                int idx = -1;
                for (int i = 0; i < x.size(); ++i) {
                    float newDist = value - x[i];
                    if (newDist >= 0 && newDist < dist) {
                        dist = newDist;
                        idx = i;
                    }
                }
                return idx;
            }

            /// <summary>
            /// Interpolation for 2D gridded data. Similar behaviour to the Matlab function interp2.
            /// Returns interpolated values of a function of two variables at specific query points 
            /// using linear interpolation. The results always pass through the original sampling of the function
            /// </summary>
            /// <param name="x">coordinates of the sample points</param>
            /// <param name="y">coordinates of the sample points</param>
            /// <param name="V">corresponding function values at each sample point</param>
            /// <param name="x_new">coordinates of the query points</param>
            /// <param name="y_new">coordinates of the query points</param>
            /// <returns>interpolated value</returns>
            static float interp2(std::vector<float>& x, std::vector<float>& y, Eigen::Matrix<real_T, 15, 15> V, float x_new, float y_new)
            {
                int x_length = x.size();
                int y_length = y.size();
                float dx_x1, dV_x1, slope_x1, dV_x2, slope_x2;
                float dx, dV, slope, col_1, col_2, interpolated;
                
                int idx_x = findNearestNeighbourIndex(x_new, x);
                int idx_y = findNearestNeighbourIndex(y_new, y);

                dx_x1 = x[idx_x + 1] - x[idx_x];
                dV_x1 = V.col(idx_y)[idx_x + 1] - V.col(idx_y)[idx_x];
                slope_x1 = dV_x1 / dx_x1;

                dV_x2 = V.col(idx_y + 1)[idx_x + 1] - V.col(idx_y + 1)[idx_x];
                slope_x2 = dV_x2 / dx_x1;

                col_1 = V.col(idx_y)[idx_x] + (x_new - x[idx_x]) * slope_x1;
                col_2 = V.col(idx_y + 1)[idx_x] + (x_new - x[idx_x]) * slope_x2;

                dx = y[idx_y + 1] - y[idx_y];
                dV = col_2 - col_1;
                slope = dV / dx;

                interpolated = col_1 + (y_new - y[idx_y]) * slope;

                return interpolated;
            }

            /// <summary>
            /// Implementation of the Matlab mod function
            /// </summary>
            /// <param name="numerator">numerator in coefficient</param>
            /// <param name="denominator">denominator in coefficient</param>
            /// <returns>mod of the two input arguments</returns>
            real_T matlabFmod(real_T numerator, real_T denominator)
            {
                real_T result = fmod(numerator, denominator);
                return result >= 0 ? result : result + denominator;
            }

            /// <summary>
            /// Function which adds Gaussian noise to a given value
            /// </summary>
            /// <param name="clean_number">clean value to which Gaussian noise should be added</param>
            /// <param name="mean">mean of Gaussian noise</param>
            /// <param name="variance">variance of Gaussian noise</param>
            /// <returns></returns>
            real_T gaussian_noise(real_T clean_number, real_T mean, real_T variance)
            {
                real_T standard_deviation = sqrt(variance);
                std::normal_distribution<float> distribution(mean, standard_deviation);
                real_T dirty_number = clean_number + distribution(generator);
                return dirty_number;
            }
            
            /// <summary>
            /// Function which adds Gaussian noise to a 3 element array
            /// </summary>
            /// <param name="clean_numbers">clean input vector</param>
            /// <param name="means">means of the Gaussian noise for each axis</param>
            /// <param name="variances">variances of the Gaussian noise for each axis</param>
            /// <returns>vector polluted with Gaussian noise</returns>
            Vector3r gaussian_noise(Vector3r clean_numbers, Vector3r means, Vector3r variances)
            {
                real_T dirty_number_x = gaussian_noise(clean_numbers.x(), means.x(), variances.x());
                real_T dirty_number_y = gaussian_noise(clean_numbers.y(), means.y(), variances.y());
                real_T dirty_number_z = gaussian_noise(clean_numbers.z(), means.z(), variances.z());
                Vector3r dirty_number(dirty_number_x, dirty_number_y, dirty_number_z);
                return dirty_number;
            }

            /// <summary>
            /// Function which decides or not to add Gaussian noise to a number
            /// </summary>
            /// <param name="clean_number">number without noise</param>
            /// <param name="mean">Gaussian mean</param>
            /// <param name="variance">Gaussian variance</param>
            /// <returns>potentially Gaussian noise polluted input clean_number</returns>
            real_T apply_process_noise(real_T clean_number, real_T mean, real_T variance)
            {
                if (activate_process_noise)
                {
                    real_T dirty_number = gaussian_noise(clean_number, mean, variance);
                    return dirty_number;
                }
                else
                {
                    return clean_number;
                }
            }

            /// <summary>
            /// Function which decides or not to add Gaussian noise to a 3D vector
            /// </summary>
            /// <param name="clean_number">vector without noise</param>
            /// <param name="mean">Gaussian mean for each element of the vector</param>
            /// <param name="variance">Gaussian variance for each element of the vector</param>
            /// <returns>potentially Gaussian noise polluted input clean_number</returns>
            Vector3r apply_process_noise(Vector3r clean_number, Vector3r mean, Vector3r variance)
            {
                if (activate_process_noise)
                {
                    Vector3r dirty_number = gaussian_noise(clean_number, mean, variance);
                    return dirty_number;
                }
                else
                {
                    return clean_number;
                }
            }

            template <typename T> int sgn(T val) {
                return (T(0) <= val) - (val < T(0));
            }

            /// <summary>
            /// Applies state hedging to the linear and angular velocities
            /// </summary>
            /// <param name="u">linear velocity in the x-direction</param>
            /// <param name="v">linear velocity in the y-direction</param>
            /// <param name="w">linear velocity in the z-direction</param>
            /// <param name="p">angular velocity in the x-direction</param>
            /// <param name="q">angular velocity in the y-direction</param>
            /// <param name="r">angular velocity in the z-direction</param>
            /// <returns></returns>
            auto stateHedging(real_T u, real_T v, real_T w, real_T p, real_T q, real_T r)
            {
                // state hedging boundary parameters for Bebop2
                real_T pmax = 0.008f;
                real_T qmax = 0.02f;
                real_T rmax = 0.015f;
                
                real_T umax = 0.5f;
                real_T vmax = 0.5f;
                real_T wmax = 0.05f;
                real_T wmin = -0.2f;

                // transfer pqr to the bound
                real_T Vpq = -qmax / pmax * abs(p) + qmax - abs(q);
                real_T p0 = p;
                real_T q0 = q;
                if (Vpq < 0. && abs(p)>0.001)
                {
                    p0 = sgn(q) * qmax / (sgn(q * p) * qmax / pmax + q / p);
                    q0 = q / p * p0;
                }
                else if (Vpq < 0.)
                {
                    p0 = 0;
                    q0 = qmax;
                }

                real_T r0 = r;
                if (abs(r) > rmax)
                {
                    r0 = sgn(r) * rmax;
                }

                // transfer uvw to the bound
                real_T Vuv = u * u + v * v - umax * umax;
                real_T u0 = u;
                real_T v0 = v;
                if (Vuv > 0 && abs(u) > 0.001)
                {
                    u0 = sgn(u) * sqrt(umax * umax / ((v / u) * (v / u) + 1));
                    v0 = v / u * u0;
                }
                else if (Vuv > 0)
                {
                    u0 = 0;
                    v0 = umax;
                }

                real_T w0 = w;
                if (w > wmax)
                {
                    w0 = wmax;
                }
                else if (w < wmin)
                {
                    w0 = wmin;
                }
                return retVals_states{ u0, v0, w0, p0, q0, r0 };
            }

            /// <summary>
            /// Function to compute the force coefficients
            /// </summary>
            /// <param name="beta">sideslip angle</param>
            /// <param name="up">difference in propeller rotational speed around the x-axis</param>
            /// <param name="uq">difference in propeller rotational speed around the y-axis</param>
            /// <param name="ur">difference in propeller rotational speed around the z-axis</param>
            /// <param name="u">linear velocity in the x-direction</param>
            /// <param name="v">linear velocity in the y-direction</param>
            /// <param name="w">linear velocity in the z-direction</param>
            /// <param name="p">angular velocity in the x-direction</param>
            /// <param name="q">angular velocity in the y-direction</param>
            /// <param name="r">angular velocity in the z-direction</param>
            /// <param name="vi">induced velocity</param>
            /// <returns>dimensionless force coefficients</returns>
            auto force_dimensionless_BB2(real_T beta, real_T up, real_T uq, real_T ur, real_T u, real_T v, real_T w, real_T p, real_T q, real_T r, real_T vi)
            {
                // Computations for Ct
                std::vector<real_T> Ct_A1{ 1, u * u + v * v, -w + vi, w, u * u * u, uq * u, u, uq, u * u * w * w, v * v * u * u * u * w };
                std::vector<real_T> Ct_k1{ 0.350649467f, - 0.834840346f, - 1.181790542f, - 0.572337294f, 0.558029278f, - 0.005075139f, - 0.015688025f, - 0.000567546f, 11.85932508f, - 314.2433046f };
                std::vector<real_T> Ct_A2{ 1, u * u + v * v, -w + vi, w, abs(v) * uq, u, abs(v) * uq * w, v * v * uq * powf(w, 5), v * v * powf(u, 4), v * v * uq * u, v * v * u * u * w, v * v * abs(q) * u * u * w };
                std::vector<real_T> Ct_k2{ 0.217632672f, -0.363087072f, -0.713021858f, -0.314832014f, -0.032823564f, -0.019832009f, -0.352240961f, 25168.56046f, 43.45950654f, 0.586207259f, -18.11478065f, -9221.159726f };
                std::vector<real_T> Ct_A3{ 1, u * u + v * v,-w + vi,v * v * w,uq * u, v * v * uq, w * w, abs(v) * uq * powf(u,3) * w };
                std::vector<real_T> Ct_k3{ 0.048354336f, 0.073426887f, -0.116204451f, 1.309353321f, -0.037616532f, -0.051136599f, -0.16196609f, -361.5340041f };

                real_T Ct;
                if (abs(beta) <= 30)
                {
                    Ct = std::inner_product(Ct_A1.begin(), Ct_A1.end(), Ct_k1.begin(), 0.0f);
                }    
                else if (abs(beta) <= 60)
                {
                    Ct = std::inner_product(Ct_A2.begin(), Ct_A2.end(), Ct_k2.begin(), 0.0f);
                }
                else
                {
                    Ct = std::inner_product(Ct_A3.begin(), Ct_A3.end(), Ct_k3.begin(), 0.0f);
                }
                    
                // Computations for Cx
                std::vector<real_T> Cx_A1{ 1.0f, u, w * powf(u,3), w * w * u * u, powf(abs(v), 3), w * abs(v) * abs(v) };
                std::vector<real_T> Cx_k1{ 0.0f, - 0.024526066f, 1.293007509f, 1.55227301f, 0.612890951f, 0.491536283f };
                std::vector<real_T> Cx_A2{ 1.0f, u, v*v, w };
                std::vector<real_T> Cx_k2{ 0.0f, - 0.024327615f, 0.022255392f, 0.000799817f };
                std::vector<real_T> Cx_A3{ 1.0f, u, u*u, abs(v)};
                std::vector<real_T> Cx_k3{ 0.0f, - 0.023672933f, 0.037625469f, 0.000425188f };

                real_T Cx;
                if (abs(beta) <= 30)
                {
                    Cx = std::inner_product(Cx_A1.begin(), Cx_A1.end(), Cx_k1.begin(), 0.0f);
                }
                else if (abs(beta) <= 60)
                {
                    Cx = std::inner_product(Cx_A2.begin(), Cx_A2.end(), Cx_k2.begin(), 0.0f);
                }
                else
                {
                    Cx = std::inner_product(Cx_A3.begin(), Cx_A3.end(), Cx_k3.begin(), 0.0f);
                }

                // Computations for the Cy
                std::vector<real_T> Cy_A1{ 1.0f, v, w * w * v, abs(u), w * w, v * v * v, w * v * abs(u) };
                std::vector<real_T> Cy_k1{ 0.0f, - 0.025299121f, - 1.577325292f, 0.001766453f, - 0.021483641f, - 0.554514508f, - 0.212647222f };
                std::vector<real_T> Cy_A2{ 1.0f, v, w * v, w * powf(v, 3), powf(v, 3), w * w * v * v };
                std::vector<real_T> Cy_k2{ 0.0f, - 0.02369769f, 0.198240073f, - 4.20402807f, - 0.095904804f, 1.069535431f };
                std::vector<real_T> Cy_A3{ 1.0f, v, w * v, w * v * abs(u), powf(v,3), w * powf(abs(u),3), powf(w,3), v * v };
                std::vector<real_T> Cy_k3{ 0.0f, - 0.02402552f, 0.055312012f, 0.229868618f, - 0.06506344f, 8.654558373f, - 0.195754684f, 0.004037991f };

                real_T Cy;
                if (abs(beta) <= 30)
                {
                    Cy = std::inner_product(Cy_A1.begin(), Cy_A1.end(), Cy_k1.begin(), 0.0f);
                }
                else if (abs(beta) <= 60)
                {
                    Cy = std::inner_product(Cy_A2.begin(), Cy_A2.end(), Cy_k2.begin(), 0.0f);
                }
                else
                {
                    Cy = std::inner_product(Cy_A3.begin(), Cy_A3.end(), Cy_k3.begin(), 0.0f);
                }

                return retVals_forces{ Ct, Cx, Cy };
            }

            /// <summary>
            /// Function that computes moment coefficients
            /// </summary>
            /// <param name="beta">sideslip angle</param>
            /// <param name="up">difference in propeller rotational speed around the x-axis</param>
            /// <param name="uq">difference in propeller rotational speed around the y-axis</param>
            /// <param name="ur">difference in propeller rotational speed around the z-axis</param>
            /// <param name="u">linear velocity in the x-direction</param>
            /// <param name="v">linear velocity in the y-direction</param>
            /// <param name="w">linear velocity in the z-direction</param>
            /// <param name="p">angular velocity in the x-direction</param>
            /// <param name="q">angular velocity in the y-direction</param>
            /// <param name="r">angular velocity in the z-direction</param>
            /// <param name="vi">induced velocity</param>
            /// <returns>dimensionless moment coefficients</returns>
            auto moment_dimensionless_BB2(real_T beta, real_T up, real_T uq, real_T ur, real_T u, real_T v, real_T w, real_T p, real_T q, real_T r)
            {

                // Computations for Cl
                std::vector<real_T> Cl_A1{ 1, up, up * w * w, p, v, up * w, u * u * v, up * powf(w, 5), abs(u) * p, w * w };
                std::vector<real_T> Cl_k1{ 0.0, 0.002270406, - 0.730344313, - 0.070192814, - 0.030741693, - 0.049111438, 0.648613152, - 127.5574068, - 0.517018367, - 0.018174949 };
                std::vector<real_T> Cl_A2{ 1, up, v, powf(v,3) * w, u * u * up * powf(w,5), up * w,p,up * v,up * powf(v,3) * w,abs(u) * up, abs(u) * v, up * w * w,v * v, abs(u) * up * powf(w,4), u * u * powf(v,3) * w, u * u * up * w, u * u * up };
                std::vector<real_T> Cl_k2{ 0.0, 0.003263764, - 0.027590476, - 59.60927699, 22330.71956, - 0.04916752, - 0.079038485, 0.02456213, 83.12039978, - 0.019060493, 0.086224155, - 0.857166645, - 0.08192604, 554.9298585, 1082.177412, - 0.394429003, 0.027431262 };
                std::vector<real_T> Cl_A3{ 1, up, v, abs(u) * v, abs(u) * up, w, p, abs(u) * w, abs(u) * up * powf(v,4), u * u * up, u * u * v, u * u * w,abs(u) * v * v };
                std::vector<real_T> Cl_k3{ 0, 0.003831139, - 0.03690933, 0.380781787, - 0.038330593, - 0.01870759, - 0.081877386, 0.216578652, 24.57764134, 0.155690579, - 1.292122433, - 0.686233185, - 0.08729658 };
                std::vector<real_T> Cl_A4{ 1, up, v, abs(u) * v, up * w, p, abs(u) * up * w,abs(u) * up, abs(u) * up * powf(v,3) * w, up * w * w, up * v * v, u * u * v, abs(u) * v * powf(w,3), u * u * up * w, u * u * v * v * w * w, v * powf(w,3) };
                std::vector<real_T> Cl_k4{ 0.0, 0.003698578, - 0.042426851, 0.401511568, - 0.077754952, - 0.109739169, 1.018217491, - 0.026042207, 28.44282457, - 0.147332162, 0.085736664, - 1.162860142, 76.71716783, - 4.475144365, 113.1681119, - 11.33303566 };
                std::vector<real_T> Cl_A5{ 1, up, v, abs(u) * v, p * w, abs(u) * up * v * v * w, abs(u) * up,u * u * up, u * u * v, up * w, abs(u) * up * w, p, u * u * p * powf(v,3), u * u * p * v,u * u * up * w };
                std::vector<real_T> Cl_k5{ 0.0, 0.003832046, - 0.036391628, 0.616310524, 1.116802663, - 7.505676697, - 0.069317923, 0.520343033, - 3.463789744, - 0.074723675, 1.329080005, - 0.091664724, - 8164.362924, 185.5039555, - 5.033054914 };
                std::vector<real_T> Cl_A6{ 1, up, v, up * w, powf(v,3),p,v * powf(w,3), up * powf(w,3),p * v * v * w,up * v * v * w,up * powf(v,4), abs(u) * p * powf(v,3) * w, up * w * w };
                std::vector<real_T> Cl_k6{ 0.0, 0.002898713, - 0.028825354, - 0.045005399, 0.456094626, - 0.090767775, - 10.38829128, 1.138129609, 81.05334563, 0.565426991, - 0.669030578, - 4883.636289, - 0.146600003 };
                std::vector<real_T> Cl_A7{ 1, up, v, up * v * v * w, up * w,powf(v,3), p * w, powf(w,5), up * powf(v,4), p, up * w * w, up * v * v, v * w * w, abs(u) * up };
                std::vector<real_T> Cl_k7{ 0.0, 0.002241872, - 0.025284575, 0.151215819, - 0.042488592, 0.239948797, 1.675597506, - 18.33165687, - 0.781415051, - 0.055684592, - 0.324373595, 0.03190733, 0.916093795, 0.006832651 };

                real_T Cl;
                if (abs(beta) <= 7)
                {
                    Cl = std::inner_product(Cl_A1.begin(), Cl_A1.end(), Cl_k1.begin(), 0.0f);
                }
                else if (abs(beta) <= 22)
                {
                    Cl = std::inner_product(Cl_A2.begin(), Cl_A2.end(), Cl_k2.begin(), 0.0f);
                }
                else if (abs(beta) <= 37)
                {
                    Cl = std::inner_product(Cl_A3.begin(), Cl_A3.end(), Cl_k3.begin(), 0.0f);
                }
                else if (abs(beta) <= 52)
                {
                    Cl = std::inner_product(Cl_A4.begin(), Cl_A4.end(), Cl_k4.begin(), 0.0f);
                }
                else if (abs(beta) <= 67)
                {
                    Cl = std::inner_product(Cl_A5.begin(), Cl_A5.end(), Cl_k5.begin(), 0.0f);
                }
                else if (abs(beta) <= 82)
                {
                    Cl = std::inner_product(Cl_A6.begin(), Cl_A6.end(), Cl_k6.begin(), 0.0f);
                }
                else
                {
                    Cl = std::inner_product(Cl_A7.begin(), Cl_A7.end(), Cl_k7.begin(), 0.0f);
                }

                // Computation of Cm
                std::vector<real_T> Cm_A1{ 1, uq, u, powf(u,3) * w, w, q, uq * w, uq * powf(u,4), uq * u * u * w, q * w, uq * u * powf(w,3), u * u,uq * powf(w,5), u * w * w, uq * u * w * w, abs(v) * uq * u, powf(u,3) };
                std::vector<real_T> Cm_k1{ 0.0, 0.00266147, 0.025678873, 1.373505409, 0.009636309, - 0.078547738, - 0.028361902, 0.259836951, 0.513196762, 1.856906354, 18.22657421, 0.015881604, 133.551928, - 0.867818745, 1.479652708, - 0.048863317, - 0.145223649 };
                std::vector<real_T> Cm_A2{ 1, uq, u, powf(u,3) * w, w * w, abs(v) * uq * u * u, uq * w, abs(v) * uq, abs(v) * q, u * w * w, uq * powf(w,3), q, abs(v) * uq * powf(u,4), uq * u * u * w, powf(u,4) };
                std::vector<real_T> Cm_k2{ -0.0, 0.002675521, 0.022060657, 1.040685239, - 0.011926358, 1.583956091, - 0.041457929, - 0.037306321, - 2.324712328, - 1.180127417, 2.757683785, - 0.062007944, - 9.445916086, 0.580230125, - 0.150905308 };
                std::vector<real_T> Cm_A3{ 1, uq, u, v * v * uq * u * u, u * w * w, abs(v) * q, uq * w, uq * u * u * w * w, abs(v) * uq, v * v * uq * powf(u,4), powf(u,3), q, u * powf(w,3), uq * w * w, uq * u * u * w };
                std::vector<real_T> Cm_k3{ -0.0, 0.002633536, 0.022410594, 6.478889727, - 2.790517113, - 0.566811653, - 0.02955774, - 2.672295468, - 0.013303816, - 57.91586139, - 0.154234318, - 0.056515636, - 12.62832426, - 0.263099322, 0.372808375 };
                std::vector<real_T> Cm_A4{ 1, uq,u,u * w * w, abs(v) * u, abs(v) * u * w * w, uq * powf(u,3), w, powf(w,5), uq * u * u * w, v * v * uq * u * u * w * w, v * v * u * u, q, v * v * uq };
                std::vector<real_T> Cm_k4{ -0.0, 0.002510075, 0.031058213, - 2.700436817, - 0.176128706, 16.8597098, - 0.143482712, 0.00343862, - 28.39551174, 0.84918214, 406.2366553, 0.978462557, - 0.045370194, - 0.022542928 };
                std::vector<real_T> Cm_A5{ 1, uq, v * v * w * w,u,abs(v) * u, u * w * w, abs(v) * w, q, powf(u,3), v * v * uq * powf(u,3) * w, abs(v) * uq * u * powf(w,3), abs(v) * u * w * w,v * v * uq * u * u * w * w, abs(v) * w * w,v * v * q * u * w * w,uq * w };
                std::vector<real_T> Cm_k5{ -0.0, 0.002441645, - 5.735465443, 0.029967615, - 0.147965779, - 4.13216097, 0.044540033, - 0.05199089, 0.29910347, 352.413116, - 79.98030295, 21.11187111, - 447.3527141, 1.182508332, - 4326.453106, - 0.003761333 };
                std::vector<real_T> Cm_A6{ 1, uq, u, abs(v) * u,u * powf(w,3),q,u * u,v * v * u,w,abs(v) * uq };
                std::vector<real_T> Cm_k6{ -0.0, 0.002606, 0.040472401, - 0.315498084, 13.5879934, - 0.058762916, - 0.072309254, 0.695814431, - 0.001240637, - 0.001629784 };
                std::vector<real_T> Cm_A7{ 1, uq, abs(v) * uq, q,u, abs(v) * u,abs(v) * uq * w, powf(u,3) * w,powf(w,5),w * w,u * powf(w,3), v * v * u * u * w, abs(v) * u * w * w, v * v * powf(u,4), uq * w, v * v * uq * u, abs(v) * powf(w,3) };
                std::vector<real_T> Cm_k7{ -0.0, 0.003026891, - 0.006628215, - 0.086984328, 0.055200003, - 0.38058918, 0.084183188, 119.7124245, 43.73033155, 0.042116125, 74.85251484, 559.8095797, 43.9864018, 63709.49547, - 0.019657891, 0.793266118, - 2.218099587 };


                real_T Cm;
                if (abs(beta) <= 7)
                {
                    Cm = std::inner_product(Cm_A1.begin(), Cm_A1.end(), Cm_k1.begin(), 0.0f);
                }
                else if (abs(beta) <= 22)
                {
                    Cm = std::inner_product(Cm_A2.begin(), Cm_A2.end(), Cm_k2.begin(), 0.0f);
                }
                else if (abs(beta) <= 37)
                {
                    Cm = std::inner_product(Cm_A3.begin(), Cm_A3.end(), Cm_k3.begin(), 0.0f);
                }
                else if (abs(beta) <= 52)
                {
                    Cm = std::inner_product(Cm_A4.begin(), Cm_A4.end(), Cm_k4.begin(), 0.0f);
                }
                else if (abs(beta) <= 67)
                {
                    Cm = std::inner_product(Cm_A5.begin(), Cm_A5.end(), Cm_k5.begin(), 0.0f);
                }
                else if (abs(beta) <= 82)
                {
                    Cm = std::inner_product(Cm_A6.begin(), Cm_A6.end(), Cm_k6.begin(), 0.0f);
                }
                else
                {
                    Cm = std::inner_product(Cm_A7.begin(), Cm_A7.end(), Cm_k7.begin(), 0.0);
                }

                // Computations for Cn
                std::vector<real_T> Cn_A1{ 1, ur, ur * v, ur * v * v * sgn(v), powf(w,4),ur * u,w * r,v * r, powf(u,3), r * r * sgn(r), v * w * w, v * v * r * sgn(v), ur * ur * sgn(ur) * powf(u,3) };
                std::vector<real_T> Cn_k1{ 0, 0.000265351, - 0.003643848, 0.038228277, - 0.699769849, - 0.000632547, 0.336752164, 0.90661348, - 0.006698097, - 1.196236479, - 0.112512023, - 8.192480099, 0.003612272 };
                std::vector<real_T> Cn_A2{ 1, ur * ur * sgn(ur), u * v * v * sgn(v),powf(ur,3), u * u,w,r,ur * u,ur,ur * ur * sgn(ur) * powf(u,3),ur * v,ur * ur * sgn(ur) * v * v * sgn(v),ur * u * w * w };
                std::vector<real_T> Cn_k2{ 0.0, 6.55936e-05, - 0.106159568, - 1.21962E-05, 0.009126575, 0.001309498, - 0.0203606, - 0.000700018, 0.000222686, 0.004271037, - 0.000805437, 0.00258052, 0.0528163 };
                std::vector<real_T> Cn_A3{ 1, ur * ur * sgn(ur), u * v * w,powf(ur,3), u * w * w, v, ur * u * u,r * r * sgn(r), v * v * sgn(v), ur, u * v * v * sgn(v), ur * r, ur * powf(v,5) };
                std::vector<real_T> Cn_k3{ 0, 0.000158623, 0.321133087, - 3.18319E-05, 0.264495856, - 0.002012636, - 0.009774605, - 1.957811095, 0.010544897, 0.000148026, - 0.05315564, 0.002985985, - 0.221403103 };
                std::vector<real_T> Cn_A4{ 1, ur * ur * sgn(ur), powf(ur,3), r, v, ur * ur * sgn(ur) * powf(u,3) };
                std::vector<real_T> Cn_k4{ 0.0, 0.00027625, - 0.00005674, - 0.01999979, - 0.00202414, - 0.94889962 };

                real_T Cn;
                if (sqrt(u * u + v * v + w * w) <= 0.05)
                {
                    Cn = std::inner_product(Cn_A4.begin(), Cn_A4.end(), Cn_k4.begin(), 0.0);
                }
                else if (abs(beta) <= 30)
                {
                    Cn = std::inner_product(Cn_A1.begin(), Cn_A1.end(), Cn_k1.begin(), 0.0);
                }
                else if (abs(beta) <= 60)
                {
                    Cn = std::inner_product(Cn_A2.begin(), Cn_A2.end(), Cn_k2.begin(), 0.0);
                }
                else
                {
                    Cn = std::inner_product(Cn_A3.begin(), Cn_A3.end(), Cn_k3.begin(), 0.0);
                }

                return retVals_moments{ Cl, Cm, Cn };
            }

            struct retVals_states {        // Declare a local structure 
                real_T ret_u, ret_v, ret_w, ret_p, ret_q, ret_r;
            };

            struct retVals_forces {        // Declare a local structure 
                real_T Ct, Cx, Cy;
            };

            struct retVals_moments {        // Declare a local structure 
                real_T Cl, Cm, Cn;
            };

            struct retVals_blade_damage_forces_moments {        // Declare a local structure 
                Vector3r F, M;
            };

            /// <summary>
            /// Create Propeller, blades, blade sections and compute CG location
            /// </summary>
            /// <param name="damage_coeffs_advanced">vector with the damage coefficients suffered by the blades</param>
            /// <param name="damage_start_angles_advanced">vector with the initial angle of the propellers</param>
            void createPropellerModel(std::vector<real_T> damage_coeffs_advanced, std::vector<real_T> damage_start_angles_advanced) {
                int counter = 0;
                for (DamagedPropeller& propeller : damaged_propellers_)
                {
                    std::vector<real_T> damage_coeffs_advanced_local = { damage_coeffs_advanced[counter * 3], damage_coeffs_advanced[counter * 3 + 1], damage_coeffs_advanced[counter * 3 + 2] };
                    propeller.resetPropeller(damage_coeffs_advanced_local, damage_start_angles_advanced[counter]);
                    counter++;

                    propeller.createBlades();
                    propeller.computeBladesParams();
                    propeller.computeCgLocation();
                }
            }

            /// <summary>
            /// Function that computes the kinematics without any collision
            /// When used, the drone properties in SimpleFlightQuadXParams.hpp have to be changed to setupFrameBB2Leon.
            /// </summary>
            /// <param name="dt">time step</param>
            /// <param name="body">body object</param>
            /// <param name="current">current time step kinematic states</param>
            /// <param name="next">next time step kinematic states</param>
            /// <param name="next_wrench">next time step forces and moments</param>
            /// <param name="wind">windspeed</param>
            /// <param name="prop_damage"></param>
            void getNextKinematicsNoCollisionFM_BB2_dml_6DOF(TTimeDelta dt, PhysicsBody& body, const Kinematics::State& current,
                Kinematics::State& next, Wrench& next_wrench, const Vector3r& wind, float prop_damage[])
            {
                const real_T dt_real = static_cast<real_T>(dt);
                const real_T current_time = static_cast<real_T>((clock()->nowNanos() - clock()->getStart()) / 1.0E9);
                Vector3r avg_linear;
                Vector3r avg_angular;
                if (use_average_values)
                {
                    avg_linear = current.twist.linear + current.accelerations.linear * (0.5f * dt_real);
                    avg_angular = current.twist.angular + current.accelerations.angular * (0.5f * dt_real);
                }
                else
                {
                    avg_linear = current.twist.linear;
                    avg_angular = current.twist.angular;
                }
                // Transform to the body frame the relative velocity of the body with respect to the wind
                const Vector3r relative_vel = avg_linear - wind;
                const Vector3r linear_vel_body = VectorMath::transformToBodyFrame(relative_vel, current.pose.orientation);

                std::vector<real_T> PWMs = body.getPWMrotors_INDI(previous, dt_real, current_time, mass_F_, mass_M_, aero_F_, aero_M_);

                real_T omega1 = PWMs[0];
                if (omega1 <= 1) { omega1 = 0.0f; }
                real_T omega2 = PWMs[1];
                if (omega2 <= 1) { omega2 = 0.0f; }
                real_T omega3 = PWMs[2];
                if (omega3 <= 1) { omega3 = 0.0f; }
                real_T omega4 = PWMs[3];
                if (omega4 <= 1) { omega4 = 0.0f; }

                bool teleport_reset = body.getSwitchTeleportReset();
                if (teleport_reset)
                {
                    omega1_last = omega1;
                    omega2_last = omega2;
                    omega3_last = omega3;
                    omega4_last = omega4;
                    previous.accelerations.linear = Vector3r::Zero();
                    previous.accelerations.angular = Vector3r::Zero();
                }

                real_T N = 4.;
                real_T Omega = sqrt((omega1 * omega1 + omega2 * omega2 + omega3 * omega3 + omega4 * omega4) / N);
                real_T small = 0.001;

                // Compute the sideslip angle
                real_T beta = 0.0;
                if (abs(linear_vel_body.x()) > small)
                {
                    beta = atan(linear_vel_body.y() / linear_vel_body.x()) * rad2deg_factor;
                }

                // Correct body mass and inertia
                real_T n_broken_prop = 0;
                for (int i = 0; i < 4; i++)
                {
                    if (PWMs[i] < 1)
                    {
                        n_broken_prop++;
                    }
                }

                // Computation of angular rates
                real_T p = avg_angular.x() / (Omega * R) * b;
                real_T q = avg_angular.y() / (Omega * R) * b;
                real_T r = avg_angular.z() / (Omega * R) * b;

                // Computation of velocities
                real_T u = linear_vel_body.x() / (Omega * R);
                real_T v = linear_vel_body.y() / (Omega * R);
                real_T w = linear_vel_body.z() / (Omega * R);

                // Computation of propeller rotations
                real_T w1 = omega1 / Omega;
                real_T w2 = omega2 / Omega;
                real_T w3 = omega3 / Omega;
                real_T w4 = omega4 / Omega;

                // Computation of the up, uq and ur
                real_T up = (w1 * w1 + w4 * w4) - (w2 * w2 + w3 * w3);
                real_T uq = (w1 * w1 + w2 * w2) - (w3 * w3 + w4 * w4);
                real_T ur = signr * (-(w1 * w1 + w3 * w3) + (w2 * w2 + w4 * w4));

                retVals_states stateHedging_output = stateHedging(u, v, w, p, q, r);
                u = stateHedging_output.ret_u;
                v = stateHedging_output.ret_v;
                w = stateHedging_output.ret_w;
                p = stateHedging_output.ret_p;
                q = stateHedging_output.ret_q;
                r = stateHedging_output.ret_r;

                Eigen::Matrix<real_T, 15, 15> vi_table;
                std::vector<real_T> vh_table{ 0, 0.0500000000000000, 0.100000000000000, 0.150000000000000, 0.200000000000000, 0.250000000000000, 0.300000000000000, 0.350000000000000, 0.400000000000000, 0.450000000000000, 0.500000000000000, 0.550000000000000, 0.600000000000000, 0.650000000000000, 0.700000000000000 };
                vi_table << 0.146987035160297, 0.158204146875874, 0.170780305616963, 0.184879017223881, 0.200665937201379, 0.218300197246648, 0.237923609658487, 0.259648913287316, 0.283548942047712, 0.309648909048856, 0.337923602135004, 0.368300187958721, 0.400665927700428, 0.434879008678887, 0.470780298652877,
                    0.146504842384115, 0.157622610557716, 0.170076582631260, 0.184025493963285, 0.199629787484227, 0.217043220345493, 0.236402483355238, 0.257815954194761, 0.281353370963765, 0.307038492786579, 0.334846432065061, 0.364706310834439, 0.396508588743108, 0.430115377809710, 0.465371676738114,
                    0.145079993458777, 0.155906189790108, 0.168001811926414, 0.181511506728932, 0.196580100818639, 0.213344837094679, 0.231925939333534, 0.252416508677674, 0.274873292176612, 0.299310064272469, 0.325695013406464, 0.353952647885545, 0.383969632696544, 0.415603075206154, 0.448689392230917,
                    0.142775673968261, 0.153136840089212, 0.164661926617426, 0.177473003664349, 0.191689291928966, 0.207419864964108, 0.224754917401100, 0.243756378696087, 0.264448996004744, 0.286813049780671, 0.310779427521163, 0.336226827959540, 0.362979481350613, 0.390801913448715, 0.419384229753540,
                    0.139690725835992, 0.149442651580260, 0.160222672351479, 0.172123566859853, 0.185230563305921, 0.199613862242770, 0.215319580445153, 0.232359437090227, 0.250699493878149, 0.270247800054342, 0.290839513216365, 0.312215304365994, 0.333982888201957, 0.355536775180731, 0.375866965533774,
                    0.135950699325350, 0.144985881937342, 0.154893903828498, 0.165734418377404, 0.177553654704617, 0.190376076953357, 0.204193883177509, 0.218953957528994, 0.234541328435212, 0.250756767041075, 0.267282967639543, 0.283626607928231, 0.299006489289727, 0.312112466897927, 0.320521913440298,
                    0.131696868542241, 0.139948032283867, 0.148909408029834, 0.158607355948743, 0.169048964502274, 0.180212710633800, 0.192036342999775, 0.204401012270159, 0.217109760793472, 0.229856610650397, 0.242178844061418, 0.253378326417085, 0.262386836060561, 0.267543617855642, 0.266328808269273,
                    0.127074691394025, 0.134514033485537, 0.142505096173009, 0.151044423574286, 0.160104887599887, 0.169626004245173, 0.179501400147647, 0.189562468532949, 0.199556667632121, 0.209118094564439, 0.217727385279064, 0.224659878362214, 0.228933452853986, 0.229314196319866, 0.224556768278600,
                    0.122223294789725, 0.128857918866929, 0.135899225995722, 0.143320126943085, 0.151067877623877, 0.159055165103748, 0.167149025649078, 0.175157256627325, 0.182812151730364, 0.189752168264221, 0.195504664642156, 0.199479639184475, 0.200999379258284, 0.199411538850900, 0.194333690612593,
                    0.117267288924200, 0.123132000496871, 0.129277907677429, 0.135661682645414, 0.142214919085282, 0.148836957676042, 0.155386663357397, 0.161673557768059, 0.167449390484762, 0.172402668107304, 0.176161431416922, 0.178314048956112, 0.178462380913232, 0.176318790997450, 0.171833013937011,
                    0.112311624024273, 0.117460609563514, 0.122787549362592, 0.128240020630196, 0.133743080793777, 0.139194264340461, 0.144458592811865, 0.149364432500810, 0.153701738029435, 0.157225314357105, 0.159667135751302, 0.160762759813366, 0.160295397119373, 0.158153236700682, 0.154385547923481,
                    0.107439512494979, 0.111938312404429, 0.116533916612725, 0.121170706483662, 0.125774195746672, 0.130248083450804, 0.134472017232031, 0.138300951022569, 0.141567433043785, 0.144088642528183, 0.145680221580019, 0.146178289295676, 0.145469022503359, 0.143516592467871, 0.140389743197411,
                    0.102712886310116, 0.106631632863506, 0.110586042534577, 0.114521553245617, 0.118368661442722, 0.122041592979975, 0.125437980378142, 0.128440253160596, 0.130919632951462, 0.132743678736223, 0.133788033752336, 0.133952670624337, 0.133176598533810, 0.131458903127007, 0.128860505448582,
                    0.0981745789362859, 0.101582936540136, 0.104982752890018, 0.108323188135527, 0.111542103199643, 0.114565828892516, 0.117309950537986, 0.119681578679631, 0.121583590817868, 0.122921195543032, 0.123611269379111, 0.123590435263492, 0.122829667071355, 0.121338083929863, 0.119166523856044,
                    0.0938514088277625, 0.0968152160445956, 0.0997398879586158, 0.102579692736340, 0.105280677655034, 0.107781084927768, 0.110012643559119, 0.111903006596023, 0.113379541582707, 0.114374877069709, 0.114831492662748, 0.114711485636836, 0.114000012599906, 0.112709816053747, 0.110881666709003;
                std::vector<real_T> w_table{ -0.400000000000000, -0.350000000000000, -0.300000000000000, -0.250000000000000, -0.200000000000000, -0.150000000000000, -0.100000000000000, -0.0500000000000000, -5.55111512312578e-17, 0.0500000000000000, 0.100000000000000, 0.150000000000000, 0.200000000000000, 0.250000000000000, 0.300000000000000 };

                real_T vi = interp2(vh_table, w_table, vi_table, sqrt(u * u + v * v), w);

                retVals_forces force_dimensionless_BB2_output = force_dimensionless_BB2(beta, up, uq, ur, u, v, w, p, q, r, vi);
                retVals_moments moment_dimensionless_BB2_output = moment_dimensionless_BB2(beta, up, uq, ur, u, v, w, p, q, r);


                real_T Ct = force_dimensionless_BB2_output.Ct;
                real_T Cx = force_dimensionless_BB2_output.Cx;
                real_T Cy = force_dimensionless_BB2_output.Cy;
                real_T Cl = moment_dimensionless_BB2_output.Cl;
                real_T Cm = moment_dimensionless_BB2_output.Cm;
                real_T Cn = moment_dimensionless_BB2_output.Cn;

                real_T T = rho * Ct * Omega * Omega * (N * M_PIf * R * R) * R * R;
                real_T Fx = rho * Cx * Omega * Omega * (N * M_PIf * R * R) * R * R;
                real_T Fy = rho * Cy * Omega * Omega * (N * M_PIf * R * R) * R * R;
                real_T Mx = rho * Cl * Omega * Omega * (N * M_PIf * R * R) * R * R * b;
                real_T My = rho * Cm * Omega * Omega * (N * M_PIf * R * R) * R * R * b;
                real_T Mz = rho * Cn * Omega * Omega * (N * M_PIf * R * R) * R * R * b;

                // Obtain propeller damage information
                bool switch_damage_prop_params = body.getSwitchDamagePropParams();
                if (switch_damage_prop_params)
                {
                    std::vector<real_T> damage_coeffs_advanced = body.getDamagePropParams();
                    std::vector<real_T> damage_start_angles_advanced = body.getDamagePropStartAngles();
                    createPropellerModel(damage_coeffs_advanced, damage_start_angles_advanced);
                }

                // Add the forces and moments caused by the damaged to the healthy propeller wrench
                bool switch_activate_blade_damage = body.getSwitchActivateBladeDamage();
                Vector3r delta_F = Vector3r::Zero();
                Vector3r delta_M = Vector3r::Zero();
                real_T lost_propeller_mass = 0.0f;
                if (switch_activate_blade_damage)
                {
                    std::vector<real_T> omegas{ omega1, omega2, omega3, omega4 };
                    int counter = 0;
                    mass_F_ = Vector3r::Zero();
                    mass_M_ = Vector3r::Zero();
                    aero_F_ = Vector3r::Zero();
                    aero_M_ = Vector3r::Zero();
                    std::vector<int> SL{ 1, -1, -1, 1 };
                    std::vector<int> SM{ 1, 1, -1, -1 };
                    std::vector<int> SN{ signr * 1, signr * -1, signr * 1, signr * -1 };
                    for (DamagedPropeller& propeller : damaged_propellers_)
                    {
                        auto delta_FM = propeller.computeMassAeroFM(n_blade_segment_, omegas[counter], current.pose.orientation, cla_coeffs_, cda_coeffs_, avg_linear, avg_angular, rho);
                        delta_F += delta_FM.delta_F;
                        delta_M += delta_FM.delta_M;

                        Vector3r forces2moments = Vector3r(-SL[counter] * b * delta_FM.delta_F.z(), -SM[counter] * l * delta_FM.delta_F.z(),
                            b * SL[counter] * delta_FM.delta_F.x() + l * SM[counter] * delta_FM.delta_F.y());

                        delta_M += forces2moments;

                        mass_F_ += delta_FM.mass_F;
                        mass_M_ += delta_FM.mass_M;
                        aero_F_ += delta_FM.aero_F;
                        aero_M_ += delta_FM.aero_M;
                        lost_propeller_mass += propeller.get_propeller_lost_mass();
                        // Rotate propeller for the next computation
                        propeller.updateRotationAngle(omegas[counter], dt_real);
                        counter++;
                    }
                }

                // Obtain the force and moment vector
                body.choose_inertia(n_broken_prop, b, l, lost_propeller_mass);
                Wrench wrench = Wrench::zero();
                Vector3r local_force = Vector3r(Fx, Fy, -T) + delta_F;
                Vector3r local_torque = Vector3r(Mx, My, Mz) + delta_M;
                wrench.force = local_force;
                wrench.torque = local_torque;

                if (isnan(Fx) || isnan(Fy) || isnan(T) || isnan(Mx) || isnan(My) || isnan(Mz))
                {
                    int a = 1;
                }

                //convert force to world frame, leave torque to local frame
                wrench.force = VectorMath::transformToWorldFrame(wrench.force, current.pose.orientation);

                if (body.isGrounded()) {
                    // make it stick to the ground until the magnitude of net external force on body exceeds its weight.
                    float external_force_magnitude = wrench.force.squaredNorm();
                    Vector3r weight = body.getMass() * body.getEnvironment().getState().gravity;
                    float weight_magnitude = weight.squaredNorm();
                    if (external_force_magnitude >= weight_magnitude)
                    {
                        body.setGrounded(false);
                    }
                    next_wrench.force = Vector3r::Zero();
                    next_wrench.torque = Vector3r::Zero();
                    next.accelerations.linear = Vector3r::Zero();
                }
                else {
                    next_wrench = wrench;

                    /************************* Update accelerations due to force and torque ************************/
                    //get new acceleration due to force - we'll use this acceleration in next time step
                    // Compute the cross product of the angular velocities and the body linear velocities in the body frame.
                    // Transform to the body frame the linear velocity of the body
                    const Vector3r Vb = VectorMath::transformToBodyFrame(avg_linear, current.pose.orientation);
                    Vector3r Vb_pqr_cross = avg_angular.cross(Vb);

                    // Convert back to WorldFrame format
                    Vb_pqr_cross = VectorMath::transformToWorldFrame(Vb_pqr_cross, current.pose.orientation);

                    // Compute the next linear accelerations
                    next.accelerations.linear = apply_process_noise((next_wrench.force / body.getMass()) + body.getEnvironment().getState().gravity - Vb_pqr_cross, mean_vxyz, cov_vxyz);

                }

                if (body.isGrounded()) {
                    // this stops vehicle from vibrating while it is on the ground doing nothing.
                    next.accelerations.angular = Vector3r::Zero();
                    next.twist.linear = Vector3r::Zero();
                    next.twist.angular = Vector3r::Zero();
                }
                else {
                    //get new angular acceleration
                    //Euler's rotation equation: https://en.wikipedia.org/wiki/Euler's_equations_(body_dynamics)
                    //we will use torque to find out the angular acceleration
                    //angular momentum L = I * omega
                    const Vector3r angular_momentum = body.getInertia() * avg_angular;

                    // The gyroscopic moment has to be computed
                    // First the propeller Moment of Inertia is established
                    Matrix3x3r Ip = Matrix3x3r::Zero();
                    Ip(0, 0) = 0.0f; Ip(0, 1) = 0.0f; Ip(0, 2) = 0.0f;
                    Ip(1, 0) = 0.0f; Ip(1, 1) = 0.0f; Ip(1, 2) = 0.0f;
                    Ip(2, 0) = 0.0f; Ip(2, 1) = 0.0f; Ip(2, 2) = 7.095e-06;

                    // Then the gyroscopic part of the moment
                    const Vector3r w1_vector = Vector3r(0.0f, 0.0f, omega1);
                    const Vector3r w2_vector = Vector3r(0.0f, 0.0f, -omega2);
                    const Vector3r w3_vector = Vector3r(0.0f, 0.0f, omega3);
                    const Vector3r w4_vector = Vector3r(0.0f, 0.0f, -omega4);
                    const Vector3r M_gyro_second = avg_angular.cross(Ip * w1_vector + Ip * w2_vector + Ip * w3_vector + Ip * w4_vector);

                    real_T omega1_dot = (omega1 - omega1_last) / dt_real;
                    real_T omega2_dot = (omega2 - omega2_last) / dt_real;
                    real_T omega3_dot = (omega3 - omega3_last) / dt_real;
                    real_T omega4_dot = (omega4 - omega4_last) / dt_real;
                    const Vector3r w1_dot_vector = Vector3r(0.0f, 0.0f, omega1_dot);
                    const Vector3r w2_dot_vector = Vector3r(0.0f, 0.0f, -omega2_dot);
                    const Vector3r w3_dot_vector = Vector3r(0.0f, 0.0f, omega3_dot);
                    const Vector3r w4_dot_vector = Vector3r(0.0f, 0.0f, -omega4_dot);
                    const Vector3r M_gyro_first = Ip * w1_dot_vector + Ip * w2_dot_vector + Ip * w3_dot_vector + Ip * w4_dot_vector;

                    omega1_last = omega1;
                    omega2_last = omega2;
                    omega3_last = omega3;
                    omega4_last = omega4;

                    // It is strange that the change in omega in a time step is constant independently of the time-step size. As a result, the rest of the contributions to the angular acceleration are minuscule compared to the gyroscopic effect.
                    Vector3r M_gyro = signr * (M_gyro_first + M_gyro_second);
                    if (dummy_trial)
                    {
                        M_gyro = signr * (M_gyro_second);
                    }
                    const Vector3r angular_momentum_rate = next_wrench.torque - M_gyro - avg_angular.cross(angular_momentum);
                    //new angular acceleration - we'll use this acceleration in next time step
                    next.accelerations.angular = apply_process_noise(body.getInertiaInv() * angular_momentum_rate, mean_pqr, cov_pqr);

                    /************************* Update pose and twist after dt ************************/
                    //Verlet integration: http://www.physics.udel.edu/~bnikolic/teaching/phys660/numerical_ode/node5.html
                    // ode2 Heuns method
                    if (integration_method_v == 0)  // Verlet algorithm
                    {
                        next.twist.linear = apply_process_noise(current.twist.linear + (current.accelerations.linear + next.accelerations.linear) * (0.5f * dt_real), mean_xyz, cov_xyz);
                        next.twist.angular = apply_process_noise(current.twist.angular + (current.accelerations.angular + next.accelerations.angular) * (0.5f * dt_real), mean_att, cov_att);
                    }
                    else if (integration_method_v == 1)  // Verlet algorithm AirSim
                    {
                        next.twist.linear = apply_process_noise(current.twist.linear + next.accelerations.linear * dt_real + (current.accelerations.linear) * (0.5f * dt_real), mean_xyz, cov_xyz);
                        next.twist.angular = apply_process_noise(current.twist.angular + next.accelerations.angular * dt_real + (current.accelerations.angular) * (0.5f * dt_real), mean_att, cov_att);
                    }
                    else if (integration_method_v == 2)  // Adams-Bashfort 2-step method
                    {
                        next.twist.linear = apply_process_noise(current.twist.linear + 3 / 2.0f * next.accelerations.linear * dt_real - current.accelerations.linear * (0.5f * dt_real), mean_xyz, cov_xyz);
                        next.twist.angular = apply_process_noise(current.twist.angular + 3 / 2.0f * next.accelerations.angular * dt_real - current.accelerations.angular * (0.5f * dt_real), mean_att, cov_att);
                    }
                    else if (integration_method_v == 3)  // Beeman and Schofield
                    {
                        next.twist.linear = apply_process_noise(current.twist.linear + 1 / 6.0f * (2 * next.accelerations.linear + 5 * current.accelerations.linear - previous.accelerations.linear) * dt_real, mean_xyz, cov_xyz);
                        next.twist.angular = apply_process_noise(current.twist.angular + 1 / 6.0f * (2 * next.accelerations.angular + 5 * current.accelerations.angular - previous.accelerations.angular) * dt_real, mean_att, cov_att);
                    }

                    real_T vx_print = abs(next.twist.linear.x());
                    real_T vy_print = abs(next.twist.linear.y());
                    real_T vz_print = abs(next.twist.linear.z());
                    real_T p_print = abs(next.twist.angular.x());
                    real_T q_print = abs(next.twist.angular.y());
                    real_T r_print = abs(next.twist.angular.z());

                    //if controller has bug, velocities can increase idenfinitely 
                    //so we need to clip this or everything will turn in to infinity/nans

                    if (next.twist.linear.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                        next.twist.linear /= (next.twist.linear.norm() / EarthUtils::SpeedOfLight);
                        next.accelerations.linear = Vector3r::Zero();
                    }
                    //
                    //for disc of 1m radius which angular velocity translates to speed of light on tangent?
                    if (next.twist.angular.squaredNorm() > EarthUtils::SpeedOfLight * EarthUtils::SpeedOfLight) { //speed of light
                        next.twist.angular /= (next.twist.angular.norm() / EarthUtils::SpeedOfLight);
                        next.accelerations.angular = Vector3r::Zero();
                    }
                }
                real_T x_location = current.pose.position.x();
                real_T z_location = current.pose.position.z();
                computeNextPose(dt, current, next);

            };

            /// <summary>
            /// Function that translates the current linear and angular velocities and accelerations into the next step
            /// velocities, position and vehicle attitude.
            /// </summary>
            /// <param name="dt">time step</param>
            /// <param name="current">the current time step vehicle states</param>
            /// <param name="next">the next time step vehicle states</param>
            void computeNextPose(TTimeDelta dt, const Kinematics::State& current, Kinematics::State& next)
            {
                real_T dt_real = static_cast<real_T>(dt);

                if (!use_quat)
                {
                    real_T p_var = current.twist.angular(0);
                    real_T q_var = current.twist.angular(1);
                    real_T r_var = current.twist.angular(2);
                    real_T p_dot_var = current.accelerations.angular(0);
                    real_T q_dot_var = current.accelerations.angular(1);
                    real_T r_dot_var = current.accelerations.angular(2);

                    real_T theta, phi, psi;
                    VectorMath::toEulerianAngle(current.pose.orientation, theta, phi, psi);

                    // Corrected current angular velocity
                    real_T phi_update_v = p_var + tan(theta) * (q_var * sin(phi) + r_var * cos(phi));
                    real_T theta_update_v = q_var * cos(phi) - r_var * sin(phi);
                    real_T psi_update_v = (q_var * sin(phi) + r_var * cos(phi)) / cos(theta);

                    // Corrected current angular acceleration
                    real_T phi_update_a = p_dot_var + tan(theta) * (q_dot_var * sin(phi) + r_dot_var * cos(phi));
                    real_T theta_update_a = q_dot_var * cos(phi) - r_dot_var * sin(phi);
                    real_T psi_update_a = (q_dot_var * sin(phi) + r_dot_var * cos(phi)) / cos(theta);

                    real_T phi_update;
                    real_T theta_update;
                    real_T psi_update;

                    if (integration_method_x == 0)   // Verlet algorithm
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real + 1 / 2.0f * current.accelerations.linear * dt_real * dt_real;
                        phi_update = phi + phi_update_v * dt_real + 1 / 2.0f * phi_update_a * dt_real * dt_real;
                        theta_update = theta + theta_update_v * dt_real + 1 / 2.0f * theta_update_a * dt_real * dt_real;
                        psi_update = psi + psi_update_v * dt_real + 1 / 2.0f * psi_update_a * dt_real * dt_real;
                    }
                    else if (integration_method_x == 1)  // Verlet algorithm AirSim
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real;
                        phi_update = phi + phi_update_v * dt_real;
                        theta_update = theta + theta_update_v * dt_real;
                        psi_update = psi + psi_update_v * dt_real;
                    }
                    else if (integration_method_x == 2)  // Adams-Bashfort 2-step method
                    {
                        next.pose.position = current.pose.position + 3 / 2.0f * next.twist.linear * dt_real - 1 / 2.0f * current.twist.linear * dt_real;

                        real_T p_var_next = next.twist.angular(0);
                        real_T q_var_next = next.twist.angular(1);
                        real_T r_var_next = next.twist.angular(2);

                        // Corrected next angular velocity (incorrect since it uses the current angle)
                        real_T phi_update_v_next = p_var_next + tan(theta) * (q_var_next * sin(phi) + r_var_next * cos(phi));
                        real_T theta_update_v_next = q_var_next * cos(phi) - r_var_next * sin(phi);
                        real_T psi_update_v_next = (q_var_next * sin(phi) + r_var_next * cos(phi)) / cos(theta);

                        phi_update = phi + 3 / 2.0f * phi_update_v_next * dt_real - 1 / 2.0f * phi_update_v * dt_real;
                        theta_update = theta + 3 / 2.0f * theta_update_v_next * dt_real - 1 / 2.0f * theta_update_v * dt_real;
                        psi_update = psi + 3 / 2.0f * psi_update_v_next * dt_real - 1 / 2.0f * psi_update_v * dt_real;
                    }
                    else if (integration_method_x == 3)  // Beeman and Schofield
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real + 1 / 6.0f * (4. * current.accelerations.linear - previous.accelerations.linear) * dt_real * dt_real;

                        real_T p_var_a_previous = previous.accelerations.angular(0);
                        real_T q_var_a_previous = previous.accelerations.angular(1);
                        real_T r_var_a_previous = previous.accelerations.angular(2);

                        real_T theta_previous, phi_previous, psi_previous;
                        VectorMath::toEulerianAngle(previous.pose.orientation, theta_previous, phi_previous, psi_previous);

                        // Corrected previous angular velocity
                        real_T phi_update_a_previous = p_var_a_previous + tan(theta_previous) * (q_var_a_previous * sin(phi_previous) + r_var_a_previous * cos(phi_previous));
                        real_T theta_update_a_previous = q_var_a_previous * cos(phi_previous) - r_var_a_previous * sin(phi_previous);
                        real_T psi_update_a_previous = (q_var_a_previous * sin(phi_previous) + r_var_a_previous * cos(phi_previous)) / cos(theta_previous);

                        phi_update = phi + phi_update_v * dt_real + 1 / 6.0f * (4. * phi_update_a - phi_update_a_previous) * dt_real * dt_real;
                        theta_update = theta + theta_update_v * dt_real + 1 / 6.0f * (4. * theta_update_a - theta_update_a_previous) * dt_real * dt_real;
                        psi_update = psi + psi_update_v * dt_real + 1 / 6.0f * (4. * psi_update_a - psi_update_a_previous) * dt_real * dt_real;
                    }
                    else
                    {
                        throw std::runtime_error("The selected integration method does not exist.");
                    }

                    next.pose.orientation = VectorMath::toQuaternion(theta_update, phi_update, psi_update);
                    if (VectorMath::hasNan(next.pose.orientation)) {
                        //Utils::DebugBreak();
                        Utils::log("orientation had NaN!", Utils::kLogLevelError);
                    }

                    //re-normalize quaternion to avoid accumulating error
                    next.pose.orientation.normalize();
                }
                else
                {
                    //use angular velocty in body frame to calculate angular displacement in last dt seconds
                    real_T angle_per_unit = current.twist.angular.norm();
                    next.pose.orientation = current.pose.orientation;
                    if (integration_method_x == 0)   // Verlet algorithm
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real + 1 / 2.0f * current.accelerations.linear * dt_real * dt_real;
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f))
                        {
                            //convert change in angle to unit quaternion
                            AngleAxisr angle_dt_aa_1 = AngleAxisr(angle_per_unit * dt_real, current.twist.angular / angle_per_unit);
                            Quaternionr angle_dt_q_1 = Quaternionr(angle_dt_aa_1);
                            next.pose.orientation *= angle_dt_q_1;
                        }

                        real_T angle_per_unit_current_acc = current.accelerations.angular.norm();
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit_current_acc, 0.0f))
                        {
                            AngleAxisr angle_dt_aa_2 = AngleAxisr(0.5f * angle_per_unit_current_acc * dt_real * dt_real, current.accelerations.angular / angle_per_unit_current_acc);
                            Quaternionr angle_dt_q_2 = Quaternionr(angle_dt_aa_2);
                            next.pose.orientation *= angle_dt_q_2;
                        }                            
                    }
                    else if (integration_method_x == 1)  // Verlet algorithm AirSim
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real;
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f))
                        {
                            //convert change in angle to unit quaternion
                            AngleAxisr angle_dt_aa = AngleAxisr(angle_per_unit * dt_real, current.twist.angular / angle_per_unit);
                            Quaternionr angle_dt_q = Quaternionr(angle_dt_aa);
                            /*
                            Add change in angle to previous orientation.
                            Proof that this is q0 * q1:
                            If rotated vector is qx*v*qx' then qx is attitude
                            Initially we have q0*v*q0'
                            Lets transform this to body coordinates to get
                            q0'*(q0*v*q0')*q0
                            Then apply q1 rotation on it to get
                            q1(q0'*(q0*v*q0')*q0)q1'
                            Then transform back to world coordinate
                            q0(q1(q0'*(q0*v*q0')*q0)q1')q0'
                            which simplifies to
                            q0(q1(v)q1')q0'
                            Thus new attitude is q0q1
                            */
                            next.pose.orientation *= angle_dt_q;
                        }
                    }
                    else  if (integration_method_x == 2)  // Adams-Bashfort 2-step method
                    {
                        next.pose.position = current.pose.position + 3 / 2.0f * next.twist.linear * dt_real - 1 / 2.0f * current.twist.linear * dt_real;
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f))
                        {
                            //convert change in angle to unit quaternion
                            AngleAxisr angle_dt_aa_1 = AngleAxisr(-1 / 2.0f * angle_per_unit * dt_real, current.twist.angular / angle_per_unit);
                            Quaternionr angle_dt_q_1 = Quaternionr(angle_dt_aa_1);
                            next.pose.orientation *= angle_dt_q_1;
                        }

                        real_T angle_per_unit_next_v = next.twist.angular.norm();
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit_next_v, 0.0f))
                        {
                            AngleAxisr angle_dt_aa_2 = AngleAxisr(3 / 2.0f * angle_per_unit_next_v * dt_real, next.twist.angular / angle_per_unit_next_v);
                            Quaternionr angle_dt_q_2 = Quaternionr(angle_dt_aa_2);
                            next.pose.orientation *= angle_dt_q_2;
                        }
                    }
                    else  if (integration_method_x == 3)  // Beeman and Schofield
                    {
                        next.pose.position = current.pose.position + current.twist.linear * dt_real + 1 / 6.0f * (4. * current.accelerations.linear - previous.accelerations.linear) * dt_real * dt_real;
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit, 0.0f))
                        {
                            //convert change in angle to unit quaternion
                            AngleAxisr angle_dt_aa_1 = AngleAxisr(angle_per_unit * dt_real, current.twist.angular / angle_per_unit);
                            Quaternionr angle_dt_q_1 = Quaternionr(angle_dt_aa_1);
                            next.pose.orientation *= angle_dt_q_1;
                        }

                        real_T angle_per_unit_current_acc = current.accelerations.angular.norm();
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit_current_acc, 0.0f))
                        {
                            AngleAxisr angle_dt_aa_2 = AngleAxisr(4 / 6.0f * angle_per_unit_current_acc * dt_real * dt_real, current.accelerations.angular / angle_per_unit_current_acc);
                            Quaternionr angle_dt_q_2 = Quaternionr(angle_dt_aa_2);
                            next.pose.orientation *= angle_dt_q_2;
                        }

                        real_T angle_per_unit_previous_acc = previous.accelerations.angular.norm();
                        if (Utils::isDefinitelyGreaterThan(angle_per_unit_previous_acc, 0.0f))
                        {
                            AngleAxisr angle_dt_aa_3 = AngleAxisr(-1 / 6.0f * angle_per_unit_previous_acc * dt_real * dt_real, previous.accelerations.angular / angle_per_unit_previous_acc);
                            Quaternionr angle_dt_q_3 = Quaternionr(angle_dt_aa_3);
                            next.pose.orientation *= angle_dt_q_3;
                        }
                    }
                    if (VectorMath::hasNan(next.pose.orientation)) {
                        //Utils::DebugBreak();
                        Utils::log("orientation had NaN!", Utils::kLogLevelError);
                    }
                    //re-normalize quaternion to avoid accumulating error
                    next.pose.orientation.normalize();

                    // limit the position of the drone to avoid core dumping
                    next.pose.position.x() = std::max(std::min(next.pose.position.x(), 10000.0f), -10000.0f);
                    next.pose.position.y() = std::max(std::min(next.pose.position.y(), 10000.0f), -10000.0f);
                    next.pose.position.z() = std::max(std::min(next.pose.position.z(), 10000.0f), -10000.0f);
                }                 
            }

        private:
            bool dummy_trial = false;
            static constexpr uint kCollisionResponseCycles = 1;
            static constexpr float kAxisTolerance = 0.25f;
            static constexpr float kRestingVelocityMax = 0.1f;
            static constexpr float kDragMinVelocity = 0.1f;

            std::stringstream debug_string_;
            bool enable_ground_lock_;
            TTimePoint last_message_time;
            Vector3r wind_;
            float propeller_damage_coefficients[4] = { 1.0, 1.0, 1.0, 1.0 };

            // Variables for the different explicit integration methods
            bool use_average_values = false;
            bool use_quat = true;
            int integration_method_v = 3; // 0 (Verlet), 1 (AirSim), 2 (Adams-Bashfort), 3 (Beeman and Schofield)
            int integration_method_x = 3; // 0 (Verlet), 1 (AirSim), 2 (Adams-Bashfort), 3 (Beeman and Schofield)
            Kinematics::State previous;
            TTimeDelta initial_time = -1;


            // Parameter for the Bebop2
            static constexpr int max_w = 1256; // maximum number of radians per second of the propeller
            static constexpr int max_w_squared = max_w * max_w; // maximum number of radians per second of the propeller squared
            static constexpr real_T R = 0.075; // radius described by the propeller
            static constexpr real_T l = 0.0875; // distance of propeller to the y-axis of the drone
            static constexpr real_T b = 0.1150; // distance of propeller to x-axis
            static constexpr int signr = -1;
            static constexpr real_T rho = 1.225;  // density at the altitude in which the drone flies
            static constexpr int h = 5;   // variable used in the computation of dCt and dCq

            static constexpr real_T rad2deg_factor = 180.0f / M_PIf;   // variable that is used to convert an angle from radians to degrees

            real_T omega1_last = 796.761719f;
            real_T omega2_last = 796.761719f;
            real_T omega3_last = 796.761719f;
            real_T omega4_last = 796.761719f;


            // Variables related to process noise
            std::default_random_engine generator;
            bool activate_process_noise = false;
            Vector3r cov_pqr = Vector3r(0.0001f, 0.0001f, 0.0001f);
            Vector3r cov_att = Vector3r(10e-10f, 10e-10f, 10e-10f);
            Vector3r cov_xyz = Vector3r(10e-10f, 10e-10f, 10e-10f);
            Vector3r cov_vxyz = Vector3r(0.0001f, 0.0001f, 0.0001f);

            Vector3r mean_pqr = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r mean_att = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r mean_xyz = Vector3r(0.0f, 0.0f, 0.0f);
            Vector3r mean_vxyz = Vector3r(0.0f, 0.0f, 0.0f);

            // Variables for propeller damage
            vector<DamagedPropeller> damaged_propellers_;
            int n_blade_segment_ = 100;
            vector<real_T> cla_coeffs_{ 0.241574347f, 5.15236959f, -12.2553556f };
            vector<real_T> cda_coeffs_{ 0.00922164567f, -0.792542848f, 15.1364609f };

            // Variables related to mass and aero forces and moments storage
            Vector3r mass_F_ = Vector3r::Zero();
            Vector3r mass_M_ = Vector3r::Zero();
            Vector3r aero_F_ = Vector3r::Zero();
            Vector3r aero_M_ = Vector3r::Zero();

        };

    }
} //namespace
#endif
