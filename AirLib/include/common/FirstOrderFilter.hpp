// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.


#ifndef airsimcore_firstorderfilter_hpp
#define airsimcore_firstorderfilter_hpp

#include <cmath>
#include "UpdatableObject.hpp"
#include "Common.hpp"

namespace msr { namespace airlib {

template <typename T>
class FirstOrderFilter : public UpdatableObject {
    /*
    This class can be used to apply a first order filter on a signal.
    It allows different acceleration and deceleration time constants.

    Short review of discrete time implementation of first order system:
    Laplace:
    X(s)/U(s) = 1/(tau*s + 1)
    continuous time system:
    dx(t) = (-1/tau)*x(t) + (1/tau)*u(t)
    discretized system (ZoH):
    x(k+1) = exp(samplingTime*(-1/tau))*x(k) + (1 - exp(samplingTime*(-1/tau))) * u(k)
    */
public:
    FirstOrderFilter()
    {
        //allow default constructor with later call for initialize
    }
    FirstOrderFilter(float timeConstant, T initial_input, T initial_output, T initial_current_state_dot = 0, T initial_previous_state_dot = 0, int integration_method = 1)
    {
        initialize(timeConstant, initial_input, initial_output, initial_current_state_dot, initial_previous_state_dot, integration_method);
    }
    void initialize(float timeConstant, T initial_input, T initial_output, T initial_current_state_dot = 0, T initial_previous_state_dot = 0, int integration_method = 1)
    {
        timeConstant_ = timeConstant;
        initial_input_ = initial_input;
        initial_output_ = initial_output;
        initial_current_state_dot_ = initial_current_state_dot;
        initial_previous_state_dot_ = initial_previous_state_dot;
        //initial_current_state_dot_ = static_cast<double>(initial_current_state_dot);
        //initial_previous_state_dot_ = static_cast<double>(initial_previous_state_dot);
        integration_method_ = integration_method;
        //output_remaining_ = 0.0;
    }

    //*** Start: UpdatableState implementation ***//
    virtual void resetImplementation() override
    {
        last_time_ = clock()->nowNanos();
        input_ = initial_input_;
        output_ = initial_output_;
        current_state_dot_ = initial_current_state_dot_;
        previous_state_dot_ = initial_previous_state_dot_;
        //output_remaining_ = 0.0;
    }

    virtual void update() override
    {
        UpdatableObject::update();

        TTimeDelta dt = clock()->updateSince(last_time_);

        if (integration_method_ == 0)  // Verlet algorithm
        {
            next_state_dot_ = -1 / timeConstant_ * output_ + 1 / timeConstant_ * input_;
            output_diff = (current_state_dot_ + next_state_dot_) * (0.5f * dt);
            output_difference_ = static_cast<real_T>(output_diff);
            output_ = static_cast<real_T>(output_ + output_diff);
            current_state_dot_ = next_state_dot_;
        }
        else if (integration_method_ == 1)  // Verlet algorithm AirSim
        {
            //lower the weight for previous value if its been long time
            //TODO: minimize use of exp
            double alpha = exp(-dt / timeConstant_);
            // x(k+1) = Ad*x(k) + Bd*u(k)
            output_difference_ = static_cast<real_T>((output_ - input_) * (1 - alpha));
            output_ = static_cast<real_T>(output_ * alpha + input_ * (1 - alpha));
        }
        else if (integration_method_ == 2)  // Adams-Bashfort 2-step method
        {
            next_state_dot_ = -1 / timeConstant_ * output_ + 1 / timeConstant_ * input_;
            output_diff =  3 / 2.0f * next_state_dot_ * dt - current_state_dot_ * (0.5f * dt);
            output_difference_ = static_cast<real_T>(output_diff);
            output_ = static_cast<real_T>(output_ + output_diff);
            current_state_dot_ = next_state_dot_;
        }
        else if (integration_method_ == 3)  // Beeman and Schofield
        {
            next_state_dot_ = -1 / timeConstant_ * output_ + 1 / timeConstant_ * input_;
            output_diff = 1 / 6.0f * (2 * next_state_dot_ + 5 * current_state_dot_ - previous_state_dot_) * dt;
            output_difference_ = static_cast<real_T>(output_diff);
            output_ = static_cast<real_T>(output_ + output_diff);
            previous_state_dot_ = current_state_dot_;
            current_state_dot_ = next_state_dot_;
        }

        //if (integration_method_ == 0)  // Verlet algorithm
        //{
        //    output_old_ = output_;
        //    output_double_ = output_remaining_ + output_;
        //    next_state_dot_ = -1 / timeConstant_ * output_double_ + 1 / timeConstant_ * input_;
        //    output_diff = (current_state_dot_ + next_state_dot_) * (0.5f * dt);
        //    output_difference_ = static_cast<real_T>(output_diff);
        //    output_ = static_cast<real_T>(output_double_ + output_diff);
        //    if (output_old_ == output_)
        //    {
        //        output_remaining_ = output_remaining_ + output_diff;
        //    }
        //    else
        //    {
        //        output_remaining_ = 0.0;
        //    }
        //    current_state_dot_ = next_state_dot_;
        //}
        //else if (integration_method_ == 1)  // Verlet algorithm AirSim
        //{
        //    //lower the weight for previous value if its been long time
        //    //TODO: minimize use of exp
        //    double alpha = exp(-dt / timeConstant_);
        //    // x(k+1) = Ad*x(k) + Bd*u(k)
        //    output_difference_ = static_cast<real_T>((output_ - input_) * (1 - alpha));
        //    output_ = static_cast<real_T>(output_ * alpha + input_ * (1 - alpha));
        //}
        //else if (integration_method_ == 2)  // Adams-Bashfort 2-step method
        //{
        //    output_old_ = output_;
        //    output_double_ = output_remaining_ + output_;
        //    next_state_dot_ = -1 / timeConstant_ * output_double_ + 1 / timeConstant_ * input_;
        //    output_diff = 3 / 2.0f * next_state_dot_ * dt - current_state_dot_ * (0.5f * dt);
        //    output_difference_ = static_cast<real_T>(output_diff);
        //    output_ = static_cast<real_T>(output_double_ + output_diff);
        //    if (output_old_ == output_)
        //    {
        //        output_remaining_ = output_remaining_ + output_diff;
        //    }
        //    else
        //    {
        //        output_remaining_ = 0.0;
        //    }
        //    current_state_dot_ = next_state_dot_;
        //}
        //else if (integration_method_ == 3)  // Beeman and Schofield
        //{
        //    output_old_ = output_;
        //    output_double_ = output_remaining_ + output_;
        //    next_state_dot_ = -1 / timeConstant_ * output_double_ + 1 / timeConstant_ * input_;
        //    output_diff = 1 / 6.0f * (2 * next_state_dot_ + 5 * current_state_dot_ - previous_state_dot_) * dt;
        //    output_difference_ = static_cast<real_T>(output_diff);
        //    output_ = static_cast<real_T>(output_double_ + output_diff);
        //    if (output_old_ == output_)
        //    {
        //        output_remaining_ = output_remaining_ + output_diff;
        //    }
        //    else
        //    {
        //        output_remaining_ = 0.0;
        //    }
        //    previous_state_dot_ = current_state_dot_;
        //    current_state_dot_ = next_state_dot_;
        //}

    }
    //*** End: UpdatableState implementation ***//


    void setInput(T input)
    {
        input_ = input;
    }
    T getInput() const
    {
        return input_;
    }

    T getOutput() const
    {
        return output_;
    }

    void setOutput(T output)
    {
        output_ = output;
    }

    T getOutputDifference() const
    {
        return output_difference_;
    }


    //T getOutputDifference()
    //{
    //    output_remaining_ = 0.0;
    //    return output_difference_;
    //}

private:
    float timeConstant_;
    int integration_method_;
    T output_, output_old_, input_, output_difference_;
    T initial_output_, initial_input_;
    TTimePoint last_time_;
    TTimeDelta output_diff;
    T next_state_dot_, current_state_dot_, previous_state_dot_;
    T initial_current_state_dot_, initial_previous_state_dot_;

    //double output_double_, output_remaining_;
    //double next_state_dot_, current_state_dot_, previous_state_dot_;
    //double initial_current_state_dot_, initial_previous_state_dot_;
};  

}} //namespace
#endif
