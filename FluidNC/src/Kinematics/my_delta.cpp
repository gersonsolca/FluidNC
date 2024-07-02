
 #include "my_delta.h"

#include "../Machine/MachineConfig.h"
#include "../Limits.h"  // ambiguousLimit()
#include "../Machine/Homing.h"

#include "../Protocol.h"  // protocol_execute_realtime

#include <cmath>

/*
==================== How it Works ====================================
On the spidermic, Grbl axis units are in radians
The kinematics converts the cartesian moves in gcode into
the radians to move the arms. The Grbl motion planner never sees
the actual cartesian values.

To make the moves straight and smooth on spidermic, the cartesian moves
are broken into small segments where the non linearity will not be noticed.
This is similar to how Grbl draws arcs.

For mpos reporting, the motor position in steps is proportional to the bobbin angle
in radians, which is then converted to cartesian via the forward kinematics 
transform.
Bobbin angle > 0 and < l * sqrt(2). Bobbin angle 0 means position in the nearest corner (hard limit)

Feedrate in gcode is in cartesian units. This must be converted to the
angles. This is done by calculating the segment move distance and the angle 
move distance and applying that ration to the feedrate.

Default configuration

kinematics:
    SpiderMic:

TODO
- 

*/

namespace Kinematics {
    
    void SpiderMic::group(Configuration::HandlerBase& handler) {
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);
        handler.item("softLimits", _softLimits);
        handler.item("x_min_mm", _x_min, 0.0, 250.0);
        handler.item("x_max_mm", _x_max, 350.0, 500.0);
        handler.item("y_min_mm", _y_min, 0.0, 250.0);
        handler.item("y_max_mm", _y_max, 350.0, 500.0);
    }

    void SpiderMic::init() {
        // print a startup message to show the kinematics are enabled. Print the offset for reference
        log_info("Kinematic system:" << name() << " soft_limits:" << _softLimits);

        init_position();
    }

    void SpiderMic::init_position() {
        // Set motor positions
        for (size_t axis = X_AXIS; axis <= A_AXIS; axis++) {
            last_motor_angles[axis] = sqrt(pow(_x_max, 2) + pow(_y_max, 2)) / 2 + sqrt(pow(_x_min, 2) + pow(_y_min, 2));
        }
        
        // Sets cartesian position
        last_cartesian[X_AXIS] = (_x_min + _x_max) / 2;
        last_cartesian[Y_AXIS] = (_y_min + _y_max) / 2;
    }

    bool SpiderMic::invalid_line(float* cartesian) {
        if (!_softLimits)
            return false;

        if (!transform_cartesian_to_motors(last_motor_angles, cartesian)) {
            limit_error();
            return true;
        }

        return false;
    }

    // Not implemented yet
    bool SpiderMic::invalid_arc(float* target, plan_line_data_t* pl_data, float* position, float center[3], float radius, size_t caxes[3], bool is_clockwise_arc) {
        return false;
    }

    // Not implemented yet. Copied from Cartesian.
    void SpiderMic::constrain_jog(float* target, plan_line_data_t* pl_data, float* position) {
        // log_debug("Jog Test: from (" << position[X_AXIS] << "," << position[Y_AXIS] << ") to (" << target[X_AXIS] << "," << position[Y_AXIS] << ")");
        if (!_softLimits)
            return;

        float motors[MAX_N_AXIS] = { 0.0, 0.0, 0.0, 0.0 };

        // Temp fix
        // If the target is reachable do nothing
        if (transform_cartesian_to_motors(motors, target)) {
            return;
        } else {
            log_warn("Kinematics soft limit jog rejection");
            copyAxes(target, position);
        }

        // TO DO better idea
        // loop back from the target in increments of  kinematic_segment_len_mm unitl the position is valid.
        // constrain to that target.
    }

    bool SpiderMic::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
        //auto axes   = config->_axes;
        //auto n_axis = axes->_numberAxis;
        float motor_angles[4];                          // 
        float seg_target[2];                            // The target of the current segment
        float cartesian_feed_rate = pl_data->feed_rate; // save original feed rate

        // Check if start position is in the work area
        if (!transform_cartesian_to_motors(motor_angles, position)) {
            log_warn("Kinematics error. Position unreachable (" << position[X_AXIS] << "," << position[Y_AXIS] << ")");
            return false;
        }
        seg_target[X_AXIS] = position[X_AXIS];
        seg_target[Y_AXIS] = position[Y_AXIS];

        // Check if destination is in the work area
        if (!transform_cartesian_to_motors(motor_angles, target)) {
            log_warn("Kinematics error. Target unreachable (" << target[X_AXIS] << "," << target[Y_AXIS] << ")");
            return false;
        }

        //position[X_AXIS] += gc_state.coord_offset[X_AXIS];
        //position[Y_AXIS] += gc_state.coord_offset[Y_AXIS];

        float cartesian_dist = vector_distance(target, position, 2);

        // Divide into segments, round up to have at least 1 except when dist is 0
        uint32_t segment_count = ceil(cartesian_dist / _kinematic_segment_len_mm);

        // Cartesian segment movement
        float dr[2];
        dr[X_AXIS] = (target[X_AXIS] - position[X_AXIS]) / segment_count;
        dr[Y_AXIS] = (target[Y_AXIS] - position[Y_AXIS]) / segment_count;

        // Calculate desired cartesian feedrate distance ratio, same for each segment
        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort) {
                return true;
            }

            // determine this segment's target
            seg_target[X_AXIS] += dr[X_AXIS];
            seg_target[Y_AXIS] += dr[Y_AXIS];

            log_debug("Segment:" << segment << " of " << segment_count);
            log_debug("Segment target (" << seg_target[X_AXIS] << "," << seg_target[Y_AXIS] << ")");

            // Calculate motor movement angles
            if (!transform_cartesian_to_motors(motor_angles, seg_target)) {
                log_error("Kinematic error. Motors (" << motor_angles[X_AXIS] << "," << motor_angles[Y_AXIS] << "," << motor_angles[Z_AXIS] << "," << motor_angles[A_AXIS] << ")");
                return false;
            }

            // Adjust feedrate by the ratio of the segment lengths in motor and cartesian spaces, accounting for all axes
            // Rapid motions ignore feedrate. Don't convert.
            // T=D/V, Tcart=Tmotor, Dcart/Vcart=Dmotor/Vmotor
            // Vmotor = Dmotor*(Vcart/Dcart)
            if (!pl_data->motion.rapidMotion) {
                float motor_segment_length   = vector_distance(last_motor_angles, motor_angles, 4);
                float cartesian_segment_dist = cartesian_dist / segment_count;
                pl_data->feed_rate           = cartesian_feed_rate * motor_segment_length / cartesian_segment_dist;
            }

            // mc_line() returns false if a jog is cancelled.
            // In that case we stop sending segments to the planner.
            if (!mc_move_motors(motor_angles, pl_data)) {
                return false;
            }

            // Save angles for next distance calculations
            // This is after mc_line() so that we do not update last_angle if the segment was discarded.
            copyAxes(last_motor_angles, motor_angles);
            copyAxes(last_cartesian, seg_target);
        }
        return true;
    }

    void SpiderMic::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        copyAxes(last_motor_angles, motors);

        float x_AD = 0.5 * ((motors[X_AXIS] + motors[A_AXIS]) * (motors[X_AXIS] - motors[A_AXIS]) / _x_max + _x_max);
        float x_BC = 0.5 * ((motors[Y_AXIS] + motors[Z_AXIS]) * (motors[Y_AXIS] - motors[Z_AXIS]) / _x_max + _x_max);
        log_debug("motors_to_cartesian x_AD:" << x_AD << " x_BC:" << x_BC);

        float y_BA = 0.5 * ((motors[Y_AXIS] + motors[X_AXIS]) * (motors[Y_AXIS] - motors[X_AXIS]) / _y_max + _y_max);
        float y_CD = 0.5 * ((motors[Z_AXIS] + motors[A_AXIS]) * (motors[Z_AXIS] - motors[A_AXIS]) / _y_max + _y_max);
        log_debug("motors_to_cartesian y_BA:" << y_BA << " y_CD:" << y_CD);

        // Average is calculated. They provide the same value anyway
        cartesian[X_AXIS] = last_cartesian[X_AXIS] = (x_AD + x_BC) / 2;
        cartesian[Y_AXIS] = last_cartesian[Y_AXIS] = (y_BA + y_CD) / 2;
    }

    // Not completely implemented
    bool SpiderMic::kinematics_homing(AxisMask& axisMask) {
        auto axes   = config->_axes;
        auto n_axis = axes->_numberAxis;

        config->_axes->set_disable(false);
        
        // Touch limits with each motor sequentially: B -> A -> D
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            //releaseMotors(AxisMask axisMask, MotorMask motors)


            //int32_t steps = mpos_to_steps(_homing_mpos, axis);
            //set_motor_steps(axis, steps);
            //set_motor_steps(axis, mpos_to_steps(axes->_axis[axis]->_homing->_mpos, axis));
        }

        // A limit switch on either axis stops both motors
        //config->_axes->_axis[X_AXIS]->_motors[0]->limitOtherAxis(Y_AXIS);
        //config->_axes->_axis[Y_AXIS]->_motors[0]->limitOtherAxis(X_AXIS);

        /*
        // Set motor position to steps
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            int32_t steps = mpos_to_steps(_homing_mpos, axis);
            set_motor_steps(axis, steps);
            //set_motor_steps(axis, mpos_to_steps(axes->_axis[axis]->_homing->_mpos, axis));
        }
        protocol_disable_steppers();
        */

        //Homing::set_all_axes_homed()

        return true;  // signal main code that this handled all homing
    }

    void SpiderMic::releaseMotors(AxisMask axisMask, MotorMask motors) {
        auto axes   = config->_axes;
        auto n_axis = axes->_numberAxis;
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            if (bitnum_is_true(axisMask, axis)) {
                axes->_axis[axis]->_motors[0]->unlimit();
            }
        }
    }

    bool SpiderMic::transform_cartesian_to_motors(float* motors, float* cartesian) {
        if (_softLimits) { // Check if target is in the cartesian space
            if (cartesian[X_AXIS] < _x_min) {
                log_debug("Kinematics error. Target:" << cartesian[X_AXIS] << " exceeds x_min:" << _x_min);
                return false;
            }
            if (cartesian[X_AXIS] > _x_max) {
                log_debug("Kinematics error. Target:" << cartesian[X_AXIS] << " exceeds x_max:" << _x_max);
                return false;
            }
            if (cartesian[Y_AXIS] < _y_min) {
                log_debug("Kinematics error. Target:" << cartesian[Y_AXIS] << " exceeds y_min:" << _y_min);
                return false;
            }
            if (cartesian[Y_AXIS] > _y_max) {
                log_debug("Kinematics error. Target:" << cartesian[Y_AXIS] << " exceeds y_max:" << _y_max);
                return false;
            }
        } else { // Check if motors movement abide the range
            float l = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));
            float l_min = sqrt(pow(_x_min, 2) + pow(_y_min, 2));
            float l_max = sqrt(pow(_x_max, 2) + pow(_y_max, 2));
            if (l > l_max or l < l_min) {
                log_debug("Kinematics transform error. Target:" << l << " exceeds d1_max:" << l_max << " or d2_max:" << l_min);
                return false;
            }
        }
        
        log_debug("transform_cartesian_to_motors: cartesian (" << cartesian[X_AXIS] << "," << cartesian[Y_AXIS] << ")");

        // Assign motor movements
        motors[X_AXIS] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS] - _y_max, 2));
        motors[Y_AXIS] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));
        motors[Z_AXIS] = sqrt(pow(cartesian[X_AXIS] - _x_max, 2) + pow(cartesian[Y_AXIS], 2));
        motors[A_AXIS] = sqrt(pow(cartesian[X_AXIS] - _x_max, 2) + pow(cartesian[Y_AXIS] - _y_max, 2));
        //vector_distance(target, position, 2)

        log_debug("transform_cartesian_to_motors: motors (" << motors[X_AXIS] << "," << motors[Y_AXIS] << "," << motors[Z_AXIS] << "," << motors[A_AXIS] << ")");

        return true;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<SpiderMic> registration("SpiderMic");
    }
}
