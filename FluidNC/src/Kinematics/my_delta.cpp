
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
        handler.item("bobbin_radius_mm", _R_b, 2.5, 20.0);
        handler.item("field_lenght_mm", _max_x, 300.0, 500.0);
        handler.item("field_width_mm", _max_y, 300.0, 500.0);
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);
        handler.item("soft_limits", _softLimits);
    }

    void SpiderMic::init() {
        // print a startup message to show the kinematics are enabled. Print the offset for reference
        log_info("Kinematic system:" << name() << " soft_limits:" << _softLimits);

        auto axes   = config->_axes;
        auto n_axis = config->_axes->_numberAxis;

        // warn about axis soft limits
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            if (axes->_axis[axis]->_softLimits) {
                log_config_error(" All soft_limits configured in axes should be false");
                break;
            }
        }

        _max_l = sqrt(pow(_max_x, 2) + pow(_max_y, 2));
        _homing_mpos = _max_l / 2;

        init_position();
    }

    void SpiderMic::init_position() {
        auto n_axis = config->_axes->_numberAxis;
        
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            last_motor_angles[axis] = _homing_mpos;
        }
        last_cartesian[X_AXIS] = _max_x * 0.5;
        last_cartesian[Y_AXIS] = _max_y * 0.5;

        motors_to_cartesian(last_cartesian, last_motor_angles, 4);  // Sets the cartesian values
    }

    bool SpiderMic::invalid_line(float* cartesian) {
        if (!_softLimits)
            return false;

        float motors[MAX_N_AXIS] = { 0.0, 0.0, 0.0, 0.0 };

        if (!transform_cartesian_to_motors(motors, cartesian)) {
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
        // log_debug("Jog Test: from(" << position[X_AXIS] << ")"
        //                             << " to(" << target[X_AXIS] << ")");
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
        float motor_angles[4];                          // 
        float seg_target[2];                            // The target of the current segment
        float cartesian_feed_rate = pl_data->feed_rate; // save original feed rate
        float dx, dy;                                   // cartesian segment distances in each cartesian axis
        uint32_t segment_count;                         // number of segments the move will be broken in to.
        bool calc_ok = true;

        // Check target is in the cartesian space
        if (target[X_AXIS] > _max_x) {
            log_debug("Kinematics error. Target:" << target[X_AXIS] << " exceeds max_x:" << _max_x);
            return false;
        }

        if (target[Y_AXIS] > _max_y) {
            log_debug("Kinematics error. Target:" << target[Y_AXIS] << " exceeds max_y:" << _max_y);
            return false;
        }
        
        log_debug("Position (" << position[0] << "," << position[1] << ")");
        log_debug("Target (" << target[0] << "," << target[1] << ")");

        // Check if start position is in the work area
        calc_ok = transform_cartesian_to_motors(last_motor_angles, position);
        if (!calc_ok) {
            log_warn("Kinematics error. Start position unreachable (" << target[0] << "," << target[1] << ")");
            return false;
        }

        // Check if destination is in the work area
        calc_ok = transform_cartesian_to_motors(motor_angles, target);
        if (!calc_ok) {
            log_warn("Kinematics error. Target unreachable (" << target[0] << "," << target[1] << ")");
            return false;
        }

        //position[X_AXIS] += gc_state.coord_offset[X_AXIS];
        //position[Y_AXIS] += gc_state.coord_offset[Y_AXIS];

        float cartesian_dist = vector_distance(target, position, 2);

        // Divide into segments, round up to have at least 1 except when dist is 0
        segment_count = ceil(cartesian_dist / _kinematic_segment_len_mm);

        // Segment cartesian movement
        dx = (target[X_AXIS] - position[X_AXIS]) / segment_count;
        dy = (target[Y_AXIS] - position[Y_AXIS]) / segment_count;

        // Calculate desired cartesian feedrate distance ratio, same for each segment
        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort) {
                return true;
            }

            // determine this segment's target
            seg_target[X_AXIS] += dx;
            seg_target[Y_AXIS] += dy;

            log_debug("Segment:" << segment << " of " << segment_count);
            log_debug("Segment target (" << seg_target[0] << "," << seg_target[1] << ")");

            // Calculate motor movement angles
            calc_ok = transform_cartesian_to_motors(motor_angles, seg_target);
            if (!calc_ok) {
                log_error("Kinematic error. Motors (" << motor_angles[0] << "," << motor_angles[1] << "," << motor_angles[2] << "," << motor_angles[3] << ")");
                return false;
            }

            // Adjust feedrate by the ratio of the segment lengths in motor and cartesian spaces,
            // accounting for all axes
            if (!pl_data->motion.rapidMotion) {  // Rapid motions ignore feedrate. Don't convert.
                                                // T=D/V, Tcart=Tmotor, Dcart/Vcart=Dmotor/Vmotor
                                                // Vmotor = Dmotor*(Vcart/Dcart)
                float motor_segment_length   = vector_distance(last_motor_angles, motor_angles, sizeof(motor_angles));
                float cartesian_segment_dist = cartesian_dist / segment_count;
                pl_data->feed_rate           = cartesian_feed_rate * motor_segment_length / cartesian_segment_dist;
            }

            // mc_line() returns false if a jog is cancelled.
            // In that case we stop sending segments to the planner.
            if (!mc_move_motors(motor_angles, pl_data)) {
                return false;
            }

            // Save angles for next distance calc
            // This is after mc_line() so that we do not update last_angle if the segment was discarded.
            copyAxes(last_motor_angles, motor_angles);
        }
        return true;
    }

    void SpiderMic::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        log_debug("motors_to_cartesian motors: (" << motors[0] << "," << motors[1] << "," << motors[2] << "," << motors[3] << ")");

        float x_AD = ((motors[0] + motors[3]) * (motors[0] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5) * _max_x;
        float x_BC = ((motors[1] + motors[2]) * (motors[1] - motors[2]) * pow(C_b, 2) * 0.5 + 0.5) * _max_x;
        log_info("motors_to_cartesian x_AD:" << x_AD << " x_BC:" << x_BC);

        float y_BA = ((motors[1] + motors[0]) * (motors[1] - motors[0]) * pow(C_b, 2) * 0.5 + 0.5) * _max_y;
        float y_CD = ((motors[2] + motors[3]) * (motors[2] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5) * _max_y;
        log_info("motors_to_cartesian y_BA:" << y_BA << " y_CD:" << y_CD);

        // Average is calculated but they should provide the same value
        cartesian[X_AXIS] = (x_AD + x_BC) / 2;
        cartesian[Y_AXIS] = (y_BA + y_CD) / 2;
    }

    // Not implemented yet
    bool SpiderMic::kinematics_homing(AxisMask& axisMask) {
        auto axes   = config->_axes;
        auto n_axis = axes->_numberAxis;

        config->_axes->set_disable(false);

        // TODO deal with non kinematic axes above Z
        for (int axis = 0; axis < 3; axis++) {
            //set_motor_steps(axis, mpos_to_steps(axes->_axis[axis]->_homing->_mpos, axis));
            int32_t steps = mpos_to_steps(_homing_mpos, axis);
            set_motor_steps(axis, steps);
        }
        protocol_disable_steppers();
        return true;  // signal main code that this handled all homing

        //float max_l1 = sqrt(pow(_max_x, 2) + pow(_max_y, 2));
        //float max_l2 = sqrt(pow(_max_x, 2) + pow(_max_y, 2));
        //float l1 = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));
        //float l2 = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));

        /*
        // This declares a lambda, which can be called just like a function
        auto print_message = [](std::string message) 
        { 
            std::cout << message << "\n"; 
        };
        */
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
        float l = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));

        if (l > _max_l) {
            log_debug("Kinematics transform error. Target:" << l << " exceeds l1:" << _max_l);
            return false;
        }

        // determine positive or negative direction?
        motors[0] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS] - l, 2)) / pow(C_b, 2);
        motors[1] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2)) / pow(C_b, 2);
        motors[2] = sqrt(pow(cartesian[X_AXIS] - l, 2) + pow(cartesian[Y_AXIS], 2)) / pow(C_b, 2);
        motors[3] = sqrt(pow(cartesian[X_AXIS] - l, 2) + pow(cartesian[Y_AXIS] - l, 2)) / pow(C_b, 2);

        return true;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<SpiderMic> registration("SpiderMic");
    }
}
