#ifdef my_delta_kinematics

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
   - Constrain the geometry values to realistic values.

*/

namespace Kinematics {
    // the geometry of the spider mic
    float R_b;
    float max_x;
    float max_y;
    float _homing_mpos = sqrt(pow(max_x / 2, 2) + pow(max_y / 2, 2));

    float C_b  = R_b * M_PI / 180.0;

    static float last_angles[MAX_N_AXIS]    = { 0.0 };  // A place to save the previous motor angles for distance/feed rate calcs
    static float last_cartesian[MAX_N_AXIS] = { 0.0 };  // A place to save the previous motor angles for distance/feed rate calcs

    void SpiderMic::group(Configuration::HandlerBase& handler) {
        handler.item("bobbin_radius_mm", R_b, 0.1, 20.0);
        handler.item("field_lenght_mm", max_x, 200.0, 500.0);
        handler.item("field_width_mm", max_y, 200.0, 500.0);
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);
        handler.item("soft_limits", _softLimits);
    }

    void SpiderMic::init() {
        // print a startup message to show the kinematics are enabled. Print the offset for reference
        log_info("Kinematic system:" << name() << " soft_limits:" << _softLimits);

        auto axes   = config->_axes;
        auto n_axis = config->_axes->_numberAxis;

        // warn about axis soft limits
        for (int axis = 0; axis < n_axis; axis++) {
            if (axes->_axis[axis]->_softLimits) {
                log_config_error(" All soft_limits configured in axes should be false");
                break;
            }
        }

        init_position();
    }

    void SpiderMic::init_position() {
        float angles[MAX_N_AXIS]    = { _homing_mpos, _homing_mpos, _homing_mpos, _homing_mpos };
        float cartesian[MAX_N_AXIS] = { (float)0.5 * max_x, (float)0.5 * max_y };
        motors_to_cartesian(cartesian, angles, 4);  // Sets the cartesian values
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

        float motors[MAX_N_AXIS] = { 0.0, 0.0, 0.0 };

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
        float dx, dy;           // distances in each cartesian axis
        float motor_angles[4];

        float seg_target[4];                    // The target of the current segment
        float feed_rate  = pl_data->feed_rate;  // save original feed rate
        bool  show_error = true;                // shows error once

        bool calc_ok = true;

        if (target[X_AXIS] > max_x) {
            log_debug("Kinematics error. Target:" << target[X_AXIS] << " exceeds max_x:" << max_x);
            return false;
        }

        if (target[Y_AXIS] > max_y) {
            log_debug("Kinematics error. Target:" << target[Y_AXIS] << " exceeds max_y:" << max_y);
            return false;
        }

        calc_ok = transform_cartesian_to_motors(last_angles, position);
        if (!calc_ok) {
            log_warn("Kinematics error. Start position error (" << position[0] << "," << position[1] << "," << position[2] << "," << position[3] << ")");
            return false;
        }

        //log_debug("Target (" << target[0] << "," << target[1] << "," << target[2] << "," << target[3]);
        // Check the destination to see if it is in work area
        calc_ok = transform_cartesian_to_motors(motor_angles, target);
        if (!calc_ok) {
            log_warn("Kinematics error. Target unreachable (" << target[0] << "," << target[1] << "," << target[2] << "," << target[3] << ")");
            return false;
        }

        position[X_AXIS] += gc_state.coord_offset[X_AXIS];
        position[Y_AXIS] += gc_state.coord_offset[Y_AXIS];

        // calculate cartesian move distance for each axis
        dx         = target[X_AXIS] - position[X_AXIS];
        dy         = target[Y_AXIS] - position[Y_AXIS];
        float dist = sqrt((dx * dx) + (dy * dy));

        // determine the number of segments we need	... round up so there is at least 1 (except when dist is 0)
        uint32_t segment_count = ceil(dist / _kinematic_segment_len_mm);

        float segment_dist = dist / ((float)segment_count);  // distance of each segment...will be used for feedrate conversion

        for (uint32_t segment = 1; segment <= segment_count; segment++) {
            if (sys.abort) {
                return true;
            }
            //log_debug("Segment:" << segment << " of " << segment_count);
            // determine this segment's target
            seg_target[X_AXIS] = position[X_AXIS] + (dx / float(segment_count) * segment);
            seg_target[Y_AXIS] = position[Y_AXIS] + (dy / float(segment_count) * segment);

            //log_debug("Segment target (" << seg_target[0] << "," << seg_target[1] << "," << seg_target[2] << ")");

            // calculate the delta motor angles
            bool calc_ok = transform_cartesian_to_motors(motor_angles, seg_target);

            if (!calc_ok) {
                if (show_error) {
                    log_error("Kinematic error motors (" << motor_angles[0] << "," << motor_angles[1] << "," << motor_angles[2] << "," << motor_angles[3] << ")");
                    show_error = false;
                }
                return false;
            }
            if (pl_data->motion.rapidMotion) {
                pl_data->feed_rate = feed_rate;
            } else {
                float delta_distance = two_axis_dist(motor_angles, last_angles);
                pl_data->feed_rate   = (feed_rate * delta_distance / segment_dist);
            }

            // mc_line() returns false if a jog is cancelled.
            // In that case we stop sending segments to the planner.
            if (!mc_move_motors(motor_angles, pl_data)) {
                return false;
            }

            // save angles for next distance calc
            // This is after mc_line() so that we do not update
            // last_angle if the segment was discarded.
            memcpy(last_angles, motor_angles, sizeof(motor_angles));
        }
        return true;
    }

    void SpiderMic::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
        //log_debug("motors_to_cartesian motors: (" << motors[0] << "," << motors[1] << "," << motors[2] << "," << motors[3] << ")");
        //log_info("motors_to_cartesian r_A0:" << r_0[0] << " r_B0:" << r_0[1] << " r_C0:" << r_0[2] << " r_D0:" << r_0[3]);

        float x_AD = (motors[0] + motors[3]) * (motors[0] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5;
        float x_BC = (motors[1] + motors[2]) * (motors[1] - motors[2]) * pow(C_b, 2) * 0.5 + 0.5;
        //log_info("motors_to_cartesian x_AD:" << x_AD << " x_BC:" << x_BC);

        float y_BA = (motors[1] + motors[0]) * (motors[1] - motors[0]) * pow(C_b, 2) * 0.5 + 0.5;
        float y_CD = (motors[2] + motors[3]) * (motors[2] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5;
        //log_info("motors_to_cartesian y_BA:" << y_BA << " y_CD:" << y_CD);

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
        float max_l1 = sqrt(pow(max_x, 2) + pow(max_y, 2));
        float max_l2 = sqrt(pow(max_x, 2) + pow(max_y, 2));
        float l1 = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));
        float l2 = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2));

        if (l1 > max_l1) {
            log_debug("Kinematics transform error. Target:" << l1 << " exceeds l1:" << max_l1);
            return false;
        }

        if (l2 > max_l2) {
            log_debug("Kinematics transform error. Target:" << l2 << " exceeds max l2:" << max_l2);
            return false;
        }

        // determine positive or negative direction?
        motors[0] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS] - l2, 2)) / pow(C_b, 2);
        motors[1] = sqrt(pow(cartesian[X_AXIS], 2) + pow(cartesian[Y_AXIS], 2)) / pow(C_b, 2);
        motors[2] = sqrt(pow(cartesian[X_AXIS] - l1, 2) + pow(cartesian[Y_AXIS], 2)) / pow(C_b, 2);
        motors[3] = sqrt(pow(cartesian[X_AXIS] - l1, 2) + pow(cartesian[Y_AXIS] - l2, 2)) / pow(C_b, 2);

        return true;
    }

    // Determine the unit distance between (2) 2D points
    float SpiderMic::two_axis_dist(float* point1, float* point2) {
        return sqrt(((point1[0] - point2[0]) * (point1[0] - point2[0])) + ((point1[1] - point2[1]) * (point1[1] - point2[1])));
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<SpiderMic> registration("spider_mic");
    }
}

#endif
