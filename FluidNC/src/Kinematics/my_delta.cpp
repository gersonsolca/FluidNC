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
        _kinematic_segment_len_mm: 1
        _x_max: 374
        _y_max: 374
        _softLimits: false
        _softLimits_mm: 100

TODO
- kinematics_homing()
- invalid_arc()
- constrain_jog()
*/

namespace Kinematics {
    
    void SpiderMic::group(Configuration::HandlerBase& handler) {
        handler.item("kinematic_segment_len_mm", _kinematic_segment_len_mm, 0.05, 20.0);
        handler.item("x_max_mm", _x_max, 200.0, 500.0);
        handler.item("y_max_mm", _y_max, 200.0, 500.0);
        handler.item("softLimits", _softLimits);
        handler.item("softLimits_mm", _softLimits_mm, 200, min(_x_max, _y_max));
    }

    void SpiderMic::init() {
        // print a startup message to show the kinematics are enabled. Print the offset for reference
        log_info("Kinematic system:" << name() << " soft_limits:" << _softLimits);
        if (_softLimits) {log_info("soft_limits_mm (from center): " << _softLimits_mm)} 

        init_position();
    }

    void SpiderMic::init_position() {
<<<<<<< HEAD
        // Machine starts at the center, cartesian (_x_max*0.5, _y_max*0.5)
        // Each motor starts in the middle, motor distance (sqrt(pow(_x_max, 2) + pow(_y_max, 2))*0.25)
        last_cartesian_mm[X_AXIS] = _x_max * 0.5;
        last_cartesian_mm[Y_AXIS] = _y_max * 0.5;
        
        float l0 = sqrt(pow(_x_max, 2) + pow(_y_max, 2)) * 0.25;
        for (size_t axis = X_AXIS; axis < B_AXIS; axis++) {
            last_motor_mm[axis] = l0;
        }

        Homing::set_mpos();
=======
        // Set motor positions
        for (size_t axis = X_AXIS; axis <= A_AXIS; axis++) {
            last_motor_angles[axis] = sqrt(pow(_x_max, 2) + pow(_y_max, 2)) / 2 + sqrt(pow(_x_min, 2) + pow(_y_min, 2));
        }
        
        // Sets cartesian position
        last_cartesian[X_AXIS] = (_x_min + _x_max) / 2;
        last_cartesian[Y_AXIS] = (_y_min + _y_max) / 2;
>>>>>>> 91a7b666 (movement debugged, correct distances)
    }

    bool SpiderMic::invalid_line(float* cartesian) {
        if (!_softLimits)
            return false;

<<<<<<< HEAD
        if (!transform_cartesian_to_motors(last_motor_mm, cartesian)) {
=======
        if (!transform_cartesian_to_motors(last_motor_angles, cartesian)) {
>>>>>>> 91a7b666 (movement debugged, correct distances)
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

        float motors[MAX_N_AXIS] = {0.0};

        // Temp fix
        // If the target is reachable do nothing
        if (transform_cartesian_to_motors(motors, target)) {
            return;
        } else {
            log_warn("Kinematics soft limit jog rejection");
            copyAxes(target, position);
        }

        // TO DO better idea
        // loop back from the target in increments of kinematic_segment_len_mm unitl the position is valid.
        // constrain to that target.
    }

    bool SpiderMic::cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) {
<<<<<<< HEAD
        float motor_angles_mm[4];                       // Dummy motor angles
        float seg_target_mm[2];                         // The target of the current segment
=======
        //auto axes   = config->_axes;
        //auto n_axis = axes->_numberAxis;
        float motor_angles[4];                          // 
        float seg_target[2];                            // The target of the current segment
>>>>>>> 91a7b666 (movement debugged, correct distances)
        float cartesian_feed_rate = pl_data->feed_rate; // save original feed rate

        // Check if start position is in the work area
        if (!transform_cartesian_to_motors(motor_angles_mm, position)) {
            log_warn("Kinematics error. Position unreachable (" << position[X_AXIS] << ", " << position[Y_AXIS] << ")");
            return false;
        }
        seg_target_mm[X_AXIS] = position[X_AXIS];
        seg_target_mm[Y_AXIS] = position[Y_AXIS];

        // Check if destination is in the work area
        if (!transform_cartesian_to_motors(motor_angles_mm, target)) {
            log_warn("Kinematics error. Target unreachable (" << target[X_AXIS] << ", " << target[Y_AXIS] << ")");
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
            seg_target_mm[X_AXIS] += dr[X_AXIS];
            seg_target_mm[Y_AXIS] += dr[Y_AXIS];

            log_debug("Segment:" << segment << " of " << segment_count);
            log_debug("Segment target (" << seg_target_mm[X_AXIS] << "," << seg_target_mm[Y_AXIS] << ")");

            // Calculate motor movement angles
<<<<<<< HEAD
            if (!transform_cartesian_to_motors(motor_angles_mm, seg_target_mm)) {
                log_error("Kinematic error. Motors (" << motor_angles_mm[X_AXIS] << "," << motor_angles_mm[Y_AXIS] << "," << motor_angles_mm[Z_AXIS] << "," << motor_angles_mm[A_AXIS] << ")");
=======
            if (!transform_cartesian_to_motors(motor_angles, seg_target)) {
                log_error("Kinematic error. Motors (" << motor_angles[X_AXIS] << "," << motor_angles[Y_AXIS] << "," << motor_angles[Z_AXIS] << "," << motor_angles[A_AXIS] << ")");
>>>>>>> 91a7b666 (movement debugged, correct distances)
                return false;
            }

            // Adjust feedrate by the ratio of the segment lengths in motor and cartesian spaces, accounting for all axes. Rapid motions ignore feedrate
            // T=D/V, Tcart=Tmotor, Dcart/Vcart=Dmotor/Vmotor, Vmotor = Dmotor*(Vcart/Dcart)
            if (!pl_data->motion.rapidMotion) {
<<<<<<< HEAD
                float motor_segment_dist   = vector_distance(last_motor_mm, motor_angles_mm, 4);
=======
                float motor_segment_length   = vector_distance(last_motor_angles, motor_angles, 4);
>>>>>>> 91a7b666 (movement debugged, correct distances)
                float cartesian_segment_dist = cartesian_dist / segment_count;
                pl_data->feed_rate           = cartesian_feed_rate * motor_segment_dist / cartesian_segment_dist;
            }

            // mc_line() returns false if a jog is cancelled.
            // In that case we stop sending segments to the planner.
            if (!mc_move_motors(motor_angles_mm, pl_data)) {
                return false;
            }

            // Save angles for next distance calculations
            // This is after mc_line() so that we do not update last_angle if the segment was discarded.
<<<<<<< HEAD
            copyAxes(last_motor_mm, motor_angles_mm);
            copyAxes(last_cartesian_mm, seg_target_mm);
=======
            copyAxes(last_motor_angles, motor_angles);
            copyAxes(last_cartesian, seg_target);
>>>>>>> 91a7b666 (movement debugged, correct distances)
        }
        return true;
    }

    void SpiderMic::motors_to_cartesian(float* cartesian, float* motors, int n_axis) {
<<<<<<< HEAD
        //log_debug("motors_to_cartesian motors: (" << motors[0] << "," << motors[1] << "," << motors[2] << "," << motors[3] << ")");
        //log_info("motors_to_cartesian r_A0:" << r_0[0] << " r_B0:" << r_0[1] << " r_C0:" << r_0[2] << " r_D0:" << r_0[3]);

        float x_AD = (motors[0] + motors[3]) * (motors[0] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5;
        float x_BC = (motors[1] + motors[2]) * (motors[1] - motors[2]) * pow(C_b, 2) * 0.5 + 0.5;
        //log_info("motors_to_cartesian x_AD:" << x_AD << " x_BC:" << x_BC);

        float y_BA = (motors[1] + motors[0]) * (motors[1] - motors[0]) * pow(C_b, 2) * 0.5 + 0.5;
        float y_CD = (motors[2] + motors[3]) * (motors[2] - motors[3]) * pow(C_b, 2) * 0.5 + 0.5;
        //log_info("motors_to_cartesian y_BA:" << y_BA << " y_CD:" << y_CD);

        // Min of the 2 values (they provide the same value anyway)
        cartesian[X_AXIS] = fmin(x_AD, x_BC); // = last_cartesian[X_AXIS]
        cartesian[Y_AXIS] = fmin(y_BA, y_CD); // = last_cartesian[Y_AXIS]
=======
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
>>>>>>> 91a7b666 (movement debugged, correct distances)
    }

    // Not completely implemented
    bool SpiderMic::kinematics_homing(AxisMask& axisMask) {
        auto axes   = config->_axes;
        auto n_axis = axes->_numberAxis;

        log_debug("axisMask: " << axisMask);

        float maxmotor[MAX_N_AXIS], minmotor[MAX_N_AXIS] = {0};
        int time = 5;

        // Touch limits with motors B and D
        log_debug("Homing axis: Y");
        // Release motors except Y
        //releaseMotors(AxisMask axisMask, MotorMask motors);
        // Touch with Y motor
        // Set 0 on Y motor
        // Pull with Z motor
        // Set max on Z motor

        /*
        See CoreXY release motors

        releaseMotors(motormask_234);

        move_axis(X_AXIS);

        int timer = 0;
        while(Machine::stopin(X_AXIS)){
            if (timer <= time) {
                stop_axis(X_AXIS);
                log_error(“No end stop found for axis X”)
                return;
            }
            stop_axis(X_AXIS);
        }

        // Implement double touch?

        minmotor[X_AXIS] = motor[X_AXIS];

        holdmotor(X_AXIS);

        move_axis(Y_AXIS);
        move_axis(Z_AXIS);

        timer = 0;
        while (Machine::stopin(Y_AXIS) || Machine::stopin(Z_AXIS)){
            if (timer <= time) {
                stop_axis(Y_AXIS);
                stop_axis(Z_AXIS);
                log_error(“No end stop found for the axes”)
                return;
            }
            if (Machine::stopin(Y_AXIS)) {
                stop_axis(Y_AXIS);
            }
            if (Machine::stopin(Z_AXIS)) {
                stop_axis(Z_AXIS);;
            }
        }

        maxmotor[Z_AXIS] = motor[Z_AXIS];
        */        

        log_debug("Homing axis: Z");
        // Release motors except Z
        //releaseMotors(AxisMask axisMask, MotorMask motors);
        // Touch with Z motor
        // Set 0 on Z motor
        // Pull with Y motor
        // Set max on Y motor


        // Touch limits with motors B and D
        log_debug("Homing axis: X");
        // Release motors except X
        //releaseMotors(AxisMask axisMask, MotorMask motors);
        // Touch with X motor
        // Set 0 on X motor
        // Pull with A motor
        // Set max on A motor

        
<<<<<<< HEAD

        /*
        //config->_axes->motorMask
        log_debug("TCtM: homingmask (" << config->_axes->homingMask << ")");
        log_debug("TCtM: limitmask (" << config->_axes->limitMask << ")");
        log_debug("TCtM: limitmask (" << config->_axes->negLimitMask << ")");
        log_debug("TCtM: limitmask (" << config->_axes->posLimitMask << ")");
        //AxisMask axisMask; // 0x01 // 0b0001
        //MotorMask motorMask; // 0x01 // 0b00000001
        //releaseMotors(AxisMask axisMask, MotorMask motors);
        //bitnum_is_true(axisMask, axis);

        #include <chrono>  // for high_resolution_clock
        ...


        // Record start time
        auto start = std::chrono::high_resolution_clock::now();

        // Portion of code to be timed
        ...

        // Record end time
        auto finish = std::chrono::high_resolution_clock::now();
        */
        //MotorMask limited = Machine::Axes::posLimitMask | Machine::Axes::negLimitMask;
        /*
        // For Cartesian, the limit switches are associated with individual motors, since
        // an axis can have dual motors each with its own limit switch.  We clear the motors in
        // the mask whose limits have been reached.
        MotorMask axisMask, limited, motors = 0;
        MotorMask limited = 0;
        clear_bits(axisMask, limited);
        clear_bits(motors, limited);

        //axes->_axis[axis]->_motors[0]->unlimit();

        //int32_t steps = mpos_to_steps(_homing_mpos, axis);
        //set_motor_steps(axis, steps);
        //set_motor_steps(axis, mpos_to_steps(axes->_axis[axis]->_homing->_mpos, axis));
=======
        // Touch limits with each motor sequentially: B -> A -> D
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            //releaseMotors(AxisMask axisMask, MotorMask motors)


            //int32_t steps = mpos_to_steps(_homing_mpos, axis);
            //set_motor_steps(axis, steps);
            //set_motor_steps(axis, mpos_to_steps(axes->_axis[axis]->_homing->_mpos, axis));
        }
>>>>>>> 91a7b666 (movement debugged, correct distances)

        // A limit switch on either axis stops both motors
        //config->_axes->_axis[X_AXIS]->_motors[0]->limitOtherAxis(Y_AXIS);
        //config->_axes->_axis[Y_AXIS]->_motors[0]->limitOtherAxis(X_AXIS);
<<<<<<< HEAD

        // Set motor position to steps
        for (size_t axis = X_AXIS; axis < n_axis; axis++) {
            last_motor_mm[axis] = l0;
=======
>>>>>>> 91a7b666 (movement debugged, correct distances)

            int32_t steps_per_mm = axes->_axis[axis]->_stepsPerMm;
            int32_t steps = l0 / steps_per_mm;
            set_motor_steps(axis, steps);
            // motor steps = 200 * 16 = 3'200

            mpos_to_steps(l0, axis);
            set_motor_steps(axis, 0);  // Set to zero
            set_motor_steps_from_mpos(&l0);
            get_axis_motor_steps();
        }

        protocol_disable_steppers();

        Axes::set_disable();
        */
        
        Homing::set_all_axes_homed();

        return true;  // signal main code that this handled all homing
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
<<<<<<< HEAD
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
=======
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
>>>>>>> 91a7b666 (movement debugged, correct distances)

        return true;
    }

    // Configuration registration
    namespace {
        KinematicsFactory::InstanceBuilder<SpiderMic> registration("SpiderMic");
    }
}
