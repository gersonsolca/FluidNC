#pragma once

/*
    my_delta.h

    This is a kinematic system to move a puck on an horizontal plane with four strings spanning from the vertices of a rectangular frame
*/

#include "Kinematics.h"
#include "Cartesian.h"

namespace Kinematics {

    class SpiderMic : public Cartesian {
    public:
        SpiderMic() = default;

        SpiderMic(const SpiderMic&)            = delete;
        SpiderMic(SpiderMic&&)                 = delete;
        SpiderMic& operator=(const SpiderMic&) = delete;
        SpiderMic& operator=(SpiderMic&&)      = delete;

        // Kinematic Interface
        virtual void init() override;
        virtual void init_position() override;
        //bool canHome(AxisMask& axisMask) override;
        bool cartesian_to_motors(float* target, plan_line_data_t* pl_data, float* position) override;
        void motors_to_cartesian(float* cartesian, float* motors, int n_axis) override;
        bool transform_cartesian_to_motors(float* motors, float* cartesian) override;
        //bool soft_limit_error_exists(float* cartesian) override;
        bool         kinematics_homing(AxisMask& axisMask) override;
        virtual void constrain_jog(float* cartesian, plan_line_data_t* pl_data, float* position) override;
        virtual bool invalid_line(float* cartesian) override;
        virtual bool invalid_arc(float*            target,
                                plan_line_data_t* pl_data,
                                float*            position,
                                float             center[3],
                                float             radius,
                                size_t            caxes[3],
                                bool              is_clockwise_arc) override;

        void releaseMotors(AxisMask axisMask, MotorMask motors) override;

        // Configuration handlers:
        void         validate() override {}
        virtual void group(Configuration::HandlerBase& handler) override;
        void         afterParse() override {}

        // Name of the configurable. Must match the name registered in the cpp file.
        virtual const char* name() const override { return "SpiderMic"; }

        ~SpiderMic() {}

    private:
    // State
    float last_motor_angles[MAX_N_AXIS]; // Save motor positions for calculations
    float last_cartesian[MAX_N_AXIS];    // Save cartesian position for calculations

    //  Parameters
    float _R_b                      = 6.0;   // bobbin inner radius on which thread is rolled [mm], ideal value R_b = 20 / π -> steps_per_mm = 80
    float _kinematic_segment_len_mm = 1.0;   // the maximun segment length the move is broken into
    bool  _softLimits               = false;
    float _x_min                    = 0;
    float _x_max                    = 372.0; // ~ (370 - 372) side lenght of field [mm]
    float _y_min                    = 0;
    float _y_max                    = 372.0; // ~ (370 - 372) side width of field [mm]

    float _d1_max;
    float _d2_max;
    float _homing_mpos;

    // Constants for calculations
    // M_PI is not defined in standard C/C++ but some compilers support it anyway.  The following suppresses Intellisense problem reports.
    #ifndef M_PI
    #    define M_PI 3.14159265358979323846
    #endif
    float C_b  = _R_b * M_PI / 180.0;
    
    protected:
    };
    
}  //  namespace Kinematics
