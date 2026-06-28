#include "custom_mcode.h"
#include "driver.h"

#if CUSTOM_MCODE_ENABLE

#include "grbl/core_handlers.h"
#include "grbl/gcode.h"
#include "grbl/hal.h"
#include "grbl/kinematics.h"
#include "grbl/machine_limits.h"
#include "grbl/motion_control.h"
#include "grbl/nuts_bolts.h"
#include "grbl/stepper.h"
#include <math.h>
#include <stdio.h>

// #include "grbl/hal.h"
// #include "grbl/protocol.h"
// #include "grbl/task.h"

static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;

static void set_rpm(uint32_t value)
{
#if MOTOR_CONTROL_ENABLE
#include "motor_control.h"
    set_adrc_spindle_speed((float)value);
#endif
}

#if LINEAR_MOTOR
#include "custom/linear_motor.h"
#endif

static void home_loop()
{
    // 1. Get the current axis mask for the X axis
    axes_signals_t x_axis_mask = { .mask = bit(X_AXIS) };
    home_signals_t limit_signals;

    // 2. Poll continuously until the X switch registers an active state
    do {
        // Read the current live state of all homing/limit switches via the HAL
        limit_signals = hal.homing.get_state();

        // Run essential real-time background checks (handles E-Stop, Serial communication, and Feed Holds)
        grbl.on_execute_realtime(STATE_HOMING);

        // Break the loop if an abort/reset was requested by the host application
        if (sys.rt_exec_state & (EXEC_RESET | EXEC_SAFETY_DOOR)) {
            break;
        }

        // Condition: Keep looping as long as the X bit in the limit signals array remains 0 (untriggered)
    } while (!(limit_signals.a.mask & x_axis_mask.mask));
}

static void seek_home()
{

    // Ensure the system isn't currently in an aborted state
    if (ABORTED)
        return;

    char debug_buf[80];
    uint_fast8_t idx = 1;
    axes_signals_t axis_mask = { .bits = bit(X_AXIS) };

    coord_data_t target;
    plan_line_data_t plan_data;
    homing_mode_t mode = HomingMode_Seek; // Seek | Pulloff | Locate
    home_signals_t home_signals;
    float pos_start[idx];
    float pos_end[idx];

    float homing_rate = 0.0f;

    snprintf(debug_buf, sizeof(debug_buf), "seek_home" ASCII_EOL);
    hal.stream.write_all(debug_buf);

    do {

        --idx;
        pos_start[idx] = (float)sys.position[idx] / settings.axis[idx].steps_per_mm;

        // Hardware/Driver board pins are allocated and capable of homing axis
        // if (!bit_istrue(hal.driver_cap.homing_cycle, bit(idx))) {
        //     continue;
        // }

        axis_mask = (axes_signals_t) { .bits = bit(idx) };

        axes_signals_t axis_lock = { 0 };
        axis_lock.mask |= kinematics.limits_get_axis_mask(idx);

        system_convert_array_steps_to_mpos(target.values, sys.position); // Initialize with current machine position
        float travel_distance = settings.axis[idx].max_travel;
        if (bit_istrue(settings.homing.dir_mask.value, bit(idx))) {
            target.values[idx] -= fabsf(travel_distance); // Move in positive direction
        } else {
            target.values[idx] += fabsf(travel_distance); // Move in negative direction
        }

        if ((homing_rate = hal.homing.get_feedrate(axis_mask, mode)) == 0.0f) {
            continue;
        }

        plan_data_init(&plan_data);
        plan_data.condition.system_motion = On; // Informs core this is firmware-directed, not user G-code
        plan_data.condition.no_feed_override = On; // Locks movement speed to the specified feed rate
        plan_data.line_number = DEFAULT_HOMING_CYCLE_LINE_NUMBER;
        plan_data.feed_rate = homing_rate;
        // plan_data.feed_rate = settings.axis[X_AXIS].homing_seek_rate;

        sys.homing_axis_lock.mask = axis_lock.mask;

#ifdef KINEMATICS_API
        coord_data_t k_target;
        plan_buffer_line(kinematics.transform_from_cartesian(k_target.values, target.values), &plan_data);
#else
        plan_buffer_line(target.values, &plan_data);
#endif

        sys.step_control.flags = 0;
        sys.step_control.execute_sys_motion = On;

        // st_prep_buffer(); // Prefill hardware segment buffers
        st_wake_up(); // Ignite step generation timers

        do {
            home_signals = hal.homing.get_state();

            grbl.on_execute_realtime(STATE_HOMING);

            if (sys.rt_exec_state & (EXEC_RESET | EXEC_SAFETY_DOOR)) {
                break;
            }

            st_prep_buffer();

        } while (!(home_signals.a.mask & axis_mask.mask));

        // Hard stop and clear buffers once the switch has been encountered
        st_reset();
        sys.step_control.flags = 0;

        pos_end[idx] = (float)sys.position[idx] / settings.axis[idx].steps_per_mm;
    } while (idx);

    float x_offset = pos_end[X_AXIS] - pos_start[X_AXIS];
    // float y_offset = pos_end[Y_AXIS] - pos_start[Y_AXIS];

    snprintf(debug_buf, sizeof(debug_buf), "seek_home X: %.3f -> %.3f = %.3f" ASCII_EOL, pos_start[X_AXIS], pos_end[X_AXIS], x_offset);
    hal.stream.write_all(debug_buf);

    // snprintf(debug_buf, sizeof(debug_buf), "seek_home Y: %.3f -> %.3f = %.3f" ASCII_EOL, pos_start[Y_AXIS], pos_end[Y_AXIS], y_offset);
    // hal.stream.write_all(debug_buf);
}

static user_mcode_type_t cb_check(user_mcode_t mcode)
{
    return (mcode == Custom_Set_RPM || mcode == Custom_Home_Seek 
        || mcode == Custom_Abort 
        || Custom_LINEAR_MOTOR_RPM
        || Custom_LINEAR_MOTOR_TRIM
    )

        ? UserMCode_Normal
        : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Unsupported);
}

static status_code_t cb_validate(parser_block_t* gc_block)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch (gc_block->user_mcode) {

    case Custom_Set_RPM:
    case Custom_LINEAR_MOTOR_RPM:

        if (gc_block->words.s && gc_block->values.s >= 0) {
            gc_block->output_command.value = gc_block->values.s;
            state = Status_OK;
        }

        break;

    case Custom_Abort:
    case Custom_Home_Seek:
    case Custom_LINEAR_MOTOR_TRIM:
        state = Status_OK;
        break;

    default:
        state = Status_Unhandled;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block) : state;
}

static status_code_t cb_execute(sys_state_t state, parser_block_t* gc_block)
{

    if (state == STATE_CHECK_MODE) {
        return Status_OK;
    }

    switch (gc_block->user_mcode) {
    case Custom_Set_RPM: {
        set_rpm(gc_block->output_command.value);
        break;
    }

    case Custom_LINEAR_MOTOR_RPM: {
        #ifdef LINEAR_MOTOR
        linear_motor_set_rpm(gc_block->output_command.value);
        #endif 
        break;
    }

    case Custom_LINEAR_MOTOR_TRIM: {
        #ifdef LINEAR_MOTOR
        linear_motor_trim();
        #endif
        break;
    }

    case Custom_Abort: {
        grbl.enqueue_realtime_command(CMD_STOP);
    }

    break;
    case Custom_Home_Seek:
        seek_home();
        break;

    default:
        break;
    }

    return Status_OK;
}

static void onReportOptions(bool newopt)
{
    on_report_options(newopt);

    if (!newopt)
        report_plugin("Custom MCode", "0.10");
}

void custom_mcode_init(void)
{
    memcpy(&user_mcode, &grbl.user_mcode, sizeof(user_mcode_ptrs_t));
    grbl.user_mcode.check = cb_check;
    grbl.user_mcode.validate = cb_validate;
    grbl.user_mcode.execute = cb_execute;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

#endif