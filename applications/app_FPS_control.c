/*
	Copyright 2019 Benjamin Vedder	benjamin@vedder.se

	This file is part of the VESC firmware.

	The VESC firmware is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    The VESC firmware is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

#include "app.h"
#include "ch.h"
#include "hal.h"

// Some useful includes
#include "mc_interface.h"
#include "utils_math.h"
#include "encoder/encoder.h"
#include "terminal.h"
#include "comm_can.h"
#include "hw.h"
#include "commands.h"
#include "timeout.h"

#include <math.h>
#include <string.h>
#include <stdio.h>

extern float can_target_pos;
extern float can_target_speed;
extern float can_Kp;
extern float can_Kd;
extern float can_forward_torque;

#define LIMIT(value, min, max) \
    do { \
        if ((value) <= (min)) { \
            (value) = (min); \
        } else if ((value) >= (max)) { \
            (value) = (max); \
        } \
    } while(0)

// Threads
static THD_FUNCTION(FPS_control_thread, arg);
static THD_WORKING_AREA(FPS_control_thread_wa, 1024);

// Private functions
static void pwm_callback(void);
static void terminal_test(int argc, const char **argv);

// Private variables
const volatile mc_configuration *mc_conf;

static volatile bool stop_now = true;
static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	chThdCreateStatic(FPS_control_thread_wa, sizeof(FPS_control_thread_wa),
			NORMALPRIO, FPS_control_thread, NULL);

	// Terminal commands for the VESC Tool terminal can be registered.
	terminal_register_command_callback(
			"custom_cmd",
			"Print the number d",
			"[d]",
			terminal_test);
}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {
	terminal_unregister_callback(terminal_test);

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(FPS_control_thread, arg) {
	(void)arg;

	chRegSetThreadName("App Custom");

	is_running = true;

	for(;;) {
	    mc_conf = mc_interface_get_configuration();

		float current_pos = mc_interface_get_pos_multiturn();   // 获取当前多圈位置,度
		float current_speed = mc_interface_get_rpm();   // 获取当前速度, erpm

		float current_pos_rad = current_pos * 2.0 * M_PI / 360.0; // 转换为弧度
		float current_speed_rad_s = current_speed / (mc_conf->si_motor_poles/2) * 2.0 * M_PI / 60.0; // 转换为rad/s

		float pos_error = can_target_pos - current_pos_rad; // 位置误差
		float speed_error = can_target_speed - current_speed_rad_s; // 速度误差

		float i_set = can_Kp * pos_error + can_Kd * speed_error + can_forward_torque; // PID控制器输出电流设定值

		float max = mc_conf->l_current_max_scale * mc_conf->l_current_max * 0.8; // 最大电流限制

		LIMIT(i_set, -max, max);
		// mc_interface_set_current(i_set); // 设置电流

		commands_printf("i_set: %f", (double)i_set);

		timeout_reset(); // Reset timeout if everything is OK.
		chThdSleepMilliseconds(1);
	}
}

// Callback function for the terminal command with arguments.
static void terminal_test(int argc, const char **argv) {
	if (argc == 2) {
		int d = -1;
		sscanf(argv[1], "%d", &d);

		commands_printf("You have entered %d", d);

		// For example, read the ADC inputs on the COMM header.
		commands_printf("ADC1: %.2f V ADC2: %.2f V",
				(double)ADC_VOLTS(ADC_IND_EXT), (double)ADC_VOLTS(ADC_IND_EXT2));
	} else {
		commands_printf("This command requires one argument.\n");
	}
}
