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

// Private variables
const volatile mc_configuration *mc_conf;

static volatile bool is_running = false;

// Called when the custom application is started. Start our
// threads here and set up callbacks.
void app_custom_start(void) {
	chThdCreateStatic(FPS_control_thread_wa, sizeof(FPS_control_thread_wa),
			NORMALPRIO, FPS_control_thread, NULL);

	commands_init_plot("x", "y");
	commands_plot_add_graph("current_set");
	commands_plot_add_graph("current_pos");
	commands_plot_add_graph("current_speed");
	commands_plot_add_graph("target_pos");
	commands_plot_add_graph("target_speed");
	commands_plot_add_graph("target_torque");

}

// Called when the custom application is stopped. Stop our threads
// and release callbacks.
void app_custom_stop(void) {

	while (is_running) {
		chThdSleepMilliseconds(1);
	}
}

void app_custom_configure(app_configuration *conf) {
	(void)conf;
}

static THD_FUNCTION(FPS_control_thread, arg) {
	(void)arg;

	float x_axis = 0.0f;
	int plot_div = 0;

	chRegSetThreadName("App Custom");

	for(;;) {
	    mc_conf = mc_interface_get_configuration();

		float current_pos = mc_interface_get_pos_multiturn();   // 获取当前多圈位置,度
		float current_speed = mc_interface_get_rpm();   // 获取当前速度, erpm

		static float current_speed_filtered = 0.0f;
		float alpha = 0.1f;
		current_speed = current_speed * alpha + current_speed_filtered * (1-alpha);
		current_speed_filtered = current_speed;

		float current_pos_rad = current_pos * 2.0 * M_PI / 360.0; // 转换为弧度
		float current_speed_rad_s = current_speed / (mc_conf->si_motor_poles/2) * 2.0 * M_PI / 60.0; // 转换为rad/s

		float pos_error = can_target_pos - current_pos_rad; // 位置误差
		float speed_error = can_target_speed - current_speed_rad_s; // 速度误差

		float i_set = can_Kp * pos_error + can_Kd * speed_error + can_forward_torque; // PID控制器输出电流设定值

		// float max = mc_conf->l_current_max_scale * mc_conf->l_current_max * 0.8; // 最大电流限制
		float max = 20.0f;

		LIMIT(i_set, -max, max);
		mc_interface_set_current(i_set); // 设置电流

		// commands_printf("i_set: %f, kp: %f, kd: %f, cur_pos: %f",
		            // (double)i_set, (double)can_Kp, (double)can_Kd, (double)current_pos_rad);

		if(++plot_div >= 20){
            plot_div = 0;

            commands_plot_set_graph(0);
            commands_send_plot_points(x_axis, i_set);

            commands_plot_set_graph(1);
            commands_send_plot_points(x_axis, current_pos_rad);

            commands_plot_set_graph(2);
            commands_send_plot_points(x_axis, current_speed_rad_s);

            commands_plot_set_graph(3);
            commands_send_plot_points(x_axis, can_target_pos);

            commands_plot_set_graph(4);
            commands_send_plot_points(x_axis, can_target_speed);

            commands_plot_set_graph(5);
            commands_send_plot_points(x_axis, can_forward_torque);

            x_axis++; // X轴递增
		}

		timeout_reset(); // Reset timeout if everything is OK.
		chThdSleepMilliseconds(1);
	}
}
