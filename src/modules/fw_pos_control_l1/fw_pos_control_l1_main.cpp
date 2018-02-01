/****************************************************************************
 *
 *   Copyright (c) 2013-2016 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


/**
 * @file fw_pos_control_l1_main.c
 * Implementation of a generic position controller based on the L1 norm. Outputs a bank / roll
 * angle, equivalent to a lateral motion (for copters and rovers).
 *
 * Original publication for horizontal control class:
 *    S. Park, J. Deyst, and J. P. How, "A New Nonlinear Guidance Logic for Trajectory Tracking,"
 *    Proceedings of the AIAA Guidance, Navigation and Control
 *    Conference, Aug 2004. AIAA-2004-4900.
 *
 * Original implementation for total energy control class:
 *    Paul Riseborough and Andrew Tridgell, 2013 (code in lib/external_lgpl)
 *
 * More details and acknowledgements in the referenced library headers.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <poll.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>

#include "landingslope.h"

#include <arch/board/board.h>
#include <drivers/drv_accel.h>
#include <drivers/drv_hrt.h>
#include <ecl/l1/ecl_l1_pos_controller.h>
#include <external_lgpl/tecs/tecs.h>
#include <geo/geo.h>
#include <launchdetection/LaunchDetector.h>
#include <mathlib/mathlib.h>
#include <platforms/px4_defines.h>
#include <runway_takeoff/RunwayTakeoff.h>
#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>
#include <systemlib/param/param.h>
#include <systemlib/perf_counter.h>
#include <systemlib/pid/pid.h>
#include <systemlib/systemlib.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/fw_pos_ctrl_status.h>
#include <uORB/topics/fw_virtual_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/tecs_status.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/uORB.h>
#include <vtol_att_control/vtol_type.h>

static int	_control_task = -1;			/**< task handle for sensor task */

#define HDG_HOLD_DIST_NEXT 			3000.0f 	// initial distance of waypoint in front of plane in heading hold mode
#define HDG_HOLD_REACHED_DIST 		1000.0f 	// distance (plane to waypoint in front) at which waypoints are reset in heading hold mode
#define HDG_HOLD_SET_BACK_DIST 		100.0f 		// distance by which previous waypoint is set behind the plane
#define HDG_HOLD_YAWRATE_THRESH 	0.15f 		// max yawrate at which plane locks yaw for heading hold mode
#define HDG_HOLD_MAN_INPUT_THRESH 	0.01f 		// max manual roll/yaw input from user which does not change the locked heading
#define T_ALT_TIMEOUT 				1 			// time after which we abort landing if terrain estimate is not valid
#define THROTTLE_THRESH 0.05f 	///< max throttle from user which will not lead to motors spinning up in altitude controlled modes
#define MANUAL_THROTTLE_CLIMBOUT_THRESH 0.85f	///< a throttle / pitch input above this value leads to the system switching to climbout mode
#define ALTHOLD_EPV_RESET_THRESH 5.0f

/**
 * L1 control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int fw_pos_control_l1_main(int argc, char *argv[]);

using namespace launchdetection;

class FixedwingPositionControl
{
public:
	/**
	 * Constructor
	 */
	FixedwingPositionControl();

	/**
	 * Destructor, also kills the sensors task.
	 */
	~FixedwingPositionControl();

	// prevent copying
	FixedwingPositionControl(const FixedwingPositionControl &) = delete;
	FixedwingPositionControl operator=(const FixedwingPositionControl &other) = delete;

	/**
	 * Start the sensors task.
	 *
	 * @return	OK on success.
	 */
	static int	start();

	/**
	 * Task status
	 *
	 * @return	true if the mainloop is running
	 */
	bool		task_running() { return _task_running; }

private:
	orb_advert_t	_mavlink_log_pub;

	bool		_task_should_exit;		/**< if true, sensor task should exit */
	bool		_task_running;			/**< if true, task is running in its mainloop */

	int		_global_pos_sub;
	int		_pos_sp_triplet_sub;
	int		_ctrl_state_sub;			/**< control state subscription */
	int		_control_mode_sub;		/**< control mode subscription */
	int		_vehicle_command_sub;		/**< vehicle command subscription */
	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_control_sub;		/**< notification of manual control updates */
	int		_sensor_combined_sub;		/**< for body frame accelerations */

	orb_advert_t	_attitude_sp_pub;		/**< attitude setpoint */
	orb_advert_t	_tecs_status_pub;		/**< TECS status publication */
	orb_advert_t	_fw_pos_ctrl_status_pub;		/**< navigation capabilities publication */

	orb_id_t _attitude_setpoint_id;

	struct control_state_s				_ctrl_state;			/**< control state */
	struct vehicle_attitude_setpoint_s		_att_sp;			/**< vehicle attitude setpoint */
	struct fw_pos_ctrl_status_s		_fw_pos_ctrl_status;		/**< navigation capabilities */
	struct manual_control_setpoint_s		_manual;			/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;			/**< control mode */
	struct vehicle_command_s			_vehicle_command;		/**< vehicle commands */
	struct vehicle_status_s				_vehicle_status;		/**< vehicle status */
	struct vehicle_land_detected_s			_vehicle_land_detected;		/**< vehicle land detected */
	struct vehicle_global_position_s		_global_pos;			/**< global vehicle position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;		/**< triplet of mission items */
	struct sensor_combined_s			_sensor_combined;		/**< for body frame accelerations */

	perf_counter_t	_loop_perf;			/**< loop performance counter */

	float	_hold_alt;				/**< hold altitude for altitude mode */
	float	_takeoff_ground_alt;				/**< ground altitude at which plane was launched */
	float	_hdg_hold_yaw;				/**< hold heading for velocity mode */
	bool	_hdg_hold_enabled;			/**< heading hold enabled */
	bool	_yaw_lock_engaged;			/**< yaw is locked for heading hold */
	float	_althold_epv;				/**< the position estimate accuracy when engaging alt hold */
	bool	_was_in_deadband;				/**< wether the last stick input was in althold deadband */
	struct position_setpoint_s _hdg_hold_prev_wp;	/**< position where heading hold started */
	struct position_setpoint_s _hdg_hold_curr_wp;	/**< position to which heading hold flies */
	hrt_abstime _control_position_last_called; /**<last call of control_position  */

	/* Landing */
	bool _land_noreturn_horizontal;
	bool _land_noreturn_vertical;
	bool _land_stayonground;
	bool _land_motor_lim;
	bool _land_onslope;
	bool _land_useterrain;

	Landingslope _landingslope;

	hrt_abstime _time_started_landing;	//*< time at which landing started */

	float _t_alt_prev_valid;	//**< last terrain estimate which was valid */
	hrt_abstime _time_last_t_alt; //*< time at which we had last valid terrain alt */

	float _flare_height;					//*< estimated height to ground at which flare started */
	float _flare_curve_alt_rel_last;
	float _target_bearing;				//*< estimated height to ground at which flare started */

	bool _was_in_air;	/**< indicated wether the plane was in the air in the previous interation*/
	hrt_abstime _time_went_in_air;	/**< time at which the plane went in the air */

	/* Takeoff launch detection and runway */
	launchdetection::LaunchDetector _launchDetector;
	LaunchDetectionResult _launch_detection_state;

	runwaytakeoff::RunwayTakeoff _runway_takeoff;

	bool _last_manual;				///< true if the last iteration was in manual mode (used to determine when a reset is needed)

	/* throttle and airspeed states */
	float _airspeed_error;				///< airspeed error to setpoint in m/s
	bool _airspeed_valid;				///< flag if a valid airspeed estimate exists
	uint64_t _airspeed_last_received;			///< last time airspeed was received. Used to detect timeouts.
	float _groundspeed_undershoot;			///< ground speed error to min. speed in m/s
	bool _global_pos_valid;				///< global position is valid
	math::Matrix<3, 3> _R_nb;			///< current attitude
	float _roll;
	float _pitch;
	float _yaw;
	bool _reinitialize_tecs;			///< indicates if the TECS states should be reinitialized (used for VTOL)
	bool _is_tecs_running;
	hrt_abstime _last_tecs_update;
	float _asp_after_transition;
	bool _was_in_transition;

	// estimator reset counters
	uint8_t _pos_reset_counter;		// captures the number of times the estimator has reset the horizontal position
	uint8_t _alt_reset_counter;		// captures the number of times the estimator has reset the altitude state

	ECL_L1_Pos_Controller				_l1_control;
	TECS						_tecs;
	enum FW_POSCTRL_MODE {
		FW_POSCTRL_MODE_AUTO,
		FW_POSCTRL_MODE_POSITION,
		FW_POSCTRL_MODE_ALTITUDE,
		FW_POSCTRL_MODE_OTHER
	} _control_mode_current;			///< used to check the mode in the last control loop iteration. Use to check if the last iteration was in the same mode.

	struct {
		float l1_period;
		float l1_damping;

		float time_const;
		float time_const_throt;
		float min_sink_rate;
		float max_sink_rate;
		float max_climb_rate;
		float climbout_diff;
		float heightrate_p;
		float heightrate_ff;
		float speedrate_p;
		float throttle_damp;
		float integrator_gain;
		float vertical_accel_limit;
		float height_comp_filter_omega;
		float speed_comp_filter_omega;
		float roll_throttle_compensation;
		float speed_weight;
		float pitch_damping;

		float airspeed_min;
		float airspeed_trim;
		float airspeed_max;
		float airspeed_trans;
		int airspeed_mode;

		float pitch_limit_min;
		float pitch_limit_max;
		float roll_limit;
		float throttle_min;
		float throttle_max;
		float throttle_idle;
		float throttle_cruise;
		float throttle_slew_max;
		float man_roll_max_rad;
		float man_pitch_max_rad;
		float rollsp_offset_rad;
		float pitchsp_offset_rad;

		float throttle_land_max;

		float land_slope_angle;
		float land_H1_virt;
		float land_flare_alt_relative;
		float land_thrust_lim_alt_relative;
		float land_heading_hold_horizontal_distance;
		float land_flare_pitch_min_deg;
		float land_flare_pitch_max_deg;
		int land_use_terrain_estimate;
		float land_airspeed_scale;

		int vtol_type;

	} _parameters;			/**< local copies of interesting parameters */

	struct {

		param_t l1_period;
		param_t l1_damping;

		param_t time_const;
		param_t time_const_throt;
		param_t min_sink_rate;
		param_t max_sink_rate;
		param_t max_climb_rate;
		param_t climbout_diff;
		param_t heightrate_p;
		param_t heightrate_ff;
		param_t speedrate_p;
		param_t throttle_damp;
		param_t integrator_gain;
		param_t vertical_accel_limit;
		param_t height_comp_filter_omega;
		param_t speed_comp_filter_omega;
		param_t roll_throttle_compensation;
		param_t speed_weight;
		param_t pitch_damping;

		param_t airspeed_min;
		param_t airspeed_trim;
		param_t airspeed_max;
		param_t airspeed_trans;
		param_t airspeed_mode;

		param_t pitch_limit_min;
		param_t pitch_limit_max;
		param_t roll_limit;
		param_t throttle_min;
		param_t throttle_max;
		param_t throttle_idle;
		param_t throttle_cruise;
		param_t throttle_slew_max;
		param_t man_roll_max_deg;
		param_t man_pitch_max_deg;
		param_t rollsp_offset_deg;
		param_t pitchsp_offset_deg;

		param_t throttle_land_max;

		param_t land_slope_angle;
		param_t land_H1_virt;
		param_t land_flare_alt_relative;
		param_t land_thrust_lim_alt_relative;
		param_t land_heading_hold_horizontal_distance;
		param_t land_flare_pitch_min_deg;
		param_t land_flare_pitch_max_deg;
		param_t land_use_terrain_estimate;
		param_t land_airspeed_scale;

		param_t vtol_type;

	} _parameter_handles;		/**< handles for interesting parameters */


	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update();

	/**
	 * Update control outputs
	 *
	 */
	void		control_update();

	/**
	 * Check for changes in control mode
	 */
	void		vehicle_control_mode_poll();

	/**
	 * Check for new in vehicle commands
	 */
	void		vehicle_command_poll();

	/**
	 * Check for changes in vehicle status.
	 */
	void		vehicle_status_poll();

	/**
	 * Check for changes in vehicle land detected.
	 */
	void		vehicle_land_detected_poll();

	/**
	 * Check for manual setpoint updates.
	 */
	bool		vehicle_manual_control_setpoint_poll();

	/**
	 * Check for changes in control state.
	 */
	void		control_state_poll();

	/**
	 * Check for accel updates.
	 */
	void		vehicle_sensor_combined_poll();

	/**
	 * Check for set triplet updates.
	 */
	void		vehicle_setpoint_poll();

	/**
	 * Publish navigation capabilities
	 */
	void		fw_pos_ctrl_status_publish();

	/**
	 * Get a new waypoint based on heading and distance from current position
	 *
	 * @param heading the heading to fly to
	 * @param distance the distance of the generated waypoint
	 * @param waypoint_prev the waypoint at the current position
	 * @param waypoint_next the waypoint in the heading direction
	 */
	void		get_waypoint_heading_distance(float heading, float distance,
			struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init);

	/**
	 * Return the terrain estimate during landing: uses the wp altitude value or the terrain estimate if available
	 */
	float		get_terrain_altitude_landing(float land_setpoint_alt, const struct vehicle_global_position_s &global_pos);

	/**
	 * Return the terrain estimate during takeoff or takeoff_alt if terrain estimate is not available
	 */
	float		get_terrain_altitude_takeoff(float takeoff_alt, const struct vehicle_global_position_s &global_pos);

	/**
	 * Check if we are in a takeoff situation
	 */
	bool 		in_takeoff_situation();

	/**
	 * Do takeoff help when in altitude controlled modes
	 * @param hold_altitude altitude setpoint for controller
	 * @param pitch_limit_min minimum pitch allowed
	 */
	void 		do_takeoff_help(float *hold_altitude, float *pitch_limit_min);

	/**
	 * Update desired altitude base on user pitch stick input
	 *
	 * @param dt Time step
	 * @return true if climbout mode was requested by user (climb with max rate and min airspeed)
	 */
	bool		update_desired_altitude(float dt);

	/**
	 * Control position.
	 */
	bool		control_position(const math::Vector<2> &global_pos, const math::Vector<3> &ground_speed,
					 const struct position_setpoint_triplet_s &_pos_sp_triplet);

	float		get_tecs_pitch();
	float		get_tecs_thrust();

	float		get_demanded_airspeed();
	float		calculate_target_airspeed(float airspeed_demand);
	void		calculate_gndspeed_undershoot(const math::Vector<2> &current_position, const math::Vector<2> &ground_speed_2d,
			const struct position_setpoint_triplet_s &pos_sp_triplet);

	/**
	 * Handle incoming vehicle commands
	 */
	void		handle_command();

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();

	/*
	 * Reset takeoff state
	 */
	void		reset_takeoff_state();

	/*
	 * Reset landing state
	 */
	void		reset_landing_state();

	/*
	 * Call TECS : a wrapper function to call the TECS implementation
	 */
	void tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
					float pitch_min_rad, float pitch_max_rad,
					float throttle_min, float throttle_max, float throttle_cruise,
					bool climbout_mode, float climbout_pitch_min_rad,
					float altitude,
					const math::Vector<3> &ground_speed,
					unsigned mode = tecs_status_s::TECS_MODE_NORMAL);

};

namespace l1_control
{

FixedwingPositionControl	*g_control = nullptr;
}

FixedwingPositionControl::FixedwingPositionControl() :

	_mavlink_log_pub(nullptr),
	_task_should_exit(false),
	_task_running(false),

	/* subscriptions */
	_global_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_ctrl_state_sub(-1),
	_control_mode_sub(-1),
	_vehicle_command_sub(-1),
	_vehicle_status_sub(-1),
	_vehicle_land_detected_sub(-1),
	_params_sub(-1),
	_manual_control_sub(-1),
	_sensor_combined_sub(-1),

	/* publications */
	_attitude_sp_pub(nullptr),
	_tecs_status_pub(nullptr),
	_fw_pos_ctrl_status_pub(nullptr),

	/* publication ID */
	_attitude_setpoint_id(0),

	/* states */
	_ctrl_state(),
	_att_sp(),
	_fw_pos_ctrl_status(),
	_manual(),
	_control_mode(),
	_vehicle_command(),
	_vehicle_status(),
	_vehicle_land_detected(),
	_global_pos(),
	_pos_sp_triplet(),
	_sensor_combined(),

	/* performance counters */
	_loop_perf(perf_alloc(PC_ELAPSED, "fw l1 control")),

	_hold_alt(0.0f),
	_takeoff_ground_alt(0.0f),
	_hdg_hold_yaw(0.0f),
	_hdg_hold_enabled(false),
	_yaw_lock_engaged(false),
	_althold_epv(0.0f),
	_was_in_deadband(false),
	_hdg_hold_prev_wp{},
	_hdg_hold_curr_wp{},
	_control_position_last_called(0),
	_land_noreturn_horizontal(false),
	_land_noreturn_vertical(false),
	_land_stayonground(false),
	_land_motor_lim(false),
	_land_onslope(false),
	_land_useterrain(false),
	_landingslope(),
	_time_started_landing(0),
	_t_alt_prev_valid(0),
	_time_last_t_alt(0),
	_flare_height(0.0f),
	_flare_curve_alt_rel_last(0.0f),
	_target_bearing(0.0f),
	_was_in_air(false),
	_time_went_in_air(0),
	_launchDetector(),
	_launch_detection_state(LAUNCHDETECTION_RES_NONE),
	_runway_takeoff(),
	_last_manual(false),
	_airspeed_error(0.0f),
	_airspeed_valid(false),
	_airspeed_last_received(0),
	_groundspeed_undershoot(0.0f),
	_global_pos_valid(false),
	_R_nb(),
	_roll(0.0f),
	_pitch(0.0f),
	_yaw(0.0f),
	_reinitialize_tecs(true),
	_is_tecs_running(false),
	_last_tecs_update(0.0f),
	_asp_after_transition(0.0f),
	_was_in_transition(false),
	_pos_reset_counter(0),
	_alt_reset_counter(0),
	_l1_control(),
	_tecs(),
	_control_mode_current(FW_POSCTRL_MODE_OTHER),
	_parameters(),
	_parameter_handles()
{
	_fw_pos_ctrl_status = {};

	_parameter_handles.l1_period = param_find("FW_L1_PERIOD");
	_parameter_handles.l1_damping = param_find("FW_L1_DAMPING");

	_parameter_handles.airspeed_min = param_find("FW_AIRSPD_MIN");
	_parameter_handles.airspeed_trim = param_find("FW_AIRSPD_TRIM");
	_parameter_handles.airspeed_max = param_find("FW_AIRSPD_MAX");
	_parameter_handles.airspeed_trans = param_find("VT_ARSP_TRANS");
	_parameter_handles.airspeed_mode = param_find("FW_ARSP_MODE");

	_parameter_handles.pitch_limit_min = param_find("FW_P_LIM_MIN");
	_parameter_handles.pitch_limit_max = param_find("FW_P_LIM_MAX");
	_parameter_handles.roll_limit = param_find("FW_R_LIM");
	_parameter_handles.throttle_min = param_find("FW_THR_MIN");
	_parameter_handles.throttle_max = param_find("FW_THR_MAX");
	_parameter_handles.throttle_idle = param_find("FW_THR_IDLE");
	_parameter_handles.throttle_slew_max = param_find("FW_THR_SLEW_MAX");
	_parameter_handles.throttle_cruise = param_find("FW_THR_CRUISE");
	_parameter_handles.throttle_land_max = param_find("FW_THR_LND_MAX");
	_parameter_handles.man_roll_max_deg = param_find("FW_MAN_R_MAX");
	_parameter_handles.man_pitch_max_deg = param_find("FW_MAN_P_MAX");
	_parameter_handles.rollsp_offset_deg = param_find("FW_RSP_OFF");
	_parameter_handles.pitchsp_offset_deg = param_find("FW_PSP_OFF");

	_parameter_handles.land_slope_angle = param_find("FW_LND_ANG");
	_parameter_handles.land_H1_virt = param_find("FW_LND_HVIRT");
	_parameter_handles.land_flare_alt_relative = param_find("FW_LND_FLALT");
	_parameter_handles.land_flare_pitch_min_deg = param_find("FW_LND_FL_PMIN");
	_parameter_handles.land_flare_pitch_max_deg = param_find("FW_LND_FL_PMAX");
	_parameter_handles.land_thrust_lim_alt_relative = param_find("FW_LND_TLALT");
	_parameter_handles.land_heading_hold_horizontal_distance = param_find("FW_LND_HHDIST");
	_parameter_handles.land_use_terrain_estimate = param_find("FW_LND_USETER");
	_parameter_handles.land_airspeed_scale = param_find("FW_LND_AIRSPD_SC");

	_parameter_handles.time_const = 			param_find("FW_T_TIME_CONST");
	_parameter_handles.time_const_throt = 			param_find("FW_T_THRO_CONST");
	_parameter_handles.min_sink_rate = 			param_find("FW_T_SINK_MIN");
	_parameter_handles.max_sink_rate =			param_find("FW_T_SINK_MAX");
	_parameter_handles.max_climb_rate =			param_find("FW_T_CLMB_MAX");
	_parameter_handles.climbout_diff =			param_find("FW_CLMBOUT_DIFF");
	_parameter_handles.throttle_damp = 			param_find("FW_T_THR_DAMP");
	_parameter_handles.integrator_gain =			param_find("FW_T_INTEG_GAIN");
	_parameter_handles.vertical_accel_limit =		param_find("FW_T_VERT_ACC");
	_parameter_handles.height_comp_filter_omega =		param_find("FW_T_HGT_OMEGA");
	_parameter_handles.speed_comp_filter_omega =		param_find("FW_T_SPD_OMEGA");
	_parameter_handles.roll_throttle_compensation = 	param_find("FW_T_RLL2THR");
	_parameter_handles.speed_weight = 			param_find("FW_T_SPDWEIGHT");
	_parameter_handles.pitch_damping = 			param_find("FW_T_PTCH_DAMP");
	_parameter_handles.heightrate_p =			param_find("FW_T_HRATE_P");
	_parameter_handles.heightrate_ff =			param_find("FW_T_HRATE_FF");
	_parameter_handles.speedrate_p =			param_find("FW_T_SRATE_P");
	_parameter_handles.vtol_type = 				param_find("VT_TYPE");

	/* fetch initial parameter values */
	parameters_update();
}

FixedwingPositionControl::~FixedwingPositionControl()
{
	if (_control_task != -1) {

		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	l1_control::g_control = nullptr;
}

int
FixedwingPositionControl::parameters_update()
{

	/* L1 control parameters */
	param_get(_parameter_handles.l1_damping, &(_parameters.l1_damping));
	param_get(_parameter_handles.l1_period, &(_parameters.l1_period));

	param_get(_parameter_handles.airspeed_min, &(_parameters.airspeed_min));
	param_get(_parameter_handles.airspeed_trim, &(_parameters.airspeed_trim));
	param_get(_parameter_handles.airspeed_max, &(_parameters.airspeed_max));
	param_get(_parameter_handles.airspeed_trans, &(_parameters.airspeed_trans));
	param_get(_parameter_handles.airspeed_mode, &(_parameters.airspeed_mode));

	param_get(_parameter_handles.pitch_limit_min, &(_parameters.pitch_limit_min));
	param_get(_parameter_handles.pitch_limit_max, &(_parameters.pitch_limit_max));
	param_get(_parameter_handles.roll_limit, &(_parameters.roll_limit));
	param_get(_parameter_handles.throttle_min, &(_parameters.throttle_min));
	param_get(_parameter_handles.throttle_max, &(_parameters.throttle_max));
	param_get(_parameter_handles.throttle_idle, &(_parameters.throttle_idle));
	param_get(_parameter_handles.throttle_cruise, &(_parameters.throttle_cruise));
	param_get(_parameter_handles.throttle_slew_max, &(_parameters.throttle_slew_max));

	param_get(_parameter_handles.throttle_land_max, &(_parameters.throttle_land_max));

	param_get(_parameter_handles.man_roll_max_deg, &_parameters.man_roll_max_rad);
	_parameters.man_roll_max_rad = math::radians(_parameters.man_roll_max_rad);
	param_get(_parameter_handles.man_pitch_max_deg, &_parameters.man_pitch_max_rad);
	_parameters.man_pitch_max_rad = math::radians(_parameters.man_pitch_max_rad);
	param_get(_parameter_handles.rollsp_offset_deg, &_parameters.rollsp_offset_rad);
	_parameters.rollsp_offset_rad = math::radians(_parameters.rollsp_offset_rad);
	param_get(_parameter_handles.pitchsp_offset_deg, &_parameters.pitchsp_offset_rad);
	_parameters.pitchsp_offset_rad = math::radians(_parameters.pitchsp_offset_rad);


	param_get(_parameter_handles.time_const, &(_parameters.time_const));
	param_get(_parameter_handles.time_const_throt, &(_parameters.time_const_throt));
	param_get(_parameter_handles.min_sink_rate, &(_parameters.min_sink_rate));
	param_get(_parameter_handles.max_sink_rate, &(_parameters.max_sink_rate));
	param_get(_parameter_handles.throttle_damp, &(_parameters.throttle_damp));
	param_get(_parameter_handles.integrator_gain, &(_parameters.integrator_gain));
	param_get(_parameter_handles.vertical_accel_limit, &(_parameters.vertical_accel_limit));
	param_get(_parameter_handles.height_comp_filter_omega, &(_parameters.height_comp_filter_omega));
	param_get(_parameter_handles.speed_comp_filter_omega, &(_parameters.speed_comp_filter_omega));
	param_get(_parameter_handles.roll_throttle_compensation, &(_parameters.roll_throttle_compensation));
	param_get(_parameter_handles.speed_weight, &(_parameters.speed_weight));
	param_get(_parameter_handles.pitch_damping, &(_parameters.pitch_damping));
	param_get(_parameter_handles.max_climb_rate, &(_parameters.max_climb_rate));
	param_get(_parameter_handles.climbout_diff, &(_parameters.climbout_diff));

	param_get(_parameter_handles.heightrate_p, &(_parameters.heightrate_p));
	param_get(_parameter_handles.heightrate_ff, &(_parameters.heightrate_ff));
	param_get(_parameter_handles.speedrate_p, &(_parameters.speedrate_p));

	param_get(_parameter_handles.land_slope_angle, &(_parameters.land_slope_angle));
	param_get(_parameter_handles.land_H1_virt, &(_parameters.land_H1_virt));
	param_get(_parameter_handles.land_flare_alt_relative, &(_parameters.land_flare_alt_relative));
	param_get(_parameter_handles.land_thrust_lim_alt_relative, &(_parameters.land_thrust_lim_alt_relative));

	/* check if negative value for 2/3 of flare altitude is set for throttle cut */
	if (_parameters.land_thrust_lim_alt_relative < 0.0f) {
		_parameters.land_thrust_lim_alt_relative = 0.66f * _parameters.land_flare_alt_relative;
	}

	param_get(_parameter_handles.land_heading_hold_horizontal_distance,
		  &(_parameters.land_heading_hold_horizontal_distance));
	param_get(_parameter_handles.land_flare_pitch_min_deg, &(_parameters.land_flare_pitch_min_deg));
	param_get(_parameter_handles.land_flare_pitch_max_deg, &(_parameters.land_flare_pitch_max_deg));
	param_get(_parameter_handles.land_use_terrain_estimate, &(_parameters.land_use_terrain_estimate));
	param_get(_parameter_handles.land_airspeed_scale, &(_parameters.land_airspeed_scale));
	param_get(_parameter_handles.vtol_type, &(_parameters.vtol_type));

	_l1_control.set_l1_damping(_parameters.l1_damping);
	_l1_control.set_l1_period(_parameters.l1_period);
	_l1_control.set_l1_roll_limit(math::radians(_parameters.roll_limit));

	_tecs.set_time_const(_parameters.time_const);
	_tecs.set_time_const_throt(_parameters.time_const_throt);
	_tecs.set_min_sink_rate(_parameters.min_sink_rate);
	_tecs.set_max_sink_rate(_parameters.max_sink_rate);
	_tecs.set_throttle_damp(_parameters.throttle_damp);
	_tecs.set_throttle_slewrate(_parameters.throttle_slew_max);
	_tecs.set_integrator_gain(_parameters.integrator_gain);
	_tecs.set_vertical_accel_limit(_parameters.vertical_accel_limit);
	_tecs.set_height_comp_filter_omega(_parameters.height_comp_filter_omega);
	_tecs.set_speed_comp_filter_omega(_parameters.speed_comp_filter_omega);
	_tecs.set_roll_throttle_compensation(_parameters.roll_throttle_compensation);
	_tecs.set_speed_weight(_parameters.speed_weight);
	_tecs.set_pitch_damping(_parameters.pitch_damping);
	_tecs.set_indicated_airspeed_min(_parameters.airspeed_min);
	_tecs.set_indicated_airspeed_max(_parameters.airspeed_max);
	_tecs.set_max_climb_rate(_parameters.max_climb_rate);
	_tecs.set_heightrate_p(_parameters.heightrate_p);
	_tecs.set_heightrate_ff(_parameters.heightrate_ff);
	_tecs.set_speedrate_p(_parameters.speedrate_p);

	/* sanity check parameters */
	if (_parameters.airspeed_max < _parameters.airspeed_min ||
	    _parameters.airspeed_max < 5.0f ||
	    _parameters.airspeed_min > 100.0f ||
	    _parameters.airspeed_trim < _parameters.airspeed_min ||
	    _parameters.airspeed_trim > _parameters.airspeed_max) {
		warnx("error: airspeed parameters invalid");
		return 1;
	}

	/* Update the landing slope */
	_landingslope.update(math::radians(_parameters.land_slope_angle), _parameters.land_flare_alt_relative,
			     _parameters.land_thrust_lim_alt_relative, _parameters.land_H1_virt);

	/* Update and publish the navigation capabilities */
	_fw_pos_ctrl_status.landing_slope_angle_rad = _landingslope.landing_slope_angle_rad();
	_fw_pos_ctrl_status.landing_horizontal_slope_displacement = _landingslope.horizontal_slope_displacement();
	_fw_pos_ctrl_status.landing_flare_length = _landingslope.flare_length();
	fw_pos_ctrl_status_publish();

	/* Update Launch Detector Parameters */
	_launchDetector.updateParams();

	_runway_takeoff.updateParams();

	return OK;
}

void
FixedwingPositionControl::vehicle_control_mode_poll()
{
	bool updated;

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}
}

void
FixedwingPositionControl::vehicle_command_poll()
{
	bool updated;

	orb_check(_vehicle_command_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_command), _vehicle_command_sub, &_vehicle_command);
		handle_command();
	}
}

void
FixedwingPositionControl::vehicle_status_poll()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(fw_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}
}

void
FixedwingPositionControl::vehicle_land_detected_poll()
{
	bool updated;

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}
}

bool
FixedwingPositionControl::vehicle_manual_control_setpoint_poll()
{
	bool manual_updated;

	/* Check if manual setpoint has changed */
	orb_check(_manual_control_sub, &manual_updated);

	if (manual_updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_control_sub, &_manual);
	}

	return manual_updated;
}

void
FixedwingPositionControl::control_state_poll()
{
	/* check if there is a new position */
	bool ctrl_state_updated;
	orb_check(_ctrl_state_sub, &ctrl_state_updated);

	if (ctrl_state_updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);
		_airspeed_valid = _ctrl_state.airspeed_valid;
		_airspeed_last_received = hrt_absolute_time();

	} else {

		/* no airspeed updates for one second */
		if (_airspeed_valid && (hrt_absolute_time() - _airspeed_last_received) > 1e6) {
			_airspeed_valid = false;
		}
	}

	/* set rotation matrix and euler angles */
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
	_R_nb = q_att.to_dcm();

	math::Vector<3> euler_angles;
	euler_angles = _R_nb.to_euler();
	_roll    = euler_angles(0);
	_pitch   = euler_angles(1);
	_yaw     = euler_angles(2);

	/* update TECS state */
	_tecs.enable_airspeed(_airspeed_valid);
}

void
FixedwingPositionControl::vehicle_sensor_combined_poll()
{
	/* check if there is a new position */
	bool sensors_updated;
	orb_check(_sensor_combined_sub, &sensors_updated);

	if (sensors_updated) {
		orb_copy(ORB_ID(sensor_combined), _sensor_combined_sub, &_sensor_combined);
	}
}

void
FixedwingPositionControl::vehicle_setpoint_poll()
{
	/* check if there is a new setpoint */
	bool pos_sp_triplet_updated;
	orb_check(_pos_sp_triplet_sub, &pos_sp_triplet_updated);

	if (pos_sp_triplet_updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}
}

void
FixedwingPositionControl::task_main_trampoline(int argc, char *argv[])
{
	l1_control::g_control = new FixedwingPositionControl();

	if (l1_control::g_control == nullptr) {
		warnx("OUT OF MEM");
		return;
	}

	/* only returns on exit */
	l1_control::g_control->task_main();
	delete l1_control::g_control;
	l1_control::g_control = nullptr;
}


// 控制过程梳理六（可本文件搜索，查看处理过程）
//　下面是讲述如何将油门杆控制的是期望空速
float
FixedwingPositionControl::get_demanded_airspeed()
{
	float altctrl_airspeed = 0;

	// neutral throttle corresponds to trim airspeed
	if (_manual.z < 0.5f) {
		// lower half of throttle is min to trim airspeed
		//可带入０　0.5辅助理解，这里是把油门杆下一半杆量　映射为空速最小值　到巡航空速
		altctrl_airspeed = _parameters.airspeed_min +
				   (_parameters.airspeed_trim - _parameters.airspeed_min) *
				   _manual.z * 2;

	} else {
		// upper half of throttle is trim to max airspeed
		//可带入１　0.5辅助理解，这里是把油门杆上一半杆量　映射为巡航空速　到空速最大值
		altctrl_airspeed = _parameters.airspeed_trim +
				   (_parameters.airspeed_max - _parameters.airspeed_trim) *
				   (_manual.z * 2 - 1);
	}

	return altctrl_airspeed;
}

float
FixedwingPositionControl::calculate_target_airspeed(float airspeed_demand)
{
	float airspeed;

	if (_airspeed_valid) {
		airspeed = _ctrl_state.airspeed;

	} else {
		airspeed = _parameters.airspeed_min + (_parameters.airspeed_max - _parameters.airspeed_min) / 2.0f;
	}

	/* cruise airspeed for all modes unless modified below */
	float target_airspeed = airspeed_demand;

	/* add minimum ground speed undershoot (only non-zero in presence of sufficient wind) */
	target_airspeed += _groundspeed_undershoot;

	if (0/* throttle nudging enabled */) {
		//target_airspeed += nudge term.
	}

	/* sanity check: limit to range */
	target_airspeed = math::constrain(target_airspeed, _parameters.airspeed_min, _parameters.airspeed_max);

	/* plain airspeed error */
	_airspeed_error = target_airspeed - airspeed;

	return target_airspeed;
}

void
FixedwingPositionControl::calculate_gndspeed_undershoot(const math::Vector<2> &current_position,
		const math::Vector<2> &ground_speed_2d, const struct position_setpoint_triplet_s &pos_sp_triplet)
{

	if (pos_sp_triplet.current.valid && !_l1_control.circle_mode()) {

		/* rotate ground speed vector with current attitude */
		math::Vector<2> yaw_vector(_R_nb(0, 0), _R_nb(1, 0));
		yaw_vector.normalize();
		float ground_speed_body = yaw_vector * ground_speed_2d;

		/* The minimum desired ground speed is the minimum airspeed projected on to the ground using the altitude and horizontal difference between the waypoints if available*/
		float distance = 0.0f;
		float delta_altitude = 0.0f;

		if (pos_sp_triplet.previous.valid) {
			distance = get_distance_to_next_waypoint(pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon,
					pos_sp_triplet.current.lat, pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt - pos_sp_triplet.previous.alt;

		} else {
			distance = get_distance_to_next_waypoint(current_position(0), current_position(1), pos_sp_triplet.current.lat,
					pos_sp_triplet.current.lon);
			delta_altitude = pos_sp_triplet.current.alt -  _global_pos.alt;
		}

		float ground_speed_desired = _parameters.airspeed_min * cosf(atan2f(delta_altitude, distance));


		/*
		 * Ground speed undershoot is the amount of ground velocity not reached
		 * by the plane. Consequently it is zero if airspeed is >= min ground speed
		 * and positive if airspeed < min ground speed.
		 *
		 * This error value ensures that a plane (as long as its throttle capability is
		 * not exceeded) travels towards a waypoint (and is not pushed more and more away
		 * by wind). Not countering this would lead to a fly-away.
		 */
		_groundspeed_undershoot = math::max(ground_speed_desired - ground_speed_body, 0.0f);

	} else {
		_groundspeed_undershoot = 0;
	}
}

void FixedwingPositionControl::fw_pos_ctrl_status_publish()
{
	_fw_pos_ctrl_status.timestamp = hrt_absolute_time();

	if (_fw_pos_ctrl_status_pub != nullptr) {
		orb_publish(ORB_ID(fw_pos_ctrl_status), _fw_pos_ctrl_status_pub, &_fw_pos_ctrl_status);

	} else {
		_fw_pos_ctrl_status_pub = orb_advertise(ORB_ID(fw_pos_ctrl_status), &_fw_pos_ctrl_status);
	}
}

void FixedwingPositionControl::get_waypoint_heading_distance(float heading, float distance,
		struct position_setpoint_s &waypoint_prev, struct position_setpoint_s &waypoint_next, bool flag_init)
{
	waypoint_prev.valid = true;
	waypoint_prev.alt = _hold_alt;
	position_setpoint_s temp_next {};
	position_setpoint_s temp_prev {};

	//第一次锁航向，进入到下面这个函数：函数释义：以这个航向　这个距离　创建航点
	if (flag_init) {
		//以当前的位置　当前航向的反方向heading+180.0f　向后创建一个距离100米的 waypoint_prev航点
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading + 180.0f * M_DEG_TO_RAD_F ,
						   HDG_HOLD_SET_BACK_DIST,
						   &temp_prev.lat, &temp_prev.lon);

		//以当前的位置　当前航向heading　向前创建一个距离3000米的 waypoint_next航点,这两个航点就是飞机调用L1实现锁航向
		waypoint_from_heading_and_distance(_global_pos.lat, _global_pos.lon, heading, HDG_HOLD_DIST_NEXT,
						   &temp_next.lat, &temp_next.lon);
		waypoint_prev = temp_prev;
		waypoint_next = temp_next;
		waypoint_next.valid = true;
		waypoint_next.alt = _hold_alt;

		return;//初始化造航点　造航线结束　return，再进来不需要初始化航点就会跳过上面这一段进入到下面


	} 
	//上面是第一次进入锁航向的时候，以当前位置　当前航向　向后100米创建waypoint_prev，向前3000米　创建waypoint_next，以调用L1实现压航线锁航向
	//下面是飞机已经在锁航向状态中，距离前面的3000米的航点剩下只有1000米了，看来需要重新造航点　但是航线已经有了，只需要在线上造点就可以了
	else {
		// for previous waypoint use the one still in front of us but shift it such that it is
		// located on the desired flight path but HDG_HOLD_SET_BACK_DIST behind us
		//以curr_wp点　curr->prev方向，向后1100米造新点prev，其实就是在当前位置后面100米重新造点prev
		create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
						   HDG_HOLD_REACHED_DIST + HDG_HOLD_SET_BACK_DIST,
						   &temp_prev.lat, &temp_prev.lon);
	}

	waypoint_next.valid = true;
	//以curr_wp点　prev->curr方向，向后4000米造新点next，其实就是在当前位置后面5000米重新造点curr
	create_waypoint_from_line_and_dist(waypoint_next.lat, waypoint_next.lon, waypoint_prev.lat, waypoint_prev.lon,
					   -(HDG_HOLD_DIST_NEXT + HDG_HOLD_REACHED_DIST),
					   &temp_next.lat, &temp_next.lon);
	waypoint_prev = temp_prev;
	waypoint_next = temp_next;
	waypoint_next.alt = _hold_alt;
}

float FixedwingPositionControl::get_terrain_altitude_landing(float land_setpoint_alt,
		const struct vehicle_global_position_s &global_pos)
{
	if (!PX4_ISFINITE(global_pos.terrain_alt)) {
		return land_setpoint_alt;
	}

	/* Decide if the terrain estimation can be used, once we switched to using the terrain we stick with it
	 * for the whole landing */
	if (_parameters.land_use_terrain_estimate && global_pos.terrain_alt_valid) {
		if (!_land_useterrain) {
			mavlink_log_info(&_mavlink_log_pub, "Landing, using terrain estimate");
			_land_useterrain = true;
		}

		return global_pos.terrain_alt;

	} else {
		return land_setpoint_alt;
	}
}

float FixedwingPositionControl::get_terrain_altitude_takeoff(float takeoff_alt,
		const struct vehicle_global_position_s &global_pos)
{
	if (PX4_ISFINITE(global_pos.terrain_alt) && global_pos.terrain_alt_valid) {
		return global_pos.terrain_alt;
	}

	return takeoff_alt;
}


// 控制过程梳理七（可本文件搜索，查看处理过程）
//　下面是讲述如何将pitch杆量转换为高度爬升率
bool FixedwingPositionControl::update_desired_altitude(float dt)
{
	const float deadBand = 0.06f;  //杆量范围[-1,+1],杆量死区中位点上下３％

	//由于死区需要的缩放 The correct scaling of the complete range needs to account for the missing part of the slope due to the deadband
	const float factor = 1.0f - deadBand;

	/* Climbout mode sets maximum throttle and pitch up */
	bool climbout_mode = false;

	/*
	 * 当不确定性变化显著时，就是高度明显变化，则将当前高度赋值为需要保持的高度
	 * Reset the hold altitude to the current altitude if the uncertainty　changes significantly.
	 * This is to guard against uncommanded altitude changes　when the altitude certainty increases or decreases.
	 */

	if (fabsf(_althold_epv - _global_pos.epv) > ALTHOLD_EPV_RESET_THRESH) {
		_hold_alt = _global_pos.alt;
		_althold_epv = _global_pos.epv;
	}

	/*
	 * Manual control has as convention the rotation around an axis. 
	 * Positive X means to rotate positively around the X axis in NED frame, which is pitching down
	 */

	//如果摇杆大于死区，当打杆时高度持续叠加积分量，并且杆量越大dt时间内高度叠加越多，所以可以理解为“爬升率”：dt时间内爬升的高度
	//注意下面正负号，pitch这个杆是反的，下面实际是在降落
	if (_manual.x > deadBand) {
		/* pitching down */
		float pitch = -(_manual.x - deadBand) / factor;
		_hold_alt += (_parameters.max_sink_rate * dt) * pitch;
		_was_in_deadband = false;

	} 
	//如果摇杆死区之外，当打杆时高度持续叠加积分量，并且杆量越大dt时间内高度叠加越多，所以可以理解为“爬升率”：dt时间内爬升的高度
	else if (_manual.x < - deadBand) {
		/* pitching up */
		float pitch = -(_manual.x + deadBand) / factor;
		_hold_alt += (_parameters.max_climb_rate * dt) * pitch;
		_was_in_deadband = false; //摇杆不在死区　
		climbout_mode = (pitch > MANUAL_THROTTLE_CLIMBOUT_THRESH);
	} 
	//摇杆虽然上一次不在死区，但是现在在死区了
	else if (!_was_in_deadband) {
		/* store altitude at which manual.x was inside deadBand
		 * The aircraft should immediately try to fly at this altitude
		 * as this is what the pilot expects when he moves the stick to the center */
		_hold_alt = _global_pos.alt;//保存当前高度
		_althold_epv = _global_pos.epv;//高度方向的精度
		_was_in_deadband = true;
	}

	if (_vehicle_status.is_vtol) {
		if (_vehicle_status.is_rotary_wing || _vehicle_status.in_transition_mode) {
			_hold_alt = _global_pos.alt;
		}
	}

	return climbout_mode;//是否处于爬升模式，决定与pitch爬升的杆量
}

bool FixedwingPositionControl::in_takeoff_situation()
{
	// in air for < 10s
	const hrt_abstime delta_takeoff = 10000000;

	if (hrt_elapsed_time(&_time_went_in_air) < delta_takeoff
	    && _global_pos.alt <= _takeoff_ground_alt + _parameters.climbout_diff) {

		return true;
	}

	return false;
}

void FixedwingPositionControl::do_takeoff_help(float *hold_altitude, float *pitch_limit_min)
{
	/* demand "climbout_diff" m above ground if user switched into this mode during takeoff */
	if (in_takeoff_situation()) {
		*hold_altitude = _takeoff_ground_alt + _parameters.climbout_diff;
		*pitch_limit_min = math::radians(10.0f);

	} else {
		*pitch_limit_min = _parameters.pitch_limit_min;
	}
}




















// 控制过程梳理三（可本文件搜索，查看处理过程）
// 位置控制模块功能：调用L1和TECS计算att_sp, 最后正确结果 publish(_attitude_setpoint_id
// 输入参数为：1. 当前位置；2. 当前地速； 3. 当前的期望航点
// 输出att_sp是否计算成功，成功则发布，不成功直接跳出。
// 如果是ALT POS AUTO都会进入这个函数计算并产生att_sp，返回setpoint　=　true发布att_sp。
// 手动控制STAB,MANUAL，return false跳出位置控制不发布att_sp，而是进入到att_control在哪里产生att_sp.

bool
FixedwingPositionControl::control_position(const math::Vector<2> &current_position, const math::Vector<3> &ground_speed,
		const struct position_setpoint_triplet_s &pos_sp_triplet)
{
	float dt = 0.01; // Using non zero value to a avoid division by zero

	if (_control_position_last_called > 0) {
		dt = (float)hrt_elapsed_time(&_control_position_last_called) * 1e-6f;
	}

	_control_position_last_called = hrt_absolute_time();

	//只有固定翼或垂起固定翼才需要运行这里，其他的如旋翼，return false跳过位置控制这个模块
	if (_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode) {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;
		return false;
	}

	//这是函数返回值,决定是否发布使用att_sp.
	//他的赋值就只有两处,这里默认true使用位置控制计算的att_sp.另一处在下面手动控制下重置为false跳过位置控制进入fw_att_control.
	bool setpoint = true;
	 
	_att_sp.fw_control_yaw = false;		//变量在位置控制中赋值，在auto下的takeoff和land借助wheel控制航向,在姿态控制中使用，默认情况下，我们不需要wheel控制偏航
	_att_sp.apply_flaps = false;		// 襟翼默认不使用，只有在LAND时才会使用襟翼，但是在姿态控制那里我已经修改成默认使用
	float eas2tas = 1.0f; 				// XXX calculate actual number based on current measurements



	/* filter speed and altitude for controller */
	math::Vector<3> accel_body(_sensor_combined.accelerometer_m_s2);

	//尾座式的垂起 坐标系需要转换一下
	if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
		float tmp = accel_body(0);
		accel_body(0) = -accel_body(2);
		accel_body(2) = tmp;
	}

	//将体轴加速度转换到NED系中
	math::Vector<3> accel_earth = _R_nb * accel_body;


	/* tell TECS to update its state, but let it know when it cannot actually control the plane */
	bool in_air_alt_control = (!_vehicle_land_detected.landed &&
				   (_control_mode.flag_control_auto_enabled ||
				    _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_altitude_enabled));

	/*************************
	* 该函数为_tecs控制器的入口函数。
	* 此处，将当前得到的高度，空速，姿态，加速度等信息，传给_tecs控制器。
	* 在这个函数中，对每个方向的加速度、速度、位置（高度）的原始数据做滤波，并存储到相关的变量，随后做控制。
	**************************/
	_tecs.update_state(_global_pos.alt, _ctrl_state.airspeed, _R_nb,
			   accel_body, accel_earth, (_global_pos.timestamp > 0), in_air_alt_control);

	


	/*************************
	* 如果当前的地速大于等于最小地速，那么groundspeed_undershoot为0；
	* 如果当前地速小于最小地速，那么这个groundspeed_undershoot为一个正值
	**************************/
	math::Vector<2> ground_speed_2d = {ground_speed(0), ground_speed(1)};
	calculate_gndspeed_undershoot(current_position, ground_speed_2d, pos_sp_triplet);


	// 当风速大于最大空速时会导致l1崩溃　l1 navigation logic breaks down when wind speed exceeds max airspeed
	// 将空速投影计算地速　compute 2D groundspeed from airspeed-heading projection
	math::Vector<2> air_speed_2d = {_ctrl_state.airspeed * cosf(_yaw), _ctrl_state.airspeed * sinf(_yaw)};
	math::Vector<2> nav_speed_2d = {0, 0};

	//计算当前的空速跟地速的夹角,并给速度赋值。
	//如果此角度大于90°，或者地速特别小（可能风速特别大且逆风）,直接认为groundspeed为空速
	float air_gnd_angle = acosf((air_speed_2d * ground_speed_2d) / (air_speed_2d.length() * ground_speed_2d.length()));

	// if angle > 90 degrees or groundspeed is less than threshold, replace groundspeed with airspeed projection
	if ((fabsf(air_gnd_angle) > (float)M_PI) || (ground_speed_2d.length() < 3.0f)) {
		nav_speed_2d = air_speed_2d;

	} else {
		nav_speed_2d = ground_speed_2d;
	}


	/* define altitude error */
	float altitude_error = pos_sp_triplet.current.alt - _global_pos.alt;

	/* no throttle limit as default */
	float throttle_max = 1.0f;


	/* save time when airplane is in air */
	if (!_was_in_air && !_vehicle_land_detected.landed) {
		_was_in_air = true;
		_time_went_in_air = hrt_absolute_time();
		_takeoff_ground_alt = _global_pos.alt;
	}

	/* reset flag when airplane landed */
	if (_vehicle_land_detected.landed) {
		_was_in_air = false;
	}


	// 控制过程梳理十　AUTO模式
	//下面是AUTO自动模式的处理！包含正常的航点模式、自动起飞、自动降落

	if (_control_mode.flag_control_auto_enabled && 
	    pos_sp_triplet.current.valid) {
		/* AUTONOMOUS FLIGHT */

		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		_control_mode_current = FW_POSCTRL_MODE_AUTO;

		//如果是从非自动模式切换到这个模式，且不做任何输入，那么随后就保持切换时刻的高度和航向
		_hold_alt = _global_pos.alt;
		_hdg_hold_yaw = _yaw;

		/* get circle mode */
		bool was_circle_mode = _l1_control.circle_mode();

		/* restore speed weight, in case changed intermittently (e.g. in landing handling) */
		_tecs.set_speed_weight(_parameters.speed_weight);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> next_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* current waypoint (the one currently heading for) */
		math::Vector<2> curr_wp((float)pos_sp_triplet.current.lat, (float)pos_sp_triplet.current.lon);

		/* Initialize attitude controller integrator reset flags to 0 */
		_att_sp.roll_reset_integral = false;
		_att_sp.pitch_reset_integral = false;
		_att_sp.yaw_reset_integral = false;

		/* previous waypoint */
		math::Vector<2> prev_wp;

		if (pos_sp_triplet.previous.valid) {
			prev_wp(0) = (float)pos_sp_triplet.previous.lat;
			prev_wp(1) = (float)pos_sp_triplet.previous.lon;

		} else {
			/*
			 * No valid previous waypoint, go for the current wp.
			 * This is automatically handled by the L1 library.
			 */
			prev_wp(0) = (float)pos_sp_triplet.current.lat;
			prev_wp(1) = (float)pos_sp_triplet.current.lon;

		}

		// 自动模式的航点中包含可以设置的巡航速度和巡航油门，将其读取并存在相应的变量中
		float mission_airspeed = _parameters.airspeed_trim;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			mission_airspeed = _pos_sp_triplet.current.cruising_speed;
		}

		float mission_throttle = _parameters.throttle_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_throttle) &&
		    _pos_sp_triplet.current.cruising_throttle > 0.01f) {

			mission_throttle = _pos_sp_triplet.current.cruising_throttle;
		}


		//空闲模式，直接都给零
		if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
			_att_sp.thrust = 0.0f;
			_att_sp.roll_body = 0.0f;
			_att_sp.pitch_body = 0.0f;

		} 
		

		//航点模式，从_L1控制器中得到期望滚转和期望航向，从_tecs控制器得到期望油门和期望俯仰角
		else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION) {
			/* waypoint is a plain navigation waypoint */
			_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, nav_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			tecs_update_pitch_throttle(pos_sp_triplet.current.alt, calculate_target_airspeed(mission_airspeed), eas2tas,
						   math::radians(_parameters.pitch_limit_min), math::radians(_parameters.pitch_limit_max),
						   _parameters.throttle_min, _parameters.throttle_max, mission_throttle,
						   false, math::radians(_parameters.pitch_limit_min), _global_pos.alt, ground_speed);

		} 
		
		/*************************
		* 盘旋模式，从_L1控制器中得到期望滚转和期望航向，从_tecs控制器得到期望油门和期望俯仰角
		* 如果当前飞机处于起飞状态，即起飞就盘旋的状态，那么会对滚转角有个正负5°的限制，优先保证起飞再盘旋
		* 如果中断降落，那么正常情况下会再降落点上空盘旋，如果高度太低，那么会优先爬升起来再盘旋。
		**************************/
		else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) {

			/* waypoint is a loiter waypoint */
			_l1_control.navigate_loiter(curr_wp, current_position, pos_sp_triplet.current.loiter_radius,
						    pos_sp_triplet.current.loiter_direction, nav_speed_2d);
			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			/* 盘旋高度由航点设置决定 */
			float alt_sp = pos_sp_triplet.current.alt;

			if (in_takeoff_situation()) {
				alt_sp = math::max(alt_sp, _takeoff_ground_alt + _parameters.climbout_diff);
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-5.0f), math::radians(5.0f));
			}

			if (_fw_pos_ctrl_status.abort_landing) {
				if (pos_sp_triplet.current.alt - _global_pos.alt  < _parameters.climbout_diff) {
					// aborted landing complete, normal loiter over landing point
					_fw_pos_ctrl_status.abort_landing = false;

				} else {
					// continue straight until vehicle has sufficient altitude
					_att_sp.roll_body = 0.0f;
				}
			}

			tecs_update_pitch_throttle(alt_sp,
						   calculate_target_airspeed(mission_airspeed),
						   eas2tas,
						   math::radians(_parameters.pitch_limit_min),
						   math::radians(_parameters.pitch_limit_max),
						   _parameters.throttle_min,
						   _parameters.throttle_max,
						   _parameters.throttle_cruise,
						   false,
						   math::radians(_parameters.pitch_limit_min),
						   _global_pos.alt,
						   ground_speed);

		} 
		/*************************
		*
		* 自动降落模式，从_L1控制器中得到期望滚转和期望航向，从_tecs控制器得到期望油门和期望俯仰角
		* 首先在降落模式的时候，会启用副翼或升降舵手动控制，然后分为三个阶段，进行水平和垂直两个方向的控制。
		* 1. 在距离降落点足够远、高度足够高的时候，进行正常的航点控制，会使飞机不断的降低高度，同时逼近理想航线与期望航向（现实中多为降落跑道）；
		* 2. 水平方向上，接近降落点的时候，即 wp_distance < _parameters.land_heading_hold_horizontal_distance 条件满足，
		*    我们认为飞机已经在跑道上方且方向正确！那么开始保持航向，调用navigate_heading函数，横向控制只为保持航向。
		* 3. 垂直方向上，高度足够高的时候，通过landingslope得到期望高度，保持飞机高度线性的下降。
		* 4. 当高度低于 flare_relative_alt 并且水平距离小于 flare_length + 5.0f 的时候，那么认为飞机开始滑翔！
		* 5. 滑翔的时候，飞机水平方向只保持航向，滚转角限制到很小的范围，同时可以手动操作方向舵或者轮子来控制航向。
		* 6. 滑翔过程中的期望高度，通过FlareCurve得到，期望高度随着与降落点的水平距离减小，以幂指数降低。
		* 7. 滑翔过程中，一旦检测到期望高度增大（异常情况），那么会将期望高度设置为零，防止出现意外的复飞。
		*
		**************************/
		else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			// apply full flaps for landings. this flag will also trigger the use of flaperons
			// if they have been enabled using the corresponding parameter
			_att_sp.apply_flaps = true;

			// save time at which we started landing and reset abort_landing
			if (_time_started_landing == 0) {
				_time_started_landing = hrt_absolute_time();

				_fw_pos_ctrl_status.abort_landing = false;
			}

			float bearing_lastwp_currwp = get_bearing_to_next_waypoint(prev_wp(0), prev_wp(1), curr_wp(0), curr_wp(1));
			float bearing_airplane_currwp = get_bearing_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
							curr_wp(1));

			/* Horizontal landing control */
			/* switch to heading hold for the last meters, continue heading hold after */
			float wp_distance = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0), curr_wp(1));

			/* calculate a waypoint distance value which is 0 when the aircraft is behind the waypoint */
			float wp_distance_save = wp_distance;

			if (fabsf(bearing_airplane_currwp - bearing_lastwp_currwp) >= math::radians(90.0f)) {
				wp_distance_save = 0.0f;
			}

			// create virtual waypoint which is on the desired flight path but
			// some distance behind landing waypoint. This will make sure that the plane
			// will always follow the desired flight path even if we get close or past
			// the landing waypoint
			/* 没用到 */
			math::Vector<2> curr_wp_shifted;
			double lat;
			double lon;
			create_waypoint_from_line_and_dist(pos_sp_triplet.current.lat, pos_sp_triplet.current.lon,
							   pos_sp_triplet.previous.lat, pos_sp_triplet.previous.lon, -1000.0f, &lat, &lon);
			curr_wp_shifted(0) = (float)lat;
			curr_wp_shifted(1) = (float)lon;

			// we want the plane to keep tracking the desired flight path until we start flaring
			// if we go into heading hold mode earlier then we risk to be pushed away from the runway by cross winds
			//if (land_noreturn_vertical) {
			if (wp_distance < _parameters.land_heading_hold_horizontal_distance || _land_noreturn_horizontal) {

				/* heading hold, along the line connecting this and the last waypoint */

				if (!_land_noreturn_horizontal) {
					// set target_bearing in first occurrence
					if (pos_sp_triplet.previous.valid) {
						_target_bearing = bearing_lastwp_currwp;

					} else {
						_target_bearing = _yaw;
					}

					mavlink_log_info(&_mavlink_log_pub, "#Landing, heading hold");
				}

    			//warnx("NORET: %d, target_bearing: %d, yaw: %d", (int)land_noreturn_horizontal, (int)math::degrees(target_bearing), (int)math::degrees(_yaw));

				_l1_control.navigate_heading(_target_bearing, _yaw, nav_speed_2d);

				_land_noreturn_horizontal = true;

			} else {

				/* normal navigation */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, nav_speed_2d);
			}

			_att_sp.roll_body = _l1_control.nav_roll();
			_att_sp.yaw_body = _l1_control.nav_bearing();

			if (_land_noreturn_horizontal) {
				/* limit roll motion to prevent wings from touching the ground first */
				_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-10.0f), math::radians(10.0f));
			}

			/* Vertical landing control */
			//xxx: using the tecs altitude controller for slope control for now
			/* apply minimum pitch (flare) and limit roll if close to touch down, altitude error is negative (going down) */
			// XXX this could make a great param

			float throttle_land = _parameters.throttle_min + (_parameters.throttle_max - _parameters.throttle_min) * 0.1f;
			float airspeed_land = _parameters.land_airspeed_scale * _parameters.airspeed_min;
			float airspeed_approach = _parameters.land_airspeed_scale * _parameters.airspeed_min;

			/* Get an estimate of the terrain altitude if available, otherwise terrain_alt will be
			 * equal to _pos_sp_triplet.current.alt */
			float terrain_alt;

			/* 如果有测距模块等额外的设备，距离地面的高度可以直接得到，正常没有的情况下，地面高度由降落点的高度替代 */
			if (_parameters.land_use_terrain_estimate) {
				if (_global_pos.terrain_alt_valid) {
					// all good, have valid terrain altitude
					terrain_alt = _global_pos.terrain_alt;
					_t_alt_prev_valid = terrain_alt;
					_time_last_t_alt = hrt_absolute_time();

				} else if (_time_last_t_alt == 0) {
					// we have started landing phase but don't have valid terrain
					// wait for some time, maybe we will soon get a valid estimate
					// until then just use the altitude of the landing waypoint
					if (hrt_elapsed_time(&_time_started_landing) < 10 * 1000 * 1000) {
						terrain_alt = pos_sp_triplet.current.alt;

					} else {
						// still no valid terrain, abort landing
						terrain_alt = pos_sp_triplet.current.alt;
						_fw_pos_ctrl_status.abort_landing = true;
					}

				} else if ((!_global_pos.terrain_alt_valid && hrt_elapsed_time(&_time_last_t_alt) < T_ALT_TIMEOUT * 1000 * 1000)
					   || _land_noreturn_vertical) {
					// use previous terrain estimate for some time and hope to recover
					// if we are already flaring (land_noreturn_vertical) then just
					//  go with the old estimate
					terrain_alt = _t_alt_prev_valid;

				} else {
					// terrain alt was not valid for long time, abort landing
					terrain_alt = _t_alt_prev_valid;
					_fw_pos_ctrl_status.abort_landing = true;
				}

			} else {
				// no terrain estimation, just use landing waypoint altitude
				terrain_alt = pos_sp_triplet.current.alt;
			}


			/* Calculate distance (to landing waypoint) and altitude of last ordinary waypoint L */
			float L_altitude_rel = pos_sp_triplet.previous.valid ?
					       pos_sp_triplet.previous.alt - terrain_alt : 0.0f;

			/* 根据当前距离降落点的距离，计算得到期望的高度，期望高度随着距离减小线性减小 */
			float landing_slope_alt_rel_desired = _landingslope.getLandingSlopeRelativeAltitudeSave(wp_distance,
							      bearing_lastwp_currwp, bearing_airplane_currwp);

			/* Check if we should start flaring with a vertical and a
			 * horizontal limit (with some tolerance)
			 * The horizontal limit is only applied when we are in front of the wp
			 */
			/* 高度和水平距离都小于阈值，进入滑翔状态！ */
			if (((_global_pos.alt < terrain_alt + _landingslope.flare_relative_alt()) &&
			     (wp_distance_save < _landingslope.flare_length() + 5.0f)) ||
			    _land_noreturn_vertical) {  //checking for land_noreturn to avoid unwanted climb out

				/* land with minimal speed */

				/* force TECS to only control speed with pitch, altitude is only implicitly controlled now */
				// _tecs.set_speed_weight(2.0f);

				/* kill the throttle if param requests it */
				throttle_max = _parameters.throttle_max;

				/* enable direct yaw control using rudder/wheel */
				_att_sp.yaw_body = _target_bearing;
				_att_sp.fw_control_yaw = true;//变量在位置控制中赋值在姿态控制中使用，land时用wheel控制航向

				/* 小于一定高度，油门最大值也降低为 throttle_land_max，可在地面站中修改该参数 */
				if (_global_pos.alt < terrain_alt + _landingslope.motor_lim_relative_alt() || _land_motor_lim) {
					throttle_max = math::min(throttle_max, _parameters.throttle_land_max);

					if (!_land_motor_lim) {
						_land_motor_lim  = true;
						mavlink_log_info(&_mavlink_log_pub, "#Landing, limiting throttle");
					}

				}

				/* 计算期望高度，随水平距离减小指数降低 */
				float flare_curve_alt_rel = _landingslope.getFlareCurveRelativeAltitudeSave(wp_distance, bearing_lastwp_currwp,
							    bearing_airplane_currwp);

				/* avoid climbout */
				if ((_flare_curve_alt_rel_last < flare_curve_alt_rel && _land_noreturn_vertical) || _land_stayonground) {
					flare_curve_alt_rel = 0.0f; // stay on ground
					_land_stayonground = true;
				}

				/* 期望高度为flare_curve_alt_rel，随水平距离减小指数降低；
				 * 期望空速为最小空速 airspeed_land = _parameters.land_airspeed_scale * _parameters.airspeed_min;
				 */
				tecs_update_pitch_throttle(terrain_alt + flare_curve_alt_rel,
							   calculate_target_airspeed(airspeed_land),
							   eas2tas,
							   math::radians(_parameters.land_flare_pitch_min_deg),
							   math::radians(_parameters.land_flare_pitch_max_deg),
							   0.0f,
							   throttle_max,
							   throttle_land,
							   false,
							   _land_motor_lim ? math::radians(_parameters.land_flare_pitch_min_deg)
							   : math::radians(_parameters.pitch_limit_min),
							   _global_pos.alt,
							   ground_speed,
							   _land_motor_lim ? tecs_status_s::TECS_MODE_LAND_THROTTLELIM : tecs_status_s::TECS_MODE_LAND);

				if (!_land_noreturn_vertical) {
					// just started with the flaring phase
					_att_sp.pitch_body = 0.0f;
					_flare_height = _global_pos.alt - terrain_alt;
					mavlink_log_info(&_mavlink_log_pub, "#Landing, flaring");
					_land_noreturn_vertical = true;

				} else {
					if (_global_pos.vel_d > 0.1f) {
						_att_sp.pitch_body = math::radians(_parameters.land_flare_pitch_min_deg) *
								     math::constrain((_flare_height - (_global_pos.alt - terrain_alt)) / _flare_height, 0.0f, 1.0f);

					} else {
						_att_sp.pitch_body = _att_sp.pitch_body;
					}
				}

				_flare_curve_alt_rel_last = flare_curve_alt_rel;

			} else {

				/* intersect glide slope:
				 * minimize speed to approach speed
				 * if current position is higher than the slope follow the glide slope (sink to the
				 * glide slope)
				 * also if the system captures the slope it should stay
				 * on the slope (bool land_onslope)
				 * if current position is below the slope continue at previous wp altitude
				 * until the intersection with slope
				 * */
				/* 高度还很高，通过当前状态解算出期望高度，令飞机沿着该下降斜线飞 */
				float altitude_desired_rel;

				if (_global_pos.alt > terrain_alt + landing_slope_alt_rel_desired || _land_onslope) {
					/* stay on slope */
					altitude_desired_rel = landing_slope_alt_rel_desired;

					if (!_land_onslope) {
						mavlink_log_info(&_mavlink_log_pub, "#Landing, on slope");
						_land_onslope = true;
					}

				} else {
					/* continue horizontally */
					altitude_desired_rel =  pos_sp_triplet.previous.valid ? L_altitude_rel :
								_global_pos.alt - terrain_alt;
				}

				tecs_update_pitch_throttle(terrain_alt + altitude_desired_rel,
							   calculate_target_airspeed(airspeed_approach), eas2tas,
							   math::radians(_parameters.pitch_limit_min),
							   math::radians(_parameters.pitch_limit_max),
							   _parameters.throttle_min,
							   _parameters.throttle_max,
							   _parameters.throttle_cruise,
							   false,
							   math::radians(_parameters.pitch_limit_min),
							   _global_pos.alt,
							   ground_speed);
			}

		} 
		/*************************
		*
		* 自动起飞模式，从_L1控制器中得到期望滚转和期望航向，从_tecs控制器得到期望油门和期望俯仰角，
		* 随后，根据不同的起飞状态，用不同的roll\pitch\yaw的值覆盖上一步常规控制得到的值并输出，
		* 直到正常的起飞完成，才会回归到正常的L1和_tecs控制器控制。
		*
		**************************/
		else if (pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {

			/*************************
			*
			* 滑跑起飞，分为五个状态：
			* 1. THROTTLE_RAMP 		油门随时间从零线性增加；
			* 2. CLAMPED_TO_RUNWAY  增加油门一定时间后开始滑跑，此时航向可以通过轮子或方向舵手动控制；
			* 3. TAKEOFF 			滑跑到一定空速后直接起飞，此时pitch由_tecs控制器解算得到；
			* 4. CLIMBOUT 			起飞到一定高度后开始爬升，此时开始有滚转角的控制，即一边修改航线一边爬升；
			* 5. FLY 				朝着预定起飞点飞行，此时控制为正常的auto模式的控制
			* 全过程中，期望空速为最低空速乘上一个系数，这两个参数均可由用户修改。
			*
			**************************/
			if (_runway_takeoff.runwayTakeoffEnabled()) {
				if (!_runway_takeoff.isInitialized()) {
					math::Quaternion q(&_ctrl_state.q[0]);
					math::Vector<3> euler = q.to_euler();
					_runway_takeoff.init(euler(2), _global_pos.lat, _global_pos.lon);

					/* need this already before takeoff is detected
					 * doesn't matter if it gets reset when takeoff is detected eventually */
					_takeoff_ground_alt = _global_pos.alt;

					mavlink_log_info(&_mavlink_log_pub, "#Takeoff on runway");
				}

				float terrain_alt = get_terrain_altitude_takeoff(_takeoff_ground_alt, _global_pos);

				// update runway takeoff helper
				_runway_takeoff.update(
					_ctrl_state.airspeed,
					_global_pos.alt - terrain_alt,
					_global_pos.lat,
					_global_pos.lon,
					&_mavlink_log_pub);

			
				 /* 通过_runway_takeoff 控制器，不同的状态得到不同的StartWP，再传入L1控制器 */
				_l1_control.navigate_waypoints(_runway_takeoff.getStartWP(), curr_wp, current_position, nav_speed_2d);

				// update tecs
				float takeoff_pitch_max_deg = _runway_takeoff.getMaxPitch(_parameters.pitch_limit_max);
				float takeoff_pitch_max_rad = math::radians(takeoff_pitch_max_deg);

				tecs_update_pitch_throttle(pos_sp_triplet.current.alt,
							   calculate_target_airspeed(
								   _runway_takeoff.getMinAirspeedScaling() * _parameters.airspeed_min),
							   eas2tas,
							   math::radians(_parameters.pitch_limit_min),
							   takeoff_pitch_max_rad,
							   _parameters.throttle_min,
							   _parameters.throttle_max, // XXX should we also set runway_takeoff_throttle here?
							   _parameters.throttle_cruise,
							   _runway_takeoff.climbout(),
							   math::radians(_runway_takeoff.getMinPitch(
									   pos_sp_triplet.current.pitch_min,
									   10.0f,
									   _parameters.pitch_limit_min)),
							   _global_pos.alt,
							   ground_speed,
							   tecs_status_s::TECS_MODE_TAKEOFF);

				// assign values
				_att_sp.roll_body = _runway_takeoff.getRoll(_l1_control.nav_roll());
				_att_sp.yaw_body = _runway_takeoff.getYaw(_l1_control.nav_bearing());
				_att_sp.fw_control_yaw = _runway_takeoff.controlYaw();//变量在位置控制中赋值在姿态控制中使用，跑道上自动起飞时用wheel控制航向
				_att_sp.pitch_body = _runway_takeoff.getPitch(get_tecs_pitch());

				// reset integrals except yaw (which also counts for the wheel controller)
				_att_sp.roll_reset_integral = _runway_takeoff.resetIntegrators();
				_att_sp.pitch_reset_integral = _runway_takeoff.resetIntegrators();

				/*warnx("yaw: %.4f, roll: %.4f, pitch: %.4f", (double)_att_sp.yaw_body,
					(double)_att_sp.roll_body, (double)_att_sp.pitch_body);*/

			} else {
				/* Perform launch detection */
				if (_launchDetector.launchDetectionEnabled() &&
				    _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS) {
					/* Inform user that launchdetection is running */
					static hrt_abstime last_sent = 0;

					if (hrt_absolute_time() - last_sent > 4e6) {
						mavlink_log_critical(&_mavlink_log_pub, "#Launch detection running");
						last_sent = hrt_absolute_time();
					}

					/* Detect launch */
					_launchDetector.update(accel_body(0));

					/* update our copy of the launch detection state */
					_launch_detection_state = _launchDetector.getLaunchDetected();

				} else	{
					/* no takeoff detection --> fly */
					_launch_detection_state = LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS;
				}

				/* Set control values depending on the detection state */
				if (_launch_detection_state != LAUNCHDETECTION_RES_NONE) {
					/* Launch has been detected, hence we have to control the plane. */

					_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, nav_speed_2d);
					_att_sp.roll_body = _l1_control.nav_roll();
					_att_sp.yaw_body = _l1_control.nav_bearing();

					/* 在刚刚弹射出去的迟滞时间中，没有油门输出，随后进入正常的模式，油门立刻拉满 */
					float takeoff_throttle = _launch_detection_state !=
								 LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS ?
								 _launchDetector.getThrottlePreTakeoff() : _parameters.throttle_max;

					/* select maximum pitch: the launchdetector may impose another limit for the pitch
					 * depending on the state of the launch */
					float takeoff_pitch_max_deg = _launchDetector.getPitchMax(_parameters.pitch_limit_max);
					float takeoff_pitch_max_rad = math::radians(takeoff_pitch_max_deg);

					/* 在高度还不够高的时候，取最小的pitch并且限制滚转角的幅度，保证飞机在高度较低的时候安全爬升 */
					if (_parameters.climbout_diff > 0.001f && altitude_error > _parameters.climbout_diff) {
						/* enforce a minimum of 10 degrees pitch up on takeoff, or take parameter */
						tecs_update_pitch_throttle(pos_sp_triplet.current.alt,
									   calculate_target_airspeed(1.3f * _parameters.airspeed_min),
									   eas2tas,
									   math::radians(_parameters.pitch_limit_min),
									   takeoff_pitch_max_rad,
									   _parameters.throttle_min, takeoff_throttle,
									   _parameters.throttle_cruise,
									   true,
									   math::max(math::radians(pos_sp_triplet.current.pitch_min),
										     math::radians(10.0f)),
									   _global_pos.alt,
									   ground_speed,
									   tecs_status_s::TECS_MODE_TAKEOFF);

						/* limit roll motion to ensure enough lift */
						_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
										    math::radians(15.0f));

					} 
					/* 达到了一定的高度，开始正常的控制，此时的期望空速也为航点模式下的空速 mission_airspeed */
					else {
						tecs_update_pitch_throttle(pos_sp_triplet.current.alt,
									   calculate_target_airspeed(mission_airspeed),
									   eas2tas,
									   math::radians(_parameters.pitch_limit_min),
									   math::radians(_parameters.pitch_limit_max),
									   _parameters.throttle_min,
									   takeoff_throttle,
									   _parameters.throttle_cruise,
									   false,
									   math::radians(_parameters.pitch_limit_min),
									   _global_pos.alt,
									   ground_speed);
					}

				} else {
					/* 这里还没有发射出去，因此积分量始终保持为0,滚转角也为0*/
					_att_sp.roll_reset_integral = true;
					_att_sp.pitch_reset_integral = true;
					_att_sp.yaw_reset_integral = true;

					/* Set default roll and pitch setpoints during detection phase */
					_att_sp.roll_body = 0.0f;
					_att_sp.pitch_body = math::max(math::radians(pos_sp_triplet.current.pitch_min),
								       math::radians(10.0f));
				}
			}

		}

		/* reset landing state */
		if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LAND) {
			reset_landing_state();
		}

		/* reset takeoff/launch state */
		if (pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_TAKEOFF) {
			reset_takeoff_state();
		}

		if (was_circle_mode && !_l1_control.circle_mode()) {
			/* just kicked out of loiter, reset roll integrals */
			_att_sp.roll_reset_integral = true;
		}

	}
	
	//控制过程梳理九　POS定点模式
	//下面是POS定点模式：定高定向飞行，ｚ轴控高度　水平控速度（方向）
	//不大杆的情况下，飞机会保持当前的航向和高度一直向前飞
	//打杆的话，和ALT模式一样，pitch杆控制的是期望高度，油门杆控制的是期望空速，
	//roll杆控制的是_att_sp.roll_body副翼舵面，yaw摇杆不响应

	//下面在POSITION模式下探讨下L1锁航向的实现过程
	else if (_control_mode.flag_control_velocity_enabled &&
		   _control_mode.flag_control_altitude_enabled && 
		   _control_mode.flight_mode_ID==9) {
		/* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
		   heading is set to a distant waypoint */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
			_hdg_hold_yaw = _yaw;		//如果从非POSITION模式进来，需要记下当前高度和航向，以便后面定高定向飞行。
			_hdg_hold_enabled = false;	 //方向控一　这个标志为表示是否为第一次进入锁航向　第一次锁航向需要初始化prve curr航点信息以用来L1锁航向
			_yaw_lock_engaged = false;	 //方向控一　这个标志代表当前是否处于锁航向的状态　锁航向的过程中需要不断判断　更新prve curr航点信息以用来L1锁航向


			// 重置一下roll和yaw方向的期望值，保证飞机平稳飞行。
			//（pitch方向不需要重置，因为上面已经给高度值重置了）
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
			_att_sp.yaw_body = 0;
		}


		//从不定高定向的状态中切换过来，例如从手动切到POSCTL，需要重置_tecs控制器的相关变量	
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		_control_mode_current = FW_POSCTRL_MODE_POSITION;

		//如果打杆，pitch杆控制的是期望高度，油门杆控制的是期望空速
		//当pitch杆输入非常大的时候>0.85，即认为飞机进入爬升模式
		float altctrl_airspeed = get_demanded_airspeed();
		bool climbout_requested = update_desired_altitude(dt);


		//如果判定到当前正在起飞，
		//那么会辅助设置一个距离地面一定高度的期望点，并且对pitch进行限值，防止擦到地面
		float pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);//这里默认是－45度，这个函数里面限制成最小10度，防止头擦地


		/* throttle limiting */
		throttle_max = _parameters.throttle_max;

		if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		//调用_tecs控制器，根据摇杆转化的期望的高度和期望空速，得出保持空速需要的throttle sp，和保持高度需要的pitch sp
		tecs_update_pitch_throttle(_hold_alt,
					   altctrl_airspeed,
					   eas2tas,
					   math::radians(_parameters.pitch_limit_min),
					   math::radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   throttle_max,
					   _parameters.throttle_cruise,
					   climbout_requested,
					   ((climbout_requested) ? math::radians(10.0f) : pitch_limit_min),
					   _global_pos.alt,
					   ground_speed,
					   tecs_status_s::TECS_MODE_NORMAL);

		//下面是POS和ALT模式的区别，就在于滚转和航向的摇杆都在死区内时，POS锁航向，保持当前的航向往前飞,ALT不锁航向随风摆
		//滚转和航向的摇杆都在死区内，通过通过L1如何实现锁航向？下面的注释就是为解释这一点

		if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH &&
		    fabsf(_manual.r) < HDG_HOLD_MAN_INPUT_THRESH) {

			/* heading / roll is zero, lock onto current heading */
			if (fabsf(_ctrl_state.yaw_rate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
				// little yaw movement, lock to current heading
				_yaw_lock_engaged = true;	//方向控二　如果摇杆在死区内并且yaw_rate很小时认为进入锁航向状态，锁航向就是保持yaw

			}

			/* 方向控三　如果在起飞过程中，那么保证每次循环都会重置yaw的期望值为当前的yaw，即保证起飞过程中没有滚转 */
			if (in_takeoff_situation()) {			//起飞过程中，这两个标志“之前不是锁航向　现在是锁yaw”,下面的初始化航点程序就会循环进入
				_hdg_hold_enabled = false;	//循环进入，每次当前的yaw就会期望的航向，那么会不会再产生roll_sp yaw_sp,但同时每小步又都在保持当前yaw
				_yaw_lock_engaged = true;
			}

			//方向控四　在锁航向的时候，分为第一次进来初始化航点，后面在进来更新航点，都是在产生后面的prev前面的curr
			//然后将创建的点传给L1控制器，通过L1控制器计算出压航线（锁航向）所期望滚转角和航向角。
			if (_yaw_lock_engaged) {	//锁yaw了，即可以进入锁heading,都锁yaw了肯定锁航向

				
				if (!_hdg_hold_enabled) {	//之前还不是锁heading，说明是第一进入锁航向状态，记录当前航向，下面开始初始航点
					_hdg_hold_enabled = true;
					_hdg_hold_yaw = _yaw;
					//第一次进入锁航向，以当前位置　当前航向　向后100米创建prev_wp，向前3000米创建curr_wp，以调用L1实现压航线锁航向
					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
				}

				//循环回来还是在锁航向状态中，但已经不是第一次进来上面判断跳过，进入到下面的航点和判断与更新，判断是不是已经很逼近前面3000米的航点了
				if (get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
								  _hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon) < HDG_HOLD_REACHED_DIST) 
				{
					//距离前面的3000米的航点剩下只有1000米了，看来需要重新造航点了，以curr向前1100造新的prev，向后4000米造新的curr
					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
				}

				//总结上面在给L1造航点的过程，第一次进入锁航向，以当前pos向前100米造prev,向后3000米造curr
				//当pos向前逼近curr还剩1000的时候，这时候以curr，向前1１00米造prev,向后４000米造curr
				//这一循环下去，飞机永远到不了curr，在Ｌ１的控制的又必须压航线　锁航向

				math::Vector<2> prev_wp;
				prev_wp(0) = (float)_hdg_hold_prev_wp.lat;
				prev_wp(1) = (float)_hdg_hold_prev_wp.lon;

				math::Vector<2> curr_wp;
				curr_wp(0) = (float)_hdg_hold_curr_wp.lat;
				curr_wp(1) = (float)_hdg_hold_curr_wp.lon;

				/* populate l1 control setpoint */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
									    math::radians(15.0f));
				}
			}

		}

		
		//POS和ALT模式的区别就在于滚转和航向的摇杆都在死区内，POS锁航向，保持当前的航向往前飞,ALT不锁航向随风摆
		//那如果滚转和航向打杆呢？POS和ALT一样响应摇杆，此时摇杆控制　yaw不控

		if (!_yaw_lock_engaged || fabsf(_manual.y) >= HDG_HOLD_MAN_INPUT_THRESH ||
		    fabsf(_manual.r) >= HDG_HOLD_MAN_INPUT_THRESH) {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = false;
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
			_att_sp.yaw_body = 0;
		}

	}


	
	//不想影响上面正常完整的POS处理，仿照POS重新写三个模态的处理过程，借助flight_mode_ID区分这四个POS
	//发电三模态重定义六 三个模态都会进入到这个POS模式，不打杆的时候按照上面计算实现锁航向
	//但是如果打杆，就开始实现不同处理了

	else if (_control_mode.flag_control_velocity_enabled &&
		   _control_mode.flag_control_altitude_enabled && 
		  	 (_control_mode.flight_mode_ID==3 || //第一模态FOLLOW
		   	 _control_mode.flight_mode_ID==4 || //第二模态RATT
		  	 _control_mode.flight_mode_ID==8) )  //第三模态ACRO
	{
		/* POSITION CONTROL: pitch stick moves altitude setpoint, throttle stick sets airspeed,
		   heading is set to a distant waypoint */

		if (_control_mode_current != FW_POSCTRL_MODE_POSITION) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
			_hdg_hold_yaw = _yaw;
			_hdg_hold_enabled = false; // this makes sure the waypoints are reset below
			_yaw_lock_engaged = false;


			// 重置一下roll和yaw方向的期望值，保证飞机平稳飞行。
			//（pitch方向不需要重置，因为上面已经给高度值重置了）
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
			_att_sp.yaw_body = 0;
		}


		//从不定高定向的状态中切换过来，例如从手动切到POSCTL，需要重置_tecs控制器的相关变量	
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		_control_mode_current = FW_POSCTRL_MODE_POSITION;

		//如果打杆，pitch杆控制的是期望高度，油门杆控制的是期望空速
		//当pitch输入非常大的时候，即认为飞机进入爬升模式
		float altctrl_airspeed = get_demanded_airspeed();
		bool climbout_requested = update_desired_altitude(dt);


		//如果判定到当前正在起飞，
		//那么会辅助设置一个距离地面一定高度的期望点，并且对pitch进行限值，防止擦到地面
		float pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);


		/* throttle limiting */
		throttle_max = _parameters.throttle_max;

		if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		//调用_tecs控制器，根据摇杆转化的期望的高度和期望空速，得出保持空速需要的throttle sp，和保持高度需要的pitch sp
		tecs_update_pitch_throttle(_hold_alt,
					   altctrl_airspeed,
					   eas2tas,
					   math::radians(_parameters.pitch_limit_min),
					   math::radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   throttle_max,
					   _parameters.throttle_cruise,
					   climbout_requested,
					   ((climbout_requested) ? math::radians(10.0f) : pitch_limit_min),
					   _global_pos.alt,
					   ground_speed,
					   tecs_status_s::TECS_MODE_NORMAL);

		//下面是POS和ALT模式的区别，就在于滚转和航向的摇杆都在死区内，POS锁航向，保持当前的航向往前飞,ALT不锁航向随风摆
		//滚转和航向的摇杆都在死区内，如何实现锁航向

		if (fabsf(_manual.y) < HDG_HOLD_MAN_INPUT_THRESH &&
		    fabsf(_manual.r) < HDG_HOLD_MAN_INPUT_THRESH) {

			/* heading / roll is zero, lock onto current heading */
			if (fabsf(_ctrl_state.yaw_rate) < HDG_HOLD_YAWRATE_THRESH && !_yaw_lock_engaged) {
				// little yaw movement, lock to current heading
				_yaw_lock_engaged = true;

			}

			/* 如果在起飞过程中，那么保证每次循环都会重置yaw的期望值为当前的yaw，即保证起飞过程中没有滚转 */
			if (in_takeoff_situation()) {
				_hdg_hold_enabled = false;
				_yaw_lock_engaged = true;
			}

			//在锁头模式的时候，会以当前的位置为起始点，当前的航向为方向，创建一个距离当前点水平3000m的点，
			//然后将创建的点传给L1控制器，通过L1控制器解算出当前的期望滚转角和航向角。
			if (_yaw_lock_engaged) {

				/* just switched back from non heading-hold to heading hold */
				if (!_hdg_hold_enabled) {
					_hdg_hold_enabled = true;
					_hdg_hold_yaw = _yaw;

					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, true);
				}

				/* we have a valid heading hold position, are we too close? */
				if (get_distance_to_next_waypoint(_global_pos.lat, _global_pos.lon,
								  _hdg_hold_curr_wp.lat, _hdg_hold_curr_wp.lon) < HDG_HOLD_REACHED_DIST) {
					get_waypoint_heading_distance(_hdg_hold_yaw, HDG_HOLD_DIST_NEXT, _hdg_hold_prev_wp, _hdg_hold_curr_wp, false);
				}

				math::Vector<2> prev_wp;
				prev_wp(0) = (float)_hdg_hold_prev_wp.lat;
				prev_wp(1) = (float)_hdg_hold_prev_wp.lon;

				math::Vector<2> curr_wp;
				curr_wp(0) = (float)_hdg_hold_curr_wp.lat;
				curr_wp(1) = (float)_hdg_hold_curr_wp.lon;

				/* populate l1 control setpoint */
				_l1_control.navigate_waypoints(prev_wp, curr_wp, current_position, ground_speed_2d);

				_att_sp.roll_body = _l1_control.nav_roll();
				_att_sp.yaw_body = _l1_control.nav_bearing();

				if (in_takeoff_situation()) {
					/* limit roll motion to ensure enough lift */
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, math::radians(-15.0f),
									    math::radians(15.0f));
				}
			}

		}

		//三种模态下，当摇杆不在死区的时候 就不同处理了
		//如果roll摇杆超过死区 摇杆控制副翼舵面
		if (!_yaw_lock_engaged || fabsf(_manual.y) >= HDG_HOLD_MAN_INPUT_THRESH ||
		    fabsf(_manual.r) >= HDG_HOLD_MAN_INPUT_THRESH) {
			_hdg_hold_enabled = false;
			_yaw_lock_engaged = false;
			_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad; //副翼的最大舵面45度 在fw_att_control_params中定义的
			_att_sp.yaw_body = 0;
		}

		//这时候如果打杆pitch呢,我只是想锁航线，pitch还是按照原来姿态控制的方法，控制pitch_sp
		switch(_control_mode.flight_mode_ID)
		{

			case 3:  //Follow第一后退模态
				_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max_rad + math::radians(6.0f);
				_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
				break;

			case 4:  //Rattitude //第二上升模态
				_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max_rad + math::radians(10.0f);
				_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
				break;

			case 8:  //ARCO  //第三下降模态
				_att_sp.pitch_body = -_manual.x * _parameters.man_pitch_max_rad + math::radians(-6.0f);
				_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
				break;
		}
	}
	//上面是三个模态的POS仿写的处理过程
	






	// 控制过程梳理四　ALT定高模式

	//下面是固定翼ATLCTL定高模式，不打杆飞机会保持当前的高度往前飞，但是如果有侧风，横向航迹和航向都会随着风漂移。
	//如果打杆，pitch杆控制的是期望高度，油门杆控制的是期望空速，
	//如果打杆，roll杆控制的是_att_sp.roll_body副翼舵面，yaw摇杆不响应，

	else if (_control_mode.flag_control_altitude_enabled) {

		//从不控高度的模式过来，记录当前高度后期好定高飞行
		if (_control_mode_current != FW_POSCTRL_MODE_POSITION && _control_mode_current != FW_POSCTRL_MODE_ALTITUDE) {
			/* Need to init because last loop iteration was in a different mode */
			_hold_alt = _global_pos.alt;
		}

		/* Reset integrators if switching to this mode from a other mode in which posctl was not active */
		if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
			/* reset integrators */
			_tecs.reset_state();
		}

		//初始化完成后赋值当前当前模式为定高模式
		_control_mode_current = FW_POSCTRL_MODE_ALTITUDE;

		// 控制过程梳理五（可本文件搜索，查看处理过程）
		//如果打杆，pitch杆控制的是期望高度，油门杆控制的是期望空速
		float altctrl_airspeed = get_demanded_airspeed();

		/* update desired altitude based on user pitch stick input */
		bool climbout_requested = update_desired_altitude(dt);


		/* if we assume that user is taking off then help by demanding altitude setpoint well above ground
		* and set limit to pitch angle to prevent stearing into ground
		*/
		float pitch_limit_min;
		do_takeoff_help(&_hold_alt, &pitch_limit_min);

		/* throttle limiting */
		throttle_max = _parameters.throttle_max;

		if (_vehicle_land_detected.landed && (fabsf(_manual.z) < THROTTLE_THRESH)) {
			throttle_max = 0.0f;
		}

		// 控制过程梳理八（可本文件搜索，查看处理过程）
		//下面是att_sp产生的过程，位置控制计算的结果
		// 调用_tecs控制器，根据摇杆转化的期望的高度和期望空速，得出保持空速需要的throttle sp，和保持高度需要的pitch sp
		tecs_update_pitch_throttle(_hold_alt,
					   altctrl_airspeed,
					   eas2tas,
					   math::radians(_parameters.pitch_limit_min),
					   math::radians(_parameters.pitch_limit_max),
					   _parameters.throttle_min,
					   throttle_max,
					   _parameters.throttle_cruise,
					   climbout_requested,
					   ((climbout_requested) ? math::radians(10.0f) : pitch_limit_min),
					   _global_pos.alt,
					   ground_speed,
					   tecs_status_s::TECS_MODE_NORMAL);
		
		//如果打杆，roll杆控制的是_att_sp.roll_body副翼舵面，yaw摇杆不响应，
		_att_sp.roll_body = _manual.y * _parameters.man_roll_max_rad;
		_att_sp.yaw_body = 0;

	} 
	
	// 控制过程梳理十一 STAB手动模式
	//不是ALT POS AUTO，那就是STAB MANUAL
	else {
		_control_mode_current = FW_POSCTRL_MODE_OTHER;

		//手动模式,返回值为false,跳出主函数中,不发布位置控制计算的att_sp.
		//即手动模式下,位置控制没用,瞎计算,在姿态控制中将会接收遥控器转化为期望姿态.
		setpoint = false;

		// reset hold altitude
		_hold_alt = _global_pos.alt;

		/* reset landing and takeoff state */
		if (!_last_manual) {
			reset_landing_state();
			reset_takeoff_state();
		}
	}

	/* Copy thrust output for publication */
	if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
		/* Set thrust to 0 to minimize damage */
		_att_sp.thrust = 0.0f;

	} 

	//之前已经用TECS算出来thrust_sp，pitch_sp,但是部分特殊场合他们有着自己的要求
	//下面就是判断是使用你算出来的，还是我自己来

	//之前已经用TECS算出来_att_sp.thrust，但是部分特殊场合对油门有要求
	//不同的状态会通过不同的控制器得到期望油门. 自动起飞相关油门设置，分别对应：
	//1. 自动弹射起飞，开桨之前
	//2. 自动滑跑起飞
	//3. 空闲状态
	//4. 其他非正常状态
	//5. 正常状态
	else if (_control_mode_current == FW_POSCTRL_MODE_AUTO && // launchdetector only available in auto
		   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
		   _launch_detection_state != LAUNCHDETECTION_RES_DETECTED_ENABLEMOTORS &&
		   !_runway_takeoff.runwayTakeoffEnabled()) {
		/* making sure again that the correct thrust is used,
		 * without depending on library calls for safety reasons.
		   the pre-takeoff throttle and the idle throttle normally map to the same parameter. */
		_att_sp.thrust = (_launchDetector.launchDetectionEnabled()) ? _launchDetector.getThrottlePreTakeoff() :
				 _parameters.throttle_idle;

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
		   _runway_takeoff.runwayTakeoffEnabled()) {
		_att_sp.thrust = _runway_takeoff.getThrottle(math::min(get_tecs_thrust(), throttle_max));

	} else if (_control_mode_current == FW_POSCTRL_MODE_AUTO &&
		   pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
		_att_sp.thrust = 0.0f;

	} else if (_control_mode_current == FW_POSCTRL_MODE_OTHER) {
		_att_sp.thrust = math::min(_att_sp.thrust, _parameters.throttle_max);

	} else {
		/* Copy thrust and pitch values from tecs */
		if (_vehicle_land_detected.landed) {
			// when we are landed state we want the motor to spin at idle speed
			_att_sp.thrust = math::min(_parameters.throttle_idle, throttle_max);

		} else {
			_att_sp.thrust = math::min(get_tecs_thrust(), throttle_max);
		}


	}


	//之前已经用TECS算出来_att_sp.pitch_body，但是部分特殊场合对pitch有特殊要求，下面判断是都使用TECS算出来的pitch_sp
	// decide when to use pitch setpoint from TECS because in some cases pitch
	// setpoint is generated by other means
	bool use_tecs_pitch = true;

	// auto runway takeoff
	use_tecs_pitch &= !(_control_mode_current ==  FW_POSCTRL_MODE_AUTO &&
			    pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF &&
			    (_launch_detection_state == LAUNCHDETECTION_RES_NONE || _runway_takeoff.runwayTakeoffEnabled()));


	// flaring during landing
	use_tecs_pitch &= !(pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND &&
			    _land_noreturn_vertical);

	// manual attitude control,手动控制下不用tecs算出来的pitch
	use_tecs_pitch &= !(_control_mode_current == FW_POSCTRL_MODE_OTHER);

	//发电三模态重定义七
	if(_control_mode.flight_mode_ID==3 ||  _control_mode.flight_mode_ID==4 || _control_mode.flight_mode_ID==8)
	{
		use_tecs_pitch=false;//三个模态不使用tecs pitch摇杆算作pitch_sp
	}

	if (use_tecs_pitch) {		
		_att_sp.pitch_body = get_tecs_pitch();
	}

	if (_control_mode.flag_control_position_enabled) {
		_last_manual = false;

	} else {
		_last_manual = true;
	}


	return setpoint;
}

float
FixedwingPositionControl::get_tecs_pitch()
{
	if (_is_tecs_running) {
		return _tecs.get_pitch_demand();

	} else {
		// return 0 to prevent stale tecs state when it's not running
		return 0.0f;
	}
}

float
FixedwingPositionControl::get_tecs_thrust()
{
	if (_is_tecs_running) {
		return _tecs.get_throttle_demand();

	} else {
		// return 0 to prevent stale tecs state when it's not running
		return 0.0f;
	}
}

void
FixedwingPositionControl::handle_command()
{
	if (_vehicle_command.command == vehicle_command_s::VEHICLE_CMD_DO_GO_AROUND) {
		// only abort landing before point of no return (horizontal and vertical)
		if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {

			if (_land_noreturn_vertical) {
				mavlink_log_info(&_mavlink_log_pub, "#Landing, can't abort after flare");

			} else {
				_fw_pos_ctrl_status.abort_landing = true;
				mavlink_log_info(&_mavlink_log_pub, "#Landing, aborted");
			}
		}
	}
}




// 控制过程梳理一（可本文件搜索，查看处理过程）
//　这是固定翼的位置控制，向上承接commander的飞行模式和navigator的期望航点
//　经过计算产生att_sp，向下发布给fw_att_control.
//　ALT POS AUTO都会计算产生att_sp，但是STAB MANUAL会跳过位置控制直接进入到姿态控制中产生att_sp

void
FixedwingPositionControl::task_main()
{

	/*
	 * do subscriptions
	 */
	_global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_sensor_combined_sub = orb_subscribe(ORB_ID(sensor_combined));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_vehicle_command_sub = orb_subscribe(ORB_ID(vehicle_command));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sub = orb_subscribe(ORB_ID(manual_control_setpoint));

	/* rate limit control mode updates to 5Hz */
	orb_set_interval(_control_mode_sub, 200);
	/* rate limit vehicle status updates to 5Hz */
	orb_set_interval(_vehicle_status_sub, 200);
	/* rate limit vehicle land detected updates to 5Hz */
	orb_set_interval(_vehicle_land_detected_sub, 200);
	/* rate limit position updates to 50 Hz */
	orb_set_interval(_global_pos_sub, 20);

	/* abort on a nonzero return value from the parameter init */
	if (parameters_update()) {
		/* parameter setup went wrong, abort */
		warnx("aborting startup due to errors.");
		_task_should_exit = true;
	}

	/* wakeup source(s) */
	px4_pollfd_struct_t fds[2];

	/* Setup of loop */
	fds[0].fd = _params_sub;
	fds[0].events = POLLIN;
	fds[1].fd = _global_pos_sub;
	fds[1].events = POLLIN;

	_task_running = true;

	while (!_task_should_exit) {

		/* wait for up to 500ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		/* timed out - periodic check for _task_should_exit, etc. */
		if (pret == 0) {
			continue;
		}

		/* this is undesirable but not much we can do - might want to flag unhappy status */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		/* check vehicle control mode for changes to publication state */
		vehicle_control_mode_poll();

		/* check for new vehicle commands */
		vehicle_command_poll();

		/* check vehicle status for changes to publication state */
		vehicle_status_poll();

		/* check vehicle land detected for changes to publication state */
		vehicle_land_detected_poll();

		/* only update parameters if they changed */
		if (fds[0].revents & POLLIN) {
			/* read from param to clear updated flag */
			struct parameter_update_s update;
			orb_copy(ORB_ID(parameter_update), _params_sub, &update);

			/* update parameters from storage */
			parameters_update();
		}

		//以下整个运算都在这个POLL里面
		if (fds[1].revents & POLLIN) {  
			perf_begin(_loop_perf);

			/* load local copies */
			orb_copy(ORB_ID(vehicle_global_position), _global_pos_sub, &_global_pos);
			
			//在手动模式的时候，是否需要高度重置、坐标重置			
			// handle estimator reset events. we only adjust setpoins for manual modes
			if (_control_mode.flag_control_manual_enabled) {
				if (_control_mode.flag_control_altitude_enabled && _global_pos.alt_reset_counter != _alt_reset_counter) {
					_hold_alt += _global_pos.delta_alt;
					// make TECS accept step in altitude and demanded altitude
					_tecs.handle_alt_step(_global_pos.delta_alt, _global_pos.alt);
				}

				// adjust navigation waypoints in position control mode
				if (_control_mode.flag_control_altitude_enabled && _control_mode.flag_control_velocity_enabled
				    && _global_pos.lat_lon_reset_counter != _pos_reset_counter) {

					// reset heading hold flag, which will re-initialise position control
					_hdg_hold_enabled = false;
				}
			}

			// update the reset counters in any case
			_alt_reset_counter = _global_pos.alt_reset_counter;
			_pos_reset_counter = _global_pos.lat_lon_reset_counter;

			// XXX add timestamp check
			_global_pos_valid = true;

			control_state_poll();
			vehicle_setpoint_poll();
			vehicle_sensor_combined_poll();
			vehicle_manual_control_setpoint_poll();		

			math::Vector<3> ground_speed(_global_pos.vel_n, _global_pos.vel_e,  _global_pos.vel_d);
			math::Vector<2> current_position((float)_global_pos.lat, (float)_global_pos.lon);

			
			//控制过程梳理二（可本文件搜索，查看处理过程）
			//位置控制模块最后正确结果是publish(_attitude_setpoint_id
			//实际代码都在control_position函数里,ALT POS AUTO都会进入这个函数计算并产生att_sp，返回true发布att_sp。
			//但旋翼状态，手动控制STAB,MANUAL，返回false跳出位置控制不发布att_sp，而是进入到att_control在哪里产生att_sp.

			//进到括号说明att_sp计算成功并返回true,计算成功后下面进行publish.
			if (control_position(current_position, ground_speed, _pos_sp_triplet)) 
			{	
				_att_sp.timestamp = hrt_absolute_time();
				_att_sp.roll_body += _parameters.rollsp_offset_rad;
				_att_sp.pitch_body += _parameters.pitchsp_offset_rad;

				if (_control_mode.flag_control_manual_enabled) {
					_att_sp.roll_body = math::constrain(_att_sp.roll_body, -_parameters.man_roll_max_rad, _parameters.man_roll_max_rad);
					_att_sp.pitch_body = math::constrain(_att_sp.pitch_body, -_parameters.man_pitch_max_rad, _parameters.man_pitch_max_rad);
				}

				/* lazily publish the setpoint only once available */
				if (_attitude_sp_pub != nullptr) {
					/* publish the attitude setpoint */
					orb_publish(_attitude_setpoint_id, _attitude_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					/* advertise and publish */
					_attitude_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

				/* XXX check if radius makes sense here */
				float turn_distance = _l1_control.switch_distance(100.0f);

				/* lazily publish navigation capabilities */
				if ((hrt_elapsed_time(&_fw_pos_ctrl_status.timestamp) > 1000000)
				    || (fabsf(turn_distance - _fw_pos_ctrl_status.turn_distance) > FLT_EPSILON
					&& turn_distance > 0)) {

					/* set new turn distance */
					_fw_pos_ctrl_status.turn_distance = turn_distance;

					_fw_pos_ctrl_status.nav_roll = _l1_control.nav_roll();
					_fw_pos_ctrl_status.nav_pitch = get_tecs_pitch();
					_fw_pos_ctrl_status.nav_bearing = _l1_control.nav_bearing();

					_fw_pos_ctrl_status.target_bearing = _l1_control.target_bearing();
					_fw_pos_ctrl_status.xtrack_error = _l1_control.crosstrack_error();

					math::Vector<2> curr_wp((float)_pos_sp_triplet.current.lat, (float)_pos_sp_triplet.current.lon);
					_fw_pos_ctrl_status.wp_dist = get_distance_to_next_waypoint(current_position(0), current_position(1), curr_wp(0),
								      curr_wp(1));

					fw_pos_ctrl_status_publish();
				}

			}
			//true则发布了两个topic,return false不成功啥也没有.

			perf_end(_loop_perf);
		}

	}

	_task_running = false;

	warnx("exiting.\n");

	_control_task = -1;
}

void FixedwingPositionControl::reset_takeoff_state()
{
	_launch_detection_state = LAUNCHDETECTION_RES_NONE;
	_launchDetector.reset();
	_runway_takeoff.reset();
}

void FixedwingPositionControl::reset_landing_state()
{
	_time_started_landing = 0;

	// reset terrain estimation relevant values
	_time_last_t_alt = 0;

	_land_noreturn_horizontal = false;
	_land_noreturn_vertical = false;
	_land_stayonground = false;
	_land_motor_lim = false;
	_land_onslope = false;
	_land_useterrain = false;

	// reset abort land, unless loitering after an abort
	if (_fw_pos_ctrl_status.abort_landing == true
	    && _pos_sp_triplet.current.type != position_setpoint_s::SETPOINT_TYPE_LOITER) {

		_fw_pos_ctrl_status.abort_landing = false;
	}

}


//将期望高度和期望空速传给tecs控制器

void FixedwingPositionControl::tecs_update_pitch_throttle(float alt_sp, float v_sp, float eas2tas,
		float pitch_min_rad, float pitch_max_rad,
		float throttle_min, float throttle_max, float throttle_cruise,
		bool climbout_mode, float climbout_pitch_min_rad,
		float altitude,
		const math::Vector<3> &ground_speed,
		unsigned mode)
{
	float dt = 0.01f; // prevent division with 0

	if (_last_tecs_update > 0) {
		dt = hrt_elapsed_time(&_last_tecs_update) * 1e-6;
	}

	_last_tecs_update = hrt_absolute_time();

	// do not run TECS if we are not in air
	bool run_tecs = !_vehicle_land_detected.landed;

	// do not run TECS if vehicle is a VTOL and we are in rotary wing mode or in transition
	// (it should also not run during VTOL blending because airspeed is too low still)
	if (_vehicle_status.is_vtol) {
		run_tecs &= !_vehicle_status.is_rotary_wing && !_vehicle_status.in_transition_mode;
	}

	// 在垂起的转换过程中，期望空速与正常飞行的时候不同，需要做相关设置
	if (_vehicle_status.is_vtol && _vehicle_status.in_transition_mode) {
		_was_in_transition = true;

		// set this to transition airspeed to init tecs correctly
		if (_parameters.airspeed_mode == control_state_s::AIRSPD_MODE_DISABLED) {
			// some vtols fly without airspeed sensor
			_asp_after_transition = _parameters.airspeed_trans;

		} else {
			_asp_after_transition = _ctrl_state.airspeed;
		}

		_asp_after_transition = math::constrain(_asp_after_transition, _parameters.airspeed_min, _parameters.airspeed_max);

	} else if (_was_in_transition) {
		// after transition we ramp up desired airspeed from the speed we had coming out of the transition
		//转换成功以后，持续保持空速增加
		_asp_after_transition += dt * 2; // increase 2m/s

		if (_asp_after_transition < v_sp && _ctrl_state.airspeed < v_sp) {
			v_sp = fmaxf(_asp_after_transition, _ctrl_state.airspeed);

		} else {
			_was_in_transition = false;
			_asp_after_transition = 0;
		}
	}

	_is_tecs_running = run_tecs;

	if (!run_tecs) {
		// next time we run TECS we should reinitialize states
		_reinitialize_tecs = true;
		return;
	}

	if (_reinitialize_tecs) {
		_tecs.reset_state();
		_reinitialize_tecs = false;
	}

	if (_vehicle_status.engine_failure || _vehicle_status.engine_failure_cmd) {
		/* Force the slow downwards spiral */
		pitch_min_rad = M_DEG_TO_RAD_F * -1.0f;
		pitch_max_rad = M_DEG_TO_RAD_F * 5.0f;
	}

	/* No underspeed protection in landing mode */
	_tecs.set_detect_underspeed_enabled(!(mode == tecs_status_s::TECS_MODE_LAND
					      || mode == tecs_status_s::TECS_MODE_LAND_THROTTLELIM));

	/* Using tecs library */
	float pitch_for_tecs = _pitch - _parameters.pitchsp_offset_rad;

	// if the vehicle is a tailsitter we have to rotate the attitude by the pitch offset
	// between multirotor and fixed wing flight
	if (_parameters.vtol_type == vtol_type::TAILSITTER && _vehicle_status.is_vtol) {
		math::Matrix<3, 3> R_offset;
		R_offset.from_euler(0, M_PI_2_F, 0);
		math::Matrix<3, 3> R_fixed_wing = _R_nb * R_offset;
		math::Vector<3> euler = R_fixed_wing.to_euler();
		pitch_for_tecs = euler(1);
	}

	//TECS控制核心函数
	//根据当前的高度，空速，期望的高度和空速，求解出期望的俯仰角和期望的油门
	_tecs.update_pitch_throttle(_R_nb, pitch_for_tecs, altitude, alt_sp, v_sp,
				    _ctrl_state.airspeed, eas2tas,
				    climbout_mode, climbout_pitch_min_rad,
				    throttle_min, throttle_max, throttle_cruise,
				    pitch_min_rad, pitch_max_rad);

	//从tecs控制器中读取结果
	struct TECS::tecs_state s;
	_tecs.get_tecs_state(s);

	struct tecs_status_s t = {};

	t.timestamp = s.timestamp;

	switch (s.mode) {
	case TECS::ECL_TECS_MODE_NORMAL:
		t.mode = tecs_status_s::TECS_MODE_NORMAL;
		break;

	case TECS::ECL_TECS_MODE_UNDERSPEED:
		t.mode = tecs_status_s::TECS_MODE_UNDERSPEED;
		break;

	case TECS::ECL_TECS_MODE_BAD_DESCENT:
		t.mode = tecs_status_s::TECS_MODE_BAD_DESCENT;
		break;

	case TECS::ECL_TECS_MODE_CLIMBOUT:
		t.mode = tecs_status_s::TECS_MODE_CLIMBOUT;
		break;
	}

	t.altitudeSp 		= s.altitude_sp;
	t.altitude_filtered = s.altitude_filtered;
	t.airspeedSp 		= s.airspeed_sp;
	t.airspeed_filtered = s.airspeed_filtered;

	t.flightPathAngleSp 		= s.altitude_rate_sp;
	t.flightPathAngle 			= s.altitude_rate;
	t.flightPathAngleFiltered 	= s.altitude_rate;

	t.airspeedDerivativeSp 	= s.airspeed_rate_sp;
	t.airspeedDerivative 	= s.airspeed_rate;

	t.totalEnergyError 				= s.total_energy_error;
	t.totalEnergyRateError 			= s.total_energy_rate_error;
	t.energyDistributionError 		= s.energy_distribution_error;
	t.energyDistributionRateError 	= s.energy_distribution_rate_error;

	t.throttle_integ 	= s.throttle_integ;
	t.pitch_integ 		= s.pitch_integ;

	if (_tecs_status_pub != nullptr) {
		orb_publish(ORB_ID(tecs_status), _tecs_status_pub, &t);

	} else {
		_tecs_status_pub = orb_advertise(ORB_ID(tecs_status), &t);
	}
}

int
FixedwingPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("fw_pos_ctrl_l1",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1700,
					   (px4_main_t)&FixedwingPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int fw_pos_control_l1_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: fw_pos_control_l1 {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (l1_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		if (OK != FixedwingPositionControl::start()) {
			warn("start failed");
			return 1;
		}

		/* avoid memory fragmentation by not exiting start handler until the task has fully started */
		while (l1_control::g_control == nullptr || !l1_control::g_control->task_running()) {
			usleep(50000);
			printf(".");
			fflush(stdout);
		}

		printf("\n");

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (l1_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete l1_control::g_control;
		l1_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (l1_control::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
