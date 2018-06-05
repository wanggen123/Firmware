/****************************************************************************
 *
 *   Copyright (c) 2013-2014 PX4 Development Team. All rights reserved.
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
 * @file Takeoff.cpp
 *
 * Helper class to Takeoff
 *
 * @author Lorenz Meier <lorenz@px4.io
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "takeoff.h"
#include "navigator.h"

Takeoff::Takeoff(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_min_alt(this, "MIS_TAKEOFF_ALT", false)
{
	// load initial params
	updateParams();
}

Takeoff::~Takeoff()
{
}

void
Takeoff::on_inactive()
{
}

void
Takeoff::on_activation() //takeoff切换模式第一次进来，准备起飞航点
{
	set_takeoff_position();
}

void
Takeoff::on_active()     //周期性运行的函数
{
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();
	//即使在offboard模式下，on_activation（）函数的最后给结构体清空了，不会进来
	if (rep->current.valid) { 
		// reset the position
		set_takeoff_position();

	} 
	//正常周期循环进来的地方
	//判断是否已经到达航点，如果还没达到 不做任何处理，如果已经达到了，进入下面if
	else if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) 
	{
		//更新任务状态，航点已经完成了
		_navigator->get_mission_result()->finished = true;
		_navigator->set_mission_result_updated();

		//设置飞机在当前位置悬停
		// set loiter item so position controllers stop doing takeoff logic
		set_loiter_item(&_mission_item);  //_mission_item  on_activation()函数中算的

		//将当前悬停打包的航点_mission_item赋值到pos_sp_triplet，并通知位置控制航点更新了，不一定是位置更新可能是类型更新
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}

	//如果没有达到航点，nothing
}



//航点来源有三种：
//offboard模式发来的参数
//mission地面站设置的一系列航点（在dataman中进行这种航点管理）
//如takeoff这种没人设置，飞控自己根据参数自己算的

void
Takeoff::set_takeoff_position()
{
	//整个函数主要功能是设置起飞航点，过程大概可以分成四步
	//第一步 设置起飞高度，这个高度有三种：offboard传参进来的，遥控器切换模式默认高度，还有已经飞很高时此时切模式
	//第二步 把航点信息打包到_mission_item结构体中
	//第三步 再把_mission_item结构体 复制到pos_sp_triplet->current
	//第四部 通知位置控制pos_sp_triplet更新了，你可以控制实现了


	//下面高度逻辑，如果offboard发来高度，好啊，但是不能太低。如果没人设置就用飞控自己默认的高度，但是飞机都已经飞得很高了再切takeoff，那就hold吧
	//abs_altitude就是最终确定的起飞高度。

	//rep表示offboard模式下给的航点信息，如果是offboard则在navigator_main.cpp 482行源码里已经给_takeoff_triplet航点赋值，下面的rep拿到已经是填充好了offboard参数的航点，否则拿到的就是一片空闲空间
	struct position_setpoint_triplet_s *rep = _navigator->get_takeoff_triplet();

	//临时变量，用来保存offboard模式下对起飞高度的设置值
	float abs_altitude = 0.0f;

	//参数对起飞高度的最小限制，相对于home点的最小高度
	const float min_abs_altitude = _navigator->get_home_position()->alt + _param_min_alt.get();

	// 如果offboard在起飞航点里设置了高度，那么则起飞到设置高度，但是起飞高度的设置是有参数底线的min_abs_altitude
	if (rep->current.valid && PX4_ISFINITE(rep->current.alt)) {
		abs_altitude = rep->current.alt;

		// If the altitude suggestion is lower than home + minimum clearance, raise it and complain.
		if (abs_altitude < min_abs_altitude) {
			abs_altitude = min_abs_altitude;
			mavlink_log_critical(_navigator->get_mavlink_log_pub(),
					     "Using minimum takeoff altitude: %.2f m", (double)_param_min_alt.get());
		}
	}
	
	//不是offboard切换模式，而是正常的遥控器模式切换，起飞高度就是参数高度
	else {
		// Use home + minimum clearance but only notify.
		abs_altitude = min_abs_altitude;
		mavlink_log_info(_navigator->get_mavlink_log_pub(),
				 "Using minimum takeoff altitude: %.2f m", (double)_param_min_alt.get());
	}


	//如果当前已经飞的很高，此时切换到takeoff模式，那就在当前高度保持HOLD
	//这是和实际飞行相对应的，实际中在很高时，切takeoff，飞机就是保持当前高度HOLD
	if (abs_altitude < _navigator->get_global_position()->alt) {
		// If the suggestion is lower than our current alt, let's not go down.
		abs_altitude = _navigator->get_global_position()->alt;
		mavlink_log_critical(_navigator->get_mavlink_log_pub(),
				     "Already higher than takeoff altitude");
	}

	//abs_altitude就是最终确定的起飞高度。高度已经确定，下面应该定经度 维度等其余起飞相关信息。


	
	//第二步 把航点信息打包到_mission_item结构体中，并更新一些标志
	// set current mission item to takeoff，默认在原地垂直起飞到上方。
	set_takeoff_item(&_mission_item, abs_altitude);
	_navigator->get_mission_result()->reached = false; //航点有没有达到 第一次进来没有达到
	_navigator->get_mission_result()->finished = false;//任务有没有完成 第一次进来任务没有完成
	_navigator->set_mission_result_updated(); //更新任务执行的结果
	reset_mission_item_reached();


	//第三步 再把_mission_item结构体 复制到pos_sp_triplet->current，这个在位置控制中使用。
	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet(); //这是拿数据存储空间，就是在初始化结构体指针
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->current.yaw = _navigator->get_home_position()->yaw;
	pos_sp_triplet->current.yaw_valid = true;
	pos_sp_triplet->next.valid = false;


	//上面只是处理了offboard的高度设置问题，如果offboard对偏航yaw 经纬度也有设置，那就覆盖
	if (rep->current.valid) {

		// Go on and check which changes had been requested
		if (PX4_ISFINITE(rep->current.yaw)) {
			pos_sp_triplet->current.yaw = rep->current.yaw;
		}

		if (PX4_ISFINITE(rep->current.lat) && PX4_ISFINITE(rep->current.lon)) {
			pos_sp_triplet->current.lat = rep->current.lat;
			pos_sp_triplet->current.lon = rep->current.lon;
		}

		// 清空rep航点结构体，会影响on_active()函数
		memset(rep, 0, sizeof(*rep));
	}


	//设置飞机完成takeoff后悬停
	_navigator->set_can_loiter_at_sp(true);


	//上面一系列处理：高度 经度 纬度等，已经填充好了position_setpoint_triplet
	_navigator->set_position_setpoint_triplet_updated();
	//在navigator_main.cpp中发布，通知位置控制position_setpoint_triplet更新了，你可以去做takeoff动作了。
	//建议看到这先不用去看其他模式下，跟随_pos_sp_triplet_updated到navigator.cpp看发布，跟随position_setpoint_triplet到位置控制里看使用过程。
}
