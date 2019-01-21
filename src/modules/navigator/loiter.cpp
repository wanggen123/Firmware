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
 * @file loiter.cpp
 *
 * Helper class to loiter
 *
 * @author Julian Oes <julian@oes.ch>
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <geo/geo.h>

#include <systemlib/mavlink_log.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "loiter.h"
#include "navigator.h"

Loiter::Loiter(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name),
	_param_min_alt(this, "MIS_LTRMIN_ALT", false),
	_loiter_pos_set(false)
{
	// load initial params
	updateParams();
}

Loiter::~Loiter()
{
}

void
Loiter::on_inactive()
{
	_loiter_pos_set = false;
}


//悬停 分为在哪个点悬停：当前位置 还是指定悬停位置
//on_activation()第一次切换进hold模式，只运行一次初始化悬停点
//gazebo下仿真测得出两种进入loiter模式的方式：
//由其他模式直接切入到hold模式 进入下面的set_loiter_position()飞机在当前位置保持悬停
//QGC上指点飞行confirm后 其实是切入hold模式，这种切入hold进入下面的set_loiter_position()，飞机飞往所指的curr点 到点后保持悬停
void
Loiter::on_activation()
{
	//QGC上指点飞行confirm后 其实是切入hold模式，这种切入hold进入下面的set_loiter_position()，飞机飞往所指的curr点 到点后保持悬停
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition(); 

	} 
	//由其他模式直接切入到hold模式 进入下面的set_loiter_position()飞机在当前位置保持悬停
	else {
		set_loiter_position();
	}
}


//hold模式的运行中
//如果QGC指新点了 reposition()，飞机飞往指点curr 到点后保持悬停
//其他情况下不做任何动作

//总结：切换到hold模式 飞机在当前位置悬停，如果地面站指新点 飞机就到飞到新点悬停（保持悬停，并同时影响指点飞行，剩下的就是看地面站指点飞行是用哪个mavlink消息传递过来的）

void
Loiter::on_active()
{
	//飞机目前已经在hold模式下 这时候QGC的指点飞行还是进入下面reposition(),飞机飞往指点curr 到点后保持悬停
	if (_navigator->get_reposition_triplet()->current.valid) {
		reposition();
	}

	//其他情况下不做任何动作，即不给位置控制发消息 让位置控制一直保持悬停就可以了

	// reset the loiter position if we get disarmed。下面情况 飞机在悬停的时候上锁了 或者飞机在上所的时候切换到hold 都没有实际意义 标志位可以忽略
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		_loiter_pos_set = false;
	}

	
}

void
Loiter::set_loiter_position()  //只在on_activation()函数中调用，在其他模式切入hold时 初始化一次：设置悬停点
{
	// not setting loiter position until armed
	//上锁状态，切换到hold直接return
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED ||
	    _loiter_pos_set) {
		return; //当飞机没有解锁的时候 切换到hold模式，直接return没有设置悬停位置 也没有通知位置控制，简单来说 既然还在上锁呢 白切
		        //当飞机已经解锁  切换到hold模式，如果是第一次进来设置_loiter_pos_set = true并设置悬停位置 通知位置控制
				//目前我看_loiter_pos_set标志位是为了防止set_loiter_position()在切入hold时重复set_loiter_position设置悬停点，但是其实这个函数在on_activation()函数里本身就只会执行一次，所以我看这个标志位是多余的

	} 
	else //解锁状态 切换到hold 第一次进来
	{
		_loiter_pos_set = true;
	}

	//设置悬停点
	// set current mission item to loiter
	set_loiter_item(&_mission_item, _param_min_alt.get());

	// convert mission item to current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->current.velocity_valid = false;
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

	_navigator->set_position_setpoint_triplet_updated();
}

void
Loiter::reposition()  	//指点飞行，飞机飞往所指的curr点 到点后保持悬停
{
	// we can't reposition if we are not armed yet
	if (_navigator->get_vstatus()->arming_state != vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	//获取指点飞行的航点信息rep
	struct position_setpoint_triplet_s *rep = _navigator->get_reposition_triplet();

	//航点信息有效
	if (rep->current.valid) { 
		// set loiter position based on reposition command
		// convert mission item to current setpoint
		//获取航点储存的空间
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		pos_sp_triplet->current.velocity_valid = false;
		//当前位置设置为previous
		pos_sp_triplet->previous.yaw = _navigator->get_global_position()->yaw;
		pos_sp_triplet->previous.lat = _navigator->get_global_position()->lat;
		pos_sp_triplet->previous.lon = _navigator->get_global_position()->lon;
		pos_sp_triplet->previous.alt = _navigator->get_global_position()->alt;
		//指点的位置设置为current
		memcpy(&pos_sp_triplet->current, &rep->current, sizeof(rep->current));
		pos_sp_triplet->next.valid = false;

		// set yaw 设置偏航方向，如果当前位置距离指定点很远，调整方向指向current
		//计算下当前到目标点的距离
		float travel_dist = get_distance_to_next_waypoint(_navigator->get_global_position()->lat,
				    _navigator->get_global_position()->lon,
				    pos_sp_triplet->current.lat, pos_sp_triplet->current.lon);

		//当前到目标点很远，需要将飞机的航向调整为 指向目标点
		if (travel_dist > 1.0f) {
			// calculate direction the vehicle should point to.
			pos_sp_triplet->current.yaw = get_bearing_to_next_waypoint(
							      _navigator->get_global_position()->lat,
							      _navigator->get_global_position()->lon,
							      pos_sp_triplet->current.lat,
							      pos_sp_triplet->current.lon);
		}

		//能否在目标点悬停
		_navigator->set_can_loiter_at_sp(pos_sp_triplet->current.type == position_setpoint_s::SETPOINT_TYPE_LOITER);

		//通知位置控制结构体更新
		_navigator->set_position_setpoint_triplet_updated();

		// 清空rep结构体，防止rep重复执行多次
		memset(rep, 0, sizeof(*rep));
	}
}