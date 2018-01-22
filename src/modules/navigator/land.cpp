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
 * @file land.cpp
 *
 * Helper class to land at the current position
 *
 * @author Andreas Antener <andreas@uaventure.com>
 */

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>
#include <fcntl.h>

#include <systemlib/err.h>
#include <systemlib/mavlink_log.h>

#include <uORB/uORB.h>
#include <uORB/topics/position_setpoint_triplet.h>

#include "land.h"
#include "navigator.h"

Land::Land(Navigator *navigator, const char *name) :
	MissionBlock(navigator, name)
{
	/* load initial params */
	updateParams();
}

Land::~Land()
{
}

void
Land::on_inactive()
{
}



//飞行模式处理中 pos_sp_triplet会发布两次，影响后面位置控制
//第一次初始化航点，如takeoff起飞点，land降落点
//任务完成后，发布新的航点，虽然位置没变，但是类型发生了改变，是在当前点悬停还是idle

//航点处理的流程分为三步：
//第一步 set_×××_item(&_mission_item, ××××); 打包航点信息到_mission_item
//第二步 将mission_item赋值给position_setpoint_triplet current setpoint
//第三步 通知pos_sp_triplet航点信息更新完成，通知位置控制开始做新的控制

void
Land::on_activation()
{
	//是否在当前位置降落，还是在home点降落，打包航点信息到_mission_item
	set_land_item(&_mission_item, true); 
	//更新标志
	_navigator->get_mission_result()->reached = false;
	_navigator->get_mission_result()->finished = false;
	_navigator->set_mission_result_updated();
	reset_mission_item_reached();

	//将mission_item赋值给current setpoint
	struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
	pos_sp_triplet->previous.valid = false;
	mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
	pos_sp_triplet->next.valid = false;

	//飞机是否在那个点悬停，land当然不需要悬停
	_navigator->set_can_loiter_at_sp(false);

	//航点信息更新完成
	_navigator->set_position_setpoint_triplet_updated();
}

void
Land::on_active()
{
	//降落是否已经完成，降落已经完成的话，将当前任务设置为idle，继而影响位置控制
	if (is_mission_item_reached() && !_navigator->get_mission_result()->finished) 
	{
		//更新标志，任务完成
		_navigator->get_mission_result()->finished = true; 
		_navigator->set_mission_result_updated();  

		//任务完成，设置当前为idle，位置信息没改变，但是改变了航点类型
		set_idle_item(&_mission_item);   

		//将mission_item赋值给current setpoint，通知位置控制pos_sp_triplet航点信息更新完成
		struct position_setpoint_triplet_s *pos_sp_triplet = _navigator->get_position_setpoint_triplet();
		mission_item_to_position_setpoint(&_mission_item, &pos_sp_triplet->current);
		_navigator->set_position_setpoint_triplet_updated();
	}
	
	//降落还没完成，nothing
}
