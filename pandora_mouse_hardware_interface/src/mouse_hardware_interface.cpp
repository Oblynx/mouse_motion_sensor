/*********************************************************************
*
* Software License Agreement (BSD License)
*
* Copyright (c) 2014, P.A.N.D.O.R.A. Team.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
* * Redistributions of source code must retain the above copyright
* notice, this list of conditions and the following disclaimer.
* * Redistributions in binary form must reproduce the above
* copyright notice, this list of conditions and the following
* disclaimer in the documentation and/or other materials provided
* with the distribution.
* * Neither the name of the P.A.N.D.O.R.A. Team nor the names of its
* contributors may be used to endorse or promote products derived
* from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*
* Author: Konstantinos Samaras-Tsakiris
*********************************************************************/
#include "pandora_mouse_hardware_interface/mouse_hardware_interface.h"

namespace pandora_hardware_interface
{
namespace mouse
{
	MouseHardwareInterface::MouseHardwareInterface(ros::NodeHandle nodeHandle) :
		nodeHandle_(nodeHandle)
	{
		nodeHandle_.getParam("dev_mouse", dev_);
		nodeHandle_.getParam("mouse_topic", mouseTopic_);
		dx_= 0, dy_= 0;
		mouseStream_.open(dev_,std::ifstream::trunc | std::ifstream::binary);
		mousePub= nodeHandle_.advertise<mouseMeasurementMsg>(mouseTopic_, 500);
	}
	MouseHardwareInterface::~MouseHardwareInterface() {}

	void MouseHardwareInterface::cycle(){
		if (!mouseStream_.good()) ...;
		read();
		//!< Generate message
		mouseMeasurementMsg motionMsg;
		motionMsg.header.stamp=ros::Time::now();
		motionMsg.dx=dx_;
		motionMsg.dy=dy_;
		//!< Publish
		mousePub.publish(motionMsg);
	}
	void MouseHardwareInterface::read(){
		char buf[3];
		mouseStream_.readsome(buf, 3);
		//!< Check if read got all the data
		if (mouseStream_.gcount()!=3){
			ROS_ERROR("Mouse read incomplete");
			...
		}
		//!< Check for mouse overflow
		if (buf[0] & (64|128)) > 0){
			ROS_ERROR("Mouse overflow");
			...
		}
		dx_= buf[1], dy_= buf[2];
	}


} // namespace mouse
} // namespace pandora_hardware_interface
