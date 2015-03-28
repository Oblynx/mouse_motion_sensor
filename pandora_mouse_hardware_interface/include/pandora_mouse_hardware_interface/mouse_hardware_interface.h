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
#ifndef PANDORA_MOUSE_HARDWARE_INTERFACE_MOUSE_HARDWARE_INTERFACE_H
#define PANDORA_MOUSE_HARDWARE_INTERFACE_MOUSE_HARDWARE_INTERFACE_H

#include <fstream>
#include <ros/ros.h>
#include "hardware_interface/robot_hw.h"
#include "pandora_mouse_hardware_interface/mouseMeasurement.h"

namespace pandora_hardware_interface
{
namespace mouse
{
	typedef pandora_mouse_hardware_interface::mouseMeasurement mouseMeasurementMsg;
	
	class MouseHardwareInterface : public hardware_interface::RobotHW
	{
		public:
			explicit MouseHardwareInterface(ros::NodeHandle nodeHandle);
			~MouseHardwareInterface();
			void cycle();
		private:
			/**
				@brief Reads dx,dy and creates msg
				@return void
			**/
			void read();
			ros::NodeHandle nodeHandle_; //!< node handle
			ros::Publisher mousePub_;
			std::ifstream mouseStream_;
			std::string dev_;	//!< Device filename
			std::string mouseTopic_;	//!< Topic on which to publish
			double dx_, dy_;
	};
} // namespace mouse
} // namespace pandora_hardware_interface
#endif // PANDORA_MOUSE_HARDWARE_INTERFACE_MOUSE_HARDWARE_INTERFACE_H
