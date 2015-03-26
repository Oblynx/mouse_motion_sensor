///Mouse motion sensor
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <stdexcept>
//#include <fcntl.h>	[system -- might be useful for I/O?]

/**Transformation coefficient from mouse meas. units to millimetres*/
#define MOUSE_TO_MM 0.02243081

class MotionSensor{
	public:	
		/**@class Contains the mouse event data and relevant operations*/
		class Motion{
			public:
				Motion(int x=0, int y=0, int flags=0): x_(x), y_(y), flags_(flags), d_(0) {};
				/**@brief Add another Motion's (x,y,d) values to own and replace own flags.
				 * Can be chained*/
				Motion& add(Motion a){
					x_+= a.x_, y_+= a.y_, d_+= a.distance(), flags_= a.flags_;
					return *this;
				}
				Motion& convert(){
					x_*=MOUSE_TO_MM, y_*=MOUSE_TO_MM, d_*=MOUSE_TO_MM;
					return *this;
				}
				Motion& reset(){
					*this= Motion(0,0,0);
					return *this;
				}
				double distance(){ return sqrt(x_*x_+y_*y_); }
				/**@brief Log x, y and distance to stream s*/
				void log(std::ostream& s){ s <<
					std::setw(3)<<x_<<' '<<
					std::setw(3)<<y_<<' '<<
					std::setw(5)<<distance()<<'\n'; }
				
				int x_,y_, flags_;
				/**Total distance traveled*/
				double d_;
		};
		/**@brief Opens streams*/
		MotionSensor(std::string filename): mouse_("/dev/input/"+filename),
		 log_("motion.log"), totalDistance_(0) {};
		~MotionSensor(){
			mouse_.close();
			log_.close();
		}
		/**@brief Logs data from mouse movements to both cout and logfile
		 * @param dataCount: How many events to log
		 * */
		void getData(const unsigned long dataCount);
		/**@brief Interactively measures length
		 * @returns The total mouse motion
		 * @throws std::overflow_error: If mouse buffer overflows
		 * */
		Motion measureLength();
		
		double getTotalDistance() { return totalDistance_; }
		static double physicalConversion(double measurement){ return measurement*MOUSE_TO_MM; }
	
	private:
		/**@brief Actually access the stream to read. Blocking.*/
		Motion readMouse();
		std::ifstream mouse_;
		std::ofstream log_;
		char buf[3];
		double totalDistance_;
};

MotionSensor::Motion MotionSensor::readMouse(){
	mouse_.read(buf,3);
	if (mouse_.gcount()!=3) throw new std::length_error("Incomplete mouse read!\n");
	return Motion(buf[1], buf[2], buf[0]);
}

void MotionSensor::getData(const unsigned long dataCount){
	std::cout << "logging...\n  x   y   Dist\n";
	unsigned long count=0;
	MotionSensor::Motion motion(0,0,0);
	while(mouse_.good() && log_.good() && count < dataCount){
		count++;
		motion= readMouse();
		motion.log(std::cout); motion.log(log_);
		totalDistance_+= motion.distance();
		if(count%80==0) std::cout << "\n  x   y\n";
	}
}

MotionSensor::Motion MotionSensor::measureLength(){
	std::cout << "Left-click to start measurement, left-click again to stop\n";
	totalDistance_= 0;
	char initLeftButtonState=0, firstTime=1;
	Motion motion(0,0,0);
	//Wait for click
	while(mouse_.good()){
		motion= readMouse();
		if(firstTime){
			initLeftButtonState= readMouse().flags_ & 1;
			firstTime= 0;
		}
		//If left-click...
		if ((motion.flags_ & 1) != initLeftButtonState) break;	//left-button flag
	}
	motion.reset();
	//Get data until next click
	while(mouse_.good()){
		motion.add(readMouse()).log(log_);
		std::cout << '.';
		if ((motion.flags_ & (64|128)) > 0){										//overflow flags
			std::cerr << "\n\n\t-->Overflow!\n";
			throw new std::overflow_error("Mouse buffer overflow");
		}
		if ((motion.flags_ & 1) > 0) break;
	}
	std::cout << "\nMeasurement complete.\n";
	return motion.convert();
}
