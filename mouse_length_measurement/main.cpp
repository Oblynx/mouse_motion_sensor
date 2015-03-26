///Main
#include <cstring>
#include "sensor.cpp"

void experiment(int times, double* average, MotionSensor& sensor){	
	MotionSensor::Motion* motion= new MotionSensor::Motion[times];
	for(int i=0; i<times; i++){
		motion[i]= sensor.measureLength();
		average[0]+=motion[i].x_, average[1]+= motion[i].y_, average[2]+= motion[i].d_;
		std::cout
			<<"Δx: "	<<std::setprecision(0)<<std::fixed<<motion[i].x_<<"mm\t"
			<<"Δy: "	<<std::setprecision(0)<<std::fixed<<motion[i].y_<<"mm\t"
			<<"Distance traveled: "<<std::setprecision(0)<<std::fixed<<motion[i].d_<<"mm\n\n";
	}
}

int main(int argc, char** argv)
{
	if(argc<2) {
		std::cout << "Call \"./main --help\" for help\n";
		return 1;
	}
	if(!strcmp(argv[1], "--help") || !strcmp(argv[1], "-h")){
		std::cout << "Syntax: sudo ./main <mouse file in /dev/input/> [--calibrate <times>]\n"
			<< "\t--calibrate <times>:\tMeasure the same length many times and get statistics for calibration\n";
		return 2;
	}
	
	MotionSensor sensor(argv[1]);
	std::cout << "--Mouse Sensor--\n";
	if(argc>2){
		if(!strcmp(argv[2],"--calibrate")){
			int times=strtol(argv[3], NULL, 10);
			double average[3]{0,0,0};
			experiment(times, average, sensor);
			average[0]/= times, average[1]/= times, average[2]/= times;
			std::cout <<"\tAverage\n"<<std::setfill('-')<<std::setw(20)<<'-'
				<<"\nΔx: "	<<std::setprecision(0)<<std::fixed<<average[0]<<"mm"
				<<"\tΔy: "	<<std::setprecision(0)<<std::fixed<<average[1]<<"mm"
				<<"\tDistance traveled: "<<std::setprecision(0)<<std::fixed<<average[2]<<"mm\n\n";
		}
	}else{													//Only once
		std::cout << "\nexp1\n";
		double average[3]{0,0,0};
		experiment(1, average, sensor);
	}
	return 0;
}
