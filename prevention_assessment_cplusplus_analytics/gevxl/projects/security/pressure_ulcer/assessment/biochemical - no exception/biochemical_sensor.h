/* biochemical_sensor.h */
/*
This class wraps the pressure ulcer assessment biochemical sensor Dll created by LabVIEW.
Labview generates a C interface so it is necessary to generate an object-oriented class
which can be used to access the base level C functions from the AllDLL.dll.
These functions are INIT_dll, CONFIG_dll, and SCAN_dll.  
*/

#ifndef __BIOCHEMCIALSENSOR_H__
#define __BIOCHEMCIALSENSOR_H__
#include <string>

namespace gevxl {
	namespace pressure_ulcer {

class BiochemicalSensor {

private:
	char comPort[32];
	std::string initResponse;
	std::string configResponse;
	std::string scanResponse;
	std::string configError;
	static const int msec = 165;
	// Configuration parameters
	unsigned int startFrequency;
	unsigned int incrementFrequency;
	unsigned int offsetFrequency;
	unsigned int numberSamples;
	unsigned int numberPoints;
	unsigned int DDSDAC;
	unsigned int calibrationSwitch;
	unsigned int numberReps;
	unsigned int numberSkips;
	unsigned int flag;

public: 
	// Constructor
	BiochemicalSensor(std::string port); 
	// Destructor
	virtual ~BiochemicalSensor(void);

	virtual int init(void);
	virtual int createConfiguration(std::string parmName[],std::string parmValue[]);
	virtual int configure(void);
	virtual int scan(void);
	virtual std::string getInitResponse(void);
	virtual std::string getConfigResponse(void);
	virtual std::string getScanResponse(void);
	virtual std::string getConfigError(void);
};

} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

