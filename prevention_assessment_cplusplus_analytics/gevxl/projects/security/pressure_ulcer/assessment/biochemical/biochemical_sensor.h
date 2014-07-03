/* biochemical_sensor.h */
/*
This class wraps the pressure ulcer assessment biochemical sensor Dll created by LabVIEW.
Labview generates a C interface so it is necessary to generate an object-oriented class
which can be used to access the base level C functions from the AllDLL.dll.
These functions are INIT_dll, CONFIG_dll, and SCAN_dll.  
*/

#ifndef __BIOCHEMCIALSENSOR_H__
#define __BIOCHEMCIALSENSOR_H__
#include <vcl_string.h>
#include <exception>
#include <vcl_vector.h>
#include <vcl_complex.h>

namespace gevxl {
	namespace pressure_ulcer {

class BiochemicalSensorException : public std::exception {
public:
	BiochemicalSensorException(const std::string m="BiochemicalSensorException!"):msg(m){}
	~BiochemicalSensorException(void){};
	const char* what(){return msg.c_str();}
private:
	std::string msg;
};

class BiochemicalSensor {

private:
	char comPort[32];
	unsigned int parms[10];
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
	// Scan responses
	vcl_vector<double> scanFrequencies;
	vcl_vector<vcl_complex<double> > scanImpedances;
	double fPeak;
	double zrPeak;

public: 
	// Constructor
	BiochemicalSensor(const vcl_string port); 
	// Destructor
	virtual ~BiochemicalSensor(void);

	virtual void init(void);
	virtual void configure(int nReps);
	virtual int scan(void);
	virtual vcl_vector<double> getScanFrequencies(void);
	virtual vcl_vector<vcl_complex<double> > getScanImpedances(void);
	virtual double getFPeak(void);
	virtual double getZrPeak(void);

};

} // end of pressure_ulcer namespace
} // end of gevxl namespace

#endif

