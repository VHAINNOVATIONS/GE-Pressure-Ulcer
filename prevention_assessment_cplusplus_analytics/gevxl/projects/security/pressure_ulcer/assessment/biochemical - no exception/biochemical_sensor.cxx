// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Barry Hathaway
/// \date 5/12/2014
/// \par Modifications:


#include "biochemical_sensor.h"

#include "AllDLL.h"
#include <string>
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace gevxl;
using namespace gevxl::pressure_ulcer;

BiochemicalSensor::BiochemicalSensor(std::string port)
{
	strcpy_s(comPort,port.c_str());
	double f;
	f = (13000000 * 4294967296 / 100000000) + 0.5;
	startFrequency = static_cast<unsigned int>(f);
	f = (20000 * 4294967296 / 100000000) + 0.5;
	incrementFrequency = static_cast<unsigned int>(f);
	f = (12207.05 * 4294967296 / 100000000) + 0.5;
	offsetFrequency = static_cast<unsigned int>(f);
	numberSamples = 128;
	numberPoints = 32;
	DDSDAC = 257;
	calibrationSwitch = 0;
	numberReps = 1;
	numberSkips = 64;
	flag = 0;

}

BiochemicalSensor::~BiochemicalSensor(void)
{

}

int BiochemicalSensor::init(void)
{
	char response[512];
	int32_t len = 512;
	int rc = INIT_dll(comPort, response, len);
	// initResponse = std::string(response,len);
	initResponse = std::string(response);
	return rc;
}

int BiochemicalSensor::createConfiguration(std::string parmName[],std::string parmValue[])
{
	int rc = 0;
	configError = "";
	std::string parm;
	double dval;
	double ival;
	for (int i=0; i<10; i++) {
		if (parmName[i].empty())
			return rc;
		parm = boost::algorithm::to_lower_copy(parmName[i]);
		if (parm.compare("startfrequency") == 0) {
			try {
				dval = boost::lexical_cast<double>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			startFrequency = static_cast<unsigned int>((dval * 4294967296 / 100000000) + 0.5);
		} else if (parm.compare("incrementfrequency") == 0) {
			try {
				dval = boost::lexical_cast<double>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			incrementFrequency = static_cast<unsigned int>((dval * 4294967296 / 100000000) + 0.5);
		} else if (parm.compare("offsetfrequency") == 0) {
			try {
				dval = boost::lexical_cast<double>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			offsetFrequency = static_cast<unsigned int>((dval * 4294967296 / 100000000) + 0.5);
		} else if (parm.compare("numbersamples") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			numberSamples = static_cast<unsigned int>(ival);
		} else if (parm.compare("numberpoints") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			numberPoints = static_cast<unsigned int>(ival);
		} else if (parm.compare("ddsdac") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			DDSDAC = static_cast<unsigned int>(ival);
		} else if (parm.compare("calibrationswitch") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			calibrationSwitch = static_cast<unsigned int>(ival);
		} else if (parm.compare("numberreps") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			numberReps = static_cast<unsigned int>(ival);
		} else if (parm.compare("numberskips") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			numberSkips = static_cast<unsigned int>(ival);
		} else if (parm.compare("flag") == 0) {
			try {
				ival = boost::lexical_cast<int>( parmValue[i] );
			} catch( boost::bad_lexical_cast const& ) {
				configError = "Error: value ( " + parmValue[i] + " ) for parameter " + parmName[i] + " was not valid";
				rc = -4;
				return rc;
			}
			flag = static_cast<unsigned int>(ival);
		} else {
			configError = "Error: parameter name, " + parmName[i] + ", was not valid";
			rc = -4;
			return rc;
		}
	}
	return rc;
}

int BiochemicalSensor::configure(void)
{
	configResponse = "";
	uint8_t response[512];
	int32_t len = 512;
	uint32_t parms[10];
	uint32_t parmsResponse[10];
	parms[0] = startFrequency;
	parms[1] = incrementFrequency;
	parms[2] = offsetFrequency;
	parms[3] = numberSamples;
	parms[4] = numberPoints;
	parms[5] = DDSDAC;
	parms[6] = calibrationSwitch;
	parms[7] = numberReps;
	parms[8] = numberSkips;
	parms[9] = flag;
 	int rc = CONFIG_dll(comPort, parms, response, 10, len);
	for (int i=0,k=0; k<10; i+=4,k++) {
		parmsResponse[k] = response[i]|response[i+1]<<8|response[i+2]<<16|response[i+3]<<24;
	}
	for (int j=0; j<10; j++) {
		if (parms[j] != parmsResponse[j]) {
			configResponse = "Error in configuration parameter # " + boost::lexical_cast<std::string>(j) +
				" in = " + boost::lexical_cast<std::string>(parms[j]) + 
				", out = " + boost::lexical_cast<std::string>(parmsResponse[j]);
			return -1;
		}
	}
	return rc;

}

int BiochemicalSensor::scan(void)
{
	uint8_t response[4096];
	int32_t len = 4096;
	uint32_t ms = msec;
 	int rc = SCAN_dll(comPort, response, &ms, len);
	// scanResponse = std::string(reinterpret_cast<char const*>(response),len);
	scanResponse = std::string(reinterpret_cast<char const*>(response));
	return rc;

}


std::string BiochemicalSensor::getInitResponse(void)
{
	return initResponse;
}

std::string BiochemicalSensor::getConfigResponse(void)
{
	return configResponse;
}

std::string BiochemicalSensor::getScanResponse(void)
{
	return scanResponse;
}

std::string BiochemicalSensor::getConfigError(void)
{
	return configError;
}
