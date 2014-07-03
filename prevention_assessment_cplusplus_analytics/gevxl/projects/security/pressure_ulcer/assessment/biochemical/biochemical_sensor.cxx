// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Barry Hathaway
/// \date 5/12/2014
/// \par Modifications:


#include "biochemical_sensor.h"
#include "64bitGen2X.h"
#include <boost/algorithm/string.hpp>
#include <boost/lexical_cast.hpp>

using namespace gevxl;
using namespace gevxl::pressure_ulcer;

BiochemicalSensor::BiochemicalSensor(const vcl_string port)
{
	strcpy_s(comPort,port.c_str());
	double f;
	f = (10000000.0 * 4294967296 / 100000000) + 0.5;
	startFrequency = static_cast<unsigned int>(f);
	f = (50000.0 * 4294967296 / 100000000) + 0.5;
	incrementFrequency = static_cast<unsigned int>(f);
	f = (12207.05 * 4294967296 / 100000000) + 0.5;
	offsetFrequency = static_cast<unsigned int>(f);
	numberSamples = 128;
	numberPoints = 101;
	DDSDAC = 771;
	calibrationSwitch = 0;
	numberReps = 1;
	numberSkips = 64;
	flag = 0;

}

BiochemicalSensor::~BiochemicalSensor(void)
{

}

void BiochemicalSensor::init(void)
{
	char response[512];
	int32_t len = 512;
	int rc = INIT_dll(comPort, response, len);
	if (rc < 0) {
		vcl_string initResponse = vcl_string(response);
		throw BiochemicalSensorException("BiochemicalSensorException - error initializing sensor: "+initResponse);
	}
}


void BiochemicalSensor::configure(int nReps)
{
	numberReps = static_cast<unsigned int>(nReps);
	vcl_string configResponse;
	uint8_t response[512];
	int32_t len = 512;
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
 	int rc = CONFIGX_dll(comPort, numberReps, response, len);
	if (rc < 0) {
		configResponse = "BiochemicalSensorException - error configuring sensor: rc = " + 
				boost::lexical_cast<vcl_string>(rc);
		throw BiochemicalSensorException(configResponse);
	}
	/*
	for (int i=0,k=0; k<10; i+=4,k++) {
		parmsResponse[k] = response[i]|response[i+1]<<8|response[i+2]<<16|response[i+3]<<24;
	}
	for (int j=0; j<9; j++) {
		if (parms[j] != parmsResponse[j]) {
			configResponse = "error in configuration parameter # " + boost::lexical_cast<vcl_string>(j) +
				" in = " + boost::lexical_cast<vcl_string>(parms[j]) + 
				", out = " + boost::lexical_cast<vcl_string>(parmsResponse[j]);
			throw BiochemicalSensorException("BiochemicalSensorException - "+configResponse);
		}
	}
	 */
}

int BiochemicalSensor::scan(void)
{
	vcl_string scanErrorMsg;
	int32_t len = 101;
	uint8_t response[4096];
	int32_t len2 = 4096;
	uint32_t parmsResponse[10];
	int32_t vri, vii, cri, cii, checksum;
	double vr, vi, cr, ci, frequency;
	vcl_complex<double> voltage, current, impedance;
	scanFrequencies.clear();
	scanImpedances.clear();
	int32_t mychecksum = 0;
	double ZrArray[101], FArray[101], ZiArray[101];
 	int rc = SCANX_dll(comPort, ZrArray, FArray, ZiArray, response, &fPeak, &zrPeak, len, len2);
	if (rc < 0) {
		scanErrorMsg = "BiochemicalSensorException - error during scan: rc = " + 
				boost::lexical_cast<vcl_string>(rc);
		throw BiochemicalSensorException(scanErrorMsg);
	}
	/*
	for (int i=0,k=0; k<10; i+=4,k++) {
		parmsResponse[k] = response[i]|response[i+1]<<8|response[i+2]<<16|response[i+3]<<24;
	}
	// Only compare 1st 9 parameters - 10th is always 1
	for (int j=0; j<9; j++) {
		if (parms[j] != parmsResponse[j]) {
			scanErrorMsg = "error in verifying configuration parameter # " + boost::lexical_cast<vcl_string>(j) +
				" in = " + boost::lexical_cast<vcl_string>(parms[j]) + 
				", out = " + boost::lexical_cast<vcl_string>(parmsResponse[j]);
			// std::cout << "Exception: " << scanErrorMsg << std::endl;
			throw BiochemicalSensorException("BiochemicalSensorException - "+scanErrorMsg);
		}
	}
	int i,j;
	int np = boost::lexical_cast<int>(numberPoints);
	for (i=40,j=0; i<sizeof(response)&&j<np; i+=16,j++) {
		if (response[i]=='E' && response[i+1]=='D' && response[i+2]=='A' && response[i+3]=='T') break; 
		vri = response[i]|response[i+1]<<8|response[i+2]<<16|response[i+3]<<24;
		vii = response[i+4]|response[i+5]<<8|response[i+6]<<16|response[i+7]<<24;
		cri = response[i+8]|response[i+9]<<8|response[i+10]<<16|response[i+11]<<24;
		cii = response[i+12]|response[i+13]<<8|response[i+14]<<16|response[i+15]<<24;
		mychecksum = mychecksum + vri + vii + cri + cii;
		try {
			vr = boost::lexical_cast<double>( vri );
			vi = boost::lexical_cast<double>( vii );
			cr = boost::lexical_cast<double>( cri );
			ci = boost::lexical_cast<double>( cii );
		} catch( boost::bad_lexical_cast const& ) {
			throw BiochemicalSensorException("BiochemicalSensorException - error converting scan data.");
		}
		voltage = vcl_complex<double>(vr,vi);
		current = vcl_complex<double>(cr,ci);
		impedance = voltage / current;
		scanVoltages.push_back(voltage);
		scanCurrents.push_back(current);
		scanImpedancesRaw.push_back(impedance);
		// Construct frequence vector
		frequency = startFrequencyD + (j * incrementFrequencyD);
		scanFrequenciesRaw.push_back(frequency);
		// Construct vectors from arrays
		scanFrequencies.push_back(FArray[j]);
		impedance = vcl_complex<double>(ZrArray[j], ZiArray[j]);
		scanImpedances.push_back(impedance);
	}
	checksum = response[i]|response[i+1]<<8|response[i+2]<<16|response[i+3]<<24;
	if (checksum != mychecksum) {
		scanErrorMsg = "checksum mismatch: " + boost::lexical_cast<vcl_string>(checksum) +
			" vs. " + boost::lexical_cast<vcl_string>(mychecksum);
		throw BiochemicalSensorException("BiochemicalSensorException - " + scanErrorMsg);
	}
	i+=4;
	// std::cout << "i = " << i << "     j = " << j << std::endl;
	if (!(response[i]=='E' && response[i+1]=='D' && response[i+2]=='A' && response[i+3]=='T'))
		throw BiochemicalSensorException("BiochemicalSensorException - missing EDAT in scan data.");
	return rc;
	 */
	int np = boost::lexical_cast<int>(numberPoints);
	for (int j=0; j<np; j++) {
		// Construct vectors from arrays
		scanFrequencies.push_back(FArray[j]);
		impedance = vcl_complex<double>(ZrArray[j], ZiArray[j]);
		scanImpedances.push_back(impedance);
	}

}

vcl_vector<double> BiochemicalSensor::getScanFrequencies(void)
{
	return scanFrequencies;
}

vcl_vector<vcl_complex<double> > BiochemicalSensor::getScanImpedances(void)
{
	return scanImpedances;
}

double BiochemicalSensor::getFPeak(void)
{
	return fPeak;
}

double BiochemicalSensor::getZrPeak(void)
{
	return zrPeak;
}



