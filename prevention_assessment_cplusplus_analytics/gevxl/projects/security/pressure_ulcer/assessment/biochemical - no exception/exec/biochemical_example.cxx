/*
The purpose of this program is to test the pressure ulcer assessment biochemical
sensor Dll created by LabVIEW in C. This program calls three functions from the AllDLL.dll.
These functions are INIT_dll, CONFIG_dll, and SCAN_dll and are encapsulated in the
BiochemicalSensor class.
*/
#include <string>
#include <iostream>   
#include <pressure_ulcer/assessment/biochemical/biochemical_sensor.h>

using namespace gevxl::pressure_ulcer;

int main ()
{
	int rc;
	std::string comPort;
	std::cout << "Please enter the communication port of the sensor in the form COMx" << std::endl;
	std::cin >> comPort;
	/* Create the sensor object */
	gevxl::pressure_ulcer::BiochemicalSensor sensor ("COM4");
	/* Initialize the sensor and check for errors */
	rc = sensor.init();
	std::cout << "Init return code = " << rc << std::endl;
	std::cout << "Init response = " << sensor.getInitResponse() << std::endl;
	/* Configure the sensor */
	std::string configParms[] = {"startFrequency", "incrementFrequency", "offsetFrequency",
		"numberSamples", "numberPoints", "DDSDAC", "calibrationSwitch", "numberReps", "numberSkips", "flag"};
	std::string configValues[] = {"13000000", "20000", "12207.05",
		"128", "32", "257", "0", "1", "64", "0"};
	rc = sensor.createConfiguration(configParms,configValues);
	std::cout << "CreateConfiguration return code = " << rc << std::endl;
	std::cout << "CreateConfiguration response = " << sensor.getConfigError() << std::endl;
	rc = sensor.configure();
	std::cout << "Configure return code = " << rc << std::endl;
	std::cout << "Configure response = " << sensor.getConfigResponse() << std::endl;
	/* Do one scan */
	rc = sensor.scan();
	std::cout << "Scan return code = " << rc << std::endl;
	std::cout << "Scan response = " << sensor.getScanResponse() << std::endl;
	/* Exiting */
	std::cout << "Press any key to exit." << std::endl;
	std::cin >> comPort;


	return 0;
}
