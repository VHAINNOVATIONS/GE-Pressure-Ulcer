/*
The purpose of this program is to test the pressure ulcer assessment biochemical
sensor Dll created by LabVIEW in C. This program calls three functions from the AllDLL.dll.
These functions are INIT_dll, CONFIG_dll, and SCAN_dll and are encapsulated in the
BiochemicalSensor class.
*/
#include <vcl_iostream.h>
#include <iostream>
#include <iomanip>
#include <exception>
#include <pressure_ulcer/assessment/biochemical/biochemical_sensor.h>

using namespace gevxl::pressure_ulcer;

int main ()
{
	int rc;
	vcl_string comPort;
	vcl_vector<double> scanFrequencies;
	vcl_vector<double> scanFrequenciesRaw;
	vcl_vector<vcl_complex<double> > scanVoltages;
	vcl_vector<vcl_complex<double> > scanCurrents;
	vcl_vector<vcl_complex<double> > scanImpedances;
	vcl_vector<vcl_complex<double> > scanImpedancesRaw;
	try {
		vcl_cout << "Please enter the communication port of the sensor in the form COMx" << vcl_endl;
		vcl_cin >> comPort;
		/* Create the sensor object */
		gevxl::pressure_ulcer::BiochemicalSensor sensor (comPort);
		/* Initialize the sensor and check for errors */
		sensor.init();
		vcl_cout << "Sensor initialized." << vcl_endl;
		/* Configure the sensor */
		/*
		vcl_vector<vcl_string> configParms;
		configParms.push_back("startFrequency");
		configParms.push_back("incrementFrequency");
		configParms.push_back("offsetFrequency");
		configParms.push_back("numberSamples");
		configParms.push_back("numberPoints");
		configParms.push_back("DDSDAC");
		configParms.push_back("calibrationSwitch");
		configParms.push_back("numberReps");
		configParms.push_back("numberSkips");
		configParms.push_back("flag");
		vcl_vector<vcl_string> configValues;
		configValues.push_back("12000000");
		configValues.push_back("20000");
		configValues.push_back("12207.05");
		configValues.push_back("128");
		configValues.push_back("128");
		configValues.push_back("257");
		configValues.push_back("0");
		configValues.push_back("17");
		configValues.push_back("64");
		configValues.push_back("0");
		sensor.createConfiguration(configParms,configValues);
		vcl_cout << "Sensor configuration created." << vcl_endl;
		sensor.setNumberReps(1);
		sensor.configure();
		*/
		int nReps = 17;
		sensor.configure(nReps);
		vcl_cout << "Sensor configured." << vcl_endl;
		/* Do one scan */
		rc = sensor.scan();
		// vcl_cout << "Scan return code = " << rc << vcl_endl;
		// vcl_cout << "Scan time = " << sensor.getScanTime() << " ms." << vcl_endl;
		scanFrequencies = sensor.getScanFrequencies();
		// scanFrequenciesRaw = sensor.getScanFrequenciesRaw();
		// scanVoltages = sensor.getScanVoltages();
		// scanCurrents = sensor.getScanCurrents();
		scanImpedances = sensor.getScanImpedances();
		// scanImpedancesRaw = sensor.getScanImpedancesRaw();
		vcl_cout << "Number of scan points = " << scanFrequencies.size() << vcl_endl;
		// vcl_cout << "Frequency, Impedance(r), Impedance(i), Frequency(raw), Impedance(r-raw), Impedance(i-raw), Voltage(r-raw), Voltage(i-raw), Current(r-raw), Current(i-raw),"  << vcl_endl;
		vcl_cout << "Frequency, Impedance(r), Impedance(i)"  << vcl_endl;
		for (long i=0; i<(long)scanFrequencies.size(); i++) {
			double f = scanFrequencies.at(i);
			// double fRaw = scanFrequenciesRaw.at(i);
			// vcl_complex<double> voltage = scanVoltages.at(i);
			// vcl_complex<double> current = scanCurrents.at(i);
			vcl_complex<double> impedance = scanImpedances.at(i);
			// vcl_complex<double> impedanceRaw = scanImpedancesRaw.at(i);
			// vcl_cout << std::setprecision(16) << f << ", " << std::real(impedance) << ", " << std::imag(impedance) << ", " << 
			// 	fRaw << ", " << std::real(impedanceRaw) << ", " << std::imag(impedanceRaw) << ", " <<
			// 	std::real(voltage) << ", " << std::imag(voltage) << ", " <<
			// 	std::real(current) << ", " << std::imag(current) << vcl_endl;
			vcl_cout << std::setprecision(16) << f << ", " << std::real(impedance) << ", " << std::imag(impedance) << vcl_endl;
		}
		vcl_cout << "Peak values: f = " << std::setprecision(16) << sensor.getFPeak() << "    Z-real = " << sensor.getZrPeak() << vcl_endl;
		/* Exiting */
		vcl_cout << "Press any key to exit." << vcl_endl;
		vcl_cin >> comPort;
	} catch (BiochemicalSensorException& e) {
		vcl_cout << e.what() << vcl_endl;
		vcl_cout << "Press any key to exit." << vcl_endl;
		vcl_cin >> comPort;
	} catch (std::exception& e) {
		vcl_cout << e.what() << vcl_endl;
		vcl_cout << "Press any key to exit." << vcl_endl;
		vcl_cin >> comPort;
	}
}
