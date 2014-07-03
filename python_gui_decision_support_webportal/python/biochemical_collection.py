import os, errno
import gevxl.security.pressure_ulcer.pressure_ulcer as pu
from pu_file_director import PUFileDirectorException
from algorithm import Algorithm
from algorithm_controller import AlgorithmController
from algorithm_defaults import AlgorithmDefaults
from algorithm_defaults_controller import AlgorithmDefaultsController

class BiochemicalCollection:
    """
    Class which manages the biochemical sensor and perform the collection
    """
    def __init__(self, port, db, director):
        self.message = ""
        self.port = str(port)
        self.db = db
        self.director = director
        try:
            self.sensor = pu.BiochemicalSensor(self.port)
            self.sensor.init()
        except pu.BiochemicalSensorException as e:
            self.message = e.what()
        except Exception as e:
            print "Exception - ",e
            self.message = e

    def Stop(self):
        # close output files
        self.chem_file_1.close()

    def Setup(self, collection_type):
        self.collection_type = collection_type
        try:
            # get output directory
            print "here 0"
            chem_dir = self.director.GetBiochemicalFileDirectory()
            # print "chem_dir: "+chem_dir
            try:
                os.makedirs(chem_dir)
            except OSError as exc: # Python >2.5
                if exc.errno == errno.EEXIST and os.path.isdir(chem_dir):
                    pass
                else: raise
            # open output files
            if self.collection_type == "baseline":
                chem_file_1_path = os.path.join(chem_dir,"baseline_chem_file_1")
            else:
                chem_file_1_path = os.path.join(chem_dir,"chem_file_1")
            self.chem_file_1 = open(chem_file_1_path,"w")
            # load configuration parameters into sensor
            aController = AlgorithmController(self.db, Algorithm, None, None)
            adController = AlgorithmDefaultsController(self.db, AlgorithmDefaults, None, None)
            algorithm = aController.getByName("biochemical sensor")
            # (names,values) = adController.getAllAsLists(algorithm.id)
            # self.sensor.createConfiguration(names, values)
            # self.sensor.configure()
            nReps = int(adController.getDefaultByName("numberReps", algorithm.id))
            print "Configuring biochemical sensor for %d repetitions" % (nReps)
            self.sensor.configure(nReps)
        except PUFileDirectorException as e:
            self.message = e.to_string()
        except pu.BiochemicalSensorException as e:
            self.message = e.what()
        except Exception as e:
            print "Exception - ",e
            self.message = e

    def Collector(self):
        """
        Worker method which performs the data collection and storing of data.
        """
        try:
            # perform the scan
            self.sensor.scan()
            # retreive the data
            self.frequencies = self.sensor.getScanFrequencies()
            # self.voltages = self.sensor.getScanVoltages()
            # self.currents = self.sensor.getScanCurrents()
            self.impedances = self.sensor.getScanImpedances()
            # convert data to output format and write it out
            for i in range(0, len(self.frequencies)):
                self.chem_file_1.write( '{0:10}, {1.real:10}, {1.imag:10}\n'.format(self.frequencies[i],self.impedances[i]) )
        except pu.BiochemicalSensorException as e:
            self.message = e.what()
            self.Stop()
        
    def GetMessage(self):
        """
        Returns the error message
        """
        return self.message
    
    def GetFrequencies(self):
        """
        Returns the frequencies
        """
        return self.frequencies
    
    def GetImpedances(self):
        """
        Returns the impedances
        """
        return self.impedances
    
    