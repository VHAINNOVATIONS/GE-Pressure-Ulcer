import os
import fnmatch
import shutil

class PUFileDirector:
    """
    The PUFileDirector class provides methods that return various types of file paths
    to be used to create/read/write data files pertaining to camera data, sensor data,
    experiment analysis, etc.
    Constructor:
        PUFileDirector(base_file_directory)
    Methods:
        SetPatientId(patientId) - Sets the patient
        SetSystemType(systemType) - Sets the system type ('prevention', 'assessment')
        SetWoundId(woundId) - Sets the wound
        SetAssessmentId(assessmentId) - Sets the assessment id
        SetExperimentId(experimentId) - Sets the experiment id
        SetSessionId(sessionId) - Sets the prevention session id
        GetDepthFileDirectory() - Gets the file directory for depth files
        GetRgbFileDirectory() - Gets the file directory for rgb files
        GetThermalFileDirectory() - Gets the file directory for thermal files
        GetMultiSpectralFileDirectory() - Gets the file directory for multi-spectral files
        GetBiochemicalFileDirectory() - Gets the file directory for biochemical files
        CleanDepthFileDirectory() - Cleans the file directory for depth files
        CleanRgbFileDirectory() - Cleans the file directory for rgb files
        CleanThermalFileDirectory() - Cleans the file directory for thermal files
        CleanMultiSpectralFileDirectory() - Cleans the file directory for multi-spectral files
        CleanBiochemicalFileDirectory() - Cleans the file directory for biochemical files
    """
    preventionName = "prevention"
    assessmentName = "assessment"
    depthName = "depth"
    rgbName = "rgb"
    thermalName = "thermal"
    mutlispectralName = "multispectral"
    biochemicalName = "biochemical"
    expermimentName = "experiments"
    
    def __init__(self, base_file_directory):
        """
        Initializes the class with the base file directory
        """
        self.base_file_directory = base_file_directory
    
    def SetPatientId(self, patientId):
        """
        Sets the patient
        """
        self.patientId = patientId
    
    def SetSystemType(self,systemType):
        """
        Sets the system type ('prevention', 'assessment')
        """
        self.systemType = systemType

    def SetWoundId(self, woundId):
        """
        Sets the wound
        """
        self.woundId = woundId
        
    def SetAssessmentId(self, assessmentId):
        """
        Sets the assessment id
        """
        self.assessmentId = assessmentId
        
    def SetExperimentId(self, experimentId):
        """
        Sets the experiment id
        """
        self.experimentId = experimentId
        
    def SetSessionId(self, sessionId):
        """
        Sets the prevention session id
        """
        self.sessionId = sessionId
        
    def GetDepthFileDirectory(self):
        """
        Gets the file directory for depth files
        """
        if not hasattr(self, 'patientId'):
            raise PUFileDirectorException("Patient Id not set")
        if not hasattr(self, 'systemType'):
            raise PUFileDirectorException("System Type not set")
        if self.systemType == "prevention":
            if not hasattr(self, 'sessionId'):
                raise PUFileDirectorException("Session Id not set")
            return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.preventionName, 
                                str(self.sessionId), PUFileDirector.depthName)
        else:
            if not hasattr(self, 'woundId'):
                raise PUFileDirectorException("Wound Id not set")
            if not hasattr(self, 'assessmentId'):
                raise PUFileDirectorException("Assessment Id not set")
            return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.assessmentName, 
                                str(self.woundId), str(self.assessmentId), PUFileDirector.depthName)
            
    def GetRgbFileDirectory(self):
        """Gets the file directory for rgb files
        
        """
        if not hasattr(self, 'patientId'):
            raise PUFileDirectorException("Patient Id not set")
        if not hasattr(self, 'systemType'):
            raise PUFileDirectorException("System Type not set")
        if self.systemType == "prevention":
            raise PUFileDirectorException("System Type not assessment")
        if not hasattr(self, 'woundId'):
            raise PUFileDirectorException("Wound Id not set")
        if not hasattr(self, 'assessmentId'):
            raise PUFileDirectorException("Assessment Id not set")
        return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.assessmentName, 
                            str(self.woundId), str(self.assessmentId), PUFileDirector.rgbName)

    def GetThermalFileDirectory(self):
        """
        Gets the file directory for thermal files
        """
        if not hasattr(self, 'patientId'):
            raise PUFileDirectorException("Patient Id not set")
        if not hasattr(self, 'systemType'):
            raise PUFileDirectorException("System Type not set")
        if self.systemType == "prevention":
            raise PUFileDirectorException("System Type not assessment")
        if not hasattr(self, 'woundId'):
            raise PUFileDirectorException("Wound Id not set")
        if not hasattr(self, 'assessmentId'):
            raise PUFileDirectorException("Assessment Id not set")
        return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.assessmentName, 
                            str(self.woundId), str(self.assessmentId), PUFileDirector.thermalName)

    def GetMultiSpectralFileDirectory(self):
        """
        Gets the file directory for multi-spectral files
        """
        if not hasattr(self, 'patientId'):
            raise PUFileDirectorException("Patient Id not set")
        if not hasattr(self, 'systemType'):
            raise PUFileDirectorException("System Type not set")
        if self.systemType == "prevention":
            raise PUFileDirectorException("System Type not assessment")
        if not hasattr(self, 'woundId'):
            raise PUFileDirectorException("Wound Id not set")
        if not hasattr(self, 'assessmentId'):
            raise PUFileDirectorException("Assessment Id not set")
        return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.assessmentName, 
                            str(self.woundId), str(self.assessmentId), PUFileDirector.mutlispectralName)

    def GetBiochemicalFileDirectory(self):
        """
        Gets the file directory for biochemical files
        """
        if not hasattr(self, 'patientId'):
            raise PUFileDirectorException("Patient Id not set")
        if not hasattr(self, 'systemType'):
            raise PUFileDirectorException("System Type not set")
        if self.systemType == "prevention":
            raise PUFileDirectorException("System Type not assessment")
        if not hasattr(self, 'woundId'):
            raise PUFileDirectorException("Wound Id not set")
        if not hasattr(self, 'assessmentId'):
            raise PUFileDirectorException("Assessment Id not set")
        print "joining path"
        return os.path.join(self.base_file_directory, str(self.patientId), PUFileDirector.assessmentName, 
                            str(self.woundId), str(self.assessmentId), PUFileDirector.biochemicalName)

    def CleanRgbFileDirectory(self):
        """Cleans the file directory for rgb files
        
        """
        d = self.GetRgbFileDirectory()
        if d != "":
            for f in os.listdir(d):
                if fnmatch.fnmatch(f,'north') or fnmatch.fnmatch(f,'south') or fnmatch.fnmatch(f,'east') or fnmatch.fnmatch(f,'west') or fnmatch.fnmatch(f,'center'):
                    fpath = os.path.join(d,f)
                    print "Deleting directory: " + fpath
                    shutil.rmtree(fpath)

        
    def CleanDepthFileDirectory(self):
        """Cleans the file directory for depth files
        
        """
        d = self.GetDepthFileDirectory()
        if d != "":
            for f in os.listdir(d):
                if fnmatch.fnmatch(f,'north') or fnmatch.fnmatch(f,'south') or fnmatch.fnmatch(f,'east') or fnmatch.fnmatch(f,'west') or fnmatch.fnmatch(f,'center'):
                    fpath = os.path.join(d,f)
                    print "Deleting directory: " + fpath
                    shutil.rmtree(fpath)

        
    def CleanThermalFileDirectory(self):
        """Cleans the file directory for thermal files
        
        """
        d = self.GetThermalFileDirectory()
        if d != "":
            for f in os.listdir(d):
                if fnmatch.fnmatch(f,'north') or fnmatch.fnmatch(f,'south') or fnmatch.fnmatch(f,'east') or fnmatch.fnmatch(f,'west') or fnmatch.fnmatch(f,'center'):
                    fpath = os.path.join(d,f)
                    print "Deleting directory: " + fpath
                    shutil.rmtree(fpath)

        
    def CleanMultiSpectralFileDirectory(self):
        """Cleans the file directory for Multi-Spectral files
        
        """
        d = self.GetMultiSpectralFileDirectory()
        if d != "":
            for f in os.listdir(d):
                if fnmatch.fnmatch(f,'north') or fnmatch.fnmatch(f,'south') or fnmatch.fnmatch(f,'east') or fnmatch.fnmatch(f,'west') or fnmatch.fnmatch(f,'center'):
                    fpath = os.path.join(d,f)
                    print "Deleting directory: " + fpath
                    shutil.rmtree(fpath)

        
    def CleanBiochemicalFileDirectory(self):
        """Cleans the file directory for Biochemical files
        
        """
        d = self.GetBiochemicalFileDirectory()
        if d != "":
            for f in os.listdir(d):
                if fnmatch.fnmatch(f,'*.dat'):
                    fpath = os.path.join(d,f)
                    print "Deleting file: " + fpath
                    os.remove(fpath)

        
class PUFileDirectorException(Exception):
    """
    Exception definition for exceptions raised in the PUFileDirector class
    """
    def __init__(self, value):
        self.value = value
    def __str__(self):
        return repr(self.value)
