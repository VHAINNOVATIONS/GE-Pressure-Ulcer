# Copyright (C) 2014 General Electric Company
#
# This software is intellectual property of General Electric Co.
# and may not be copied or redistributed without express written consent.
#
import gevxl.security.pressure_ulcer.pressure_ulcer as pu
from patient_turning import PatientTurning
from datetime import datetime
from patient_turning_controller import PatientTurningController

class PreventionDatabaseWriter(pu.pu_prevention_database_writer):
    """
    This class is the concrete implementation of the abstract class defined in the C++
    pu_prevention_database_writer class file.
    It writes information into the patient_turning database table.
    Methods:
        __init__(db) - initializes the class
        setPatientId(patient_id) - Sets the patient id
        setSessionId(session_id) - Sets the session id
        setExperimentId(experiment_id) - Sets the experiment id
        write(provider_present_flag,final_position) - inserts an entry into the patient_turning table
    """
    def __init__(self, db):
        self.db = db
        self.patientTurningController = PatientTurningController(db, PatientTurning, None, None)
        super(PreventionDatabaseWriter,self).__init__()

    def setPatientId(self, patient_id):
        """ 
        Sets the patient id
        """
        self.patient_id = patient_id
    
    def setSessionId(self, session_id):
        """ 
        Sets the session id
        """
        self.session_id = session_id
    
    def setExperimentId(self, experiment_id):
        """ 
        Sets the experiment id
        """
        self.experiment_id = experiment_id
    
    def write(self, provider_present_flag,final_position):
        """
        Inserts a patient_turning record into the database
        """
        patientTurning = PatientTurning()
        patientTurning.patient_id = self.patient_id
        patientTurning.session_id = self.session_id
        patientTurning.experiment_id = self.experiment_id
        patientTurning.turn_time = datetime.now()
        patientTurning.provider_present_flag = provider_present_flag
        patientTurning.final_position = final_position
        self.patientTurningController.add(patientTurning)

