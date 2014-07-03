from  olv_dialog_controller import OlvDialogController
from prevention_clinical_data import PreventionClinicalData

class PreventionClinicalDataController(OlvDialogController):
    """
    Controller class for PreventionClinicalData Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllForOLView(patientId) - Gets all records for a patient
    """
    def getAllForOLView(self, patientId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if patientId < 0:
            result = self.db.session.query(PreventionClinicalData).all()
        else:
            result = self.db.session.query(PreventionClinicalData).filter(PreventionClinicalData.patient_id == patientId).all()
        olvResults = self.convertResults(result)
        print olvResults
        return olvResults
