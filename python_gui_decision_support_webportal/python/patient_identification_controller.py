from  olv_dialog_controller import OlvDialogController
from patient_identification import PatientIdentification

class PatientIdentificationController(OlvDialogController):
    """
    Controller class for Wound Assessment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllByPatient(patientId) - Gets all records for a patient
    """
