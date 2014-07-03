from  olv_dialog_controller import OlvDialogController
from wound_assessment import WoundAssessment
from patient_identification import PatientIdentification
from skeleton_model import SkeletonModel
from sqlalchemy.sql import func

class WoundAssessmentController(OlvDialogController):
    """
    Controller class for Wound Assessment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllByPatient(patientId) - Gets all records for a patient
        getMaxWoundId(patientId) - Gets the maximum wound id for a patient
    """
    
    def getAllByPatient(self, patientId):
        """
        Gets all records by a patient
        """
        wounds = self.db.session.query(WoundAssessment).filter(WoundAssessment.patient_id == patientId).order_by(WoundAssessment.wound_location_description).all()
        return wounds

    def getMaxWoundId(self, patientId):
        """
        Gets the maximum wound id for a patient
        """
        woundId = self.db.session.query(func.max(WoundAssessment.wound_id)).filter(WoundAssessment.patient_id == patientId).scalar()
        return woundId

    def getAllForOLView(self, patientId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if patientId < 0:
            result = self.db.session.query(WoundAssessment.id,PatientIdentification.patient_name,SkeletonModel.body_part_code,SkeletonModel.description,WoundAssessment.wound_location_description).join(PatientIdentification, WoundAssessment.patient_id == PatientIdentification.patient_id).join(SkeletonModel, WoundAssessment.wound_location_id == SkeletonModel.location_id).all()
        else:
            result = self.db.session.query(WoundAssessment.id,PatientIdentification.patient_name,SkeletonModel.body_part_code,SkeletonModel.description,WoundAssessment.wound_location_description).join(PatientIdentification, WoundAssessment.patient_id == PatientIdentification.patient_id).join(SkeletonModel, WoundAssessment.wound_location_id == SkeletonModel.location_id).filter(WoundAssessment.patient_id == patientId).all()
        olvResults = self.convertResults(result)
        print olvResults
        return olvResults
