from  olv_dialog_controller import OlvDialogController
from clinical_wound_assessment import ClinicalWoundAssessment
from sqlalchemy import desc
from sqlalchemy import and_
from sqlalchemy import func
from datetime import datetime

class ClinicalWoundController(OlvDialogController):
    """
    Controller class for ClinicalWoundAssessment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getByPatientByWoundByNo(patientId,woundId,assessmentId) - Gets record for a patient, wound, and assessment
    """
    
    def getByPatientByWoundByNo(self, patientId, woundId, assessmentId):
        """
        Gets record for a patient, wound, and assessment
        """
        assessment = self.db.session.query(ClinicalWoundAssessment).filter(and_(ClinicalWoundAssessment.patient_id == patientId, ClinicalWoundAssessment.wound_id == woundId, ClinicalWoundAssessment.assessment_id == assessmentId)).first()
        return assessment

    def save(self, obj):
        """
        Saves an object
        Arguments:
            obj - Object to be modified
        """
        rc = 0
        msg = ""
        o = None
        try:
            o = self.db.session.merge(obj)
            self.db.session.commit()
        except Exception as e:
            rc = e.errno
            msg = e.strerror
            self.db.session.rollback()
        finally:
            return (o, rc, msg)

