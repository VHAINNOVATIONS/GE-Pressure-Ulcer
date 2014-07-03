from sqlalchemy import exc
import re
from  olv_dialog_controller import OlvDialogController
from assessment_session import AssessmentSession
from sqlalchemy import desc
from sqlalchemy import and_
from sqlalchemy import func
from datetime import datetime

class AssessmentSessionController(OlvDialogController):
    """
    Controller class for Assessment Session Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllByPatientByWound(patientId,woundId) - Gets all records for a patient
        createNewAssessment(patientId, woundId) - Creates a new assessment
        getByPatientByWoundByAssessment(patientId,woundId,assessmentId) - Gets records for an assessment
    """
    
    def getAllByPatientByWound(self, patientId, woundId):
        """
        Gets all records by a patient
        """
        assessments = self.db.session.query(AssessmentSession).filter(and_(AssessmentSession.patient_id == patientId, AssessmentSession.wound_id == woundId)).order_by(desc(AssessmentSession.assessment_id)).all()
        return assessments

    def createNewAssessment(self, patientId, woundId):
        """
        Creates a new assessment (one larger than the previous one)
        """
        rc = 0
        msg = ""
        try:
            curAssessmentId = self.db.session.query(func.max(AssessmentSession.assessment_id)).filter(and_(AssessmentSession.patient_id == patientId, AssessmentSession.wound_id == woundId)).scalar()
            if curAssessmentId == None:
                curAssessmentId = 0
            curAssessmentId = curAssessmentId + 1
            o = AssessmentSession()
            o.patient_id = patientId
            o.wound_id = woundId
            o.assessment_id = curAssessmentId
            o.start_time = datetime.today()
            o.collection_status_visual = "Not collected"
            o.collection_status_multi_spectral = "Not collected"
            o.collection_status_chemical = "Not collected"
            o.collection_status_chemical_bl = "Not collected"
            print "AssessmentSessionController...adding new AssessmentSession: "+str(o.patient_id)+" / "+str(o.wound_id)+" / "+str(o.assessment_id)+" / "+o.start_time.isoformat()
            self.db.session.add(o)
            self.db.session.commit()
        except exc.SQLAlchemyError as e:
            print "caught exception of type: "
            print type(e)
            print e
            errorNumSearch = re.search('\(.*\) (\d+) (.*)',str(e))
            rc = int(errorNumSearch.group(1))
            print "error # "+str(rc)
            msg = str(e)
            self.db.session.rollback()
        except Exception as e:
            rc = -1
            msg = str(e)
            self.db.session.rollback()
        finally:
            return (rc, msg)

    def getByPatientByWoundByAssessment(self, patientId, woundId, assessmentId):
        """
        Gets records for an assessment
        """
        assessment = self.db.session.query(AssessmentSession).filter(and_(AssessmentSession.patient_id == patientId, AssessmentSession.wound_id == woundId, AssessmentSession.assessment_id == assessmentId)).first()
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
        except exc.SQLAlchemyError as e:
            print "caught exception of type: "
            print type(e)
            print e
            errorNumSearch = re.search('\(.*\) (\d+) (.*)',str(e))
            rc = int(errorNumSearch.group(1))
            print "error # "+str(rc)
            msg = str(e)
            self.db.session.rollback()
        except Exception as e:
            rc = -1
            msg = str(e)
            self.db.session.rollback()
        finally:
            return (o, rc, msg)
