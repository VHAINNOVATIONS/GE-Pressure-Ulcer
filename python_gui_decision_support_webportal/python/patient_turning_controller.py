from sqlalchemy import exc
import re
from  olv_dialog_controller import OlvDialogController
from patient_turning import PatientTurning
from sqlalchemy import and_
from sqlalchemy import desc

class PatientTurningController(OlvDialogController):
    """
    Controller class for Experiment Configuration Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        add(o) - Creates a new PatientTurning
        update(o) - Updates a PatientTurning
        getByPatientBySessionByExperiment(patientId, sessionId, experimentId) - Gets all records for a patient/session/experiment
    """
    def add(self, o):
        """
        Creates a new PatientTurning
        """
        rc = 0
        msg = ""
        try:
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

    def update(self, o):
        """
        Updates a PatientTurning
        """
        rc = 0
        msg = ""
        try:
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

    def getByPatientBySessionByExperiment(self, patientId, sessionId, experimentId):
        """
        Gets all records for a patient/session/experiment
        """
        result = None
        self.db.session.commit()
        result = self.db.session.query(PatientTurning).filter(and_(PatientTurning.patient_id == patientId,PatientTurning.session_id == sessionId,PatientTurning.experiment_id == experimentId)).order_by(desc(PatientTurning.turn_time)).all()
        olvResults = self.convertResults(result)
        return olvResults

