from sqlalchemy import exc
import re
from  olv_dialog_controller import OlvDialogController
from prevention_session import PreventionSession
from sqlalchemy import desc
from sqlalchemy import and_
from sqlalchemy import func
from datetime import datetime

class PreventionSessionController(OlvDialogController):
    """
    Controller class for Prevention Session Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        add(o) - Creates a new Prevention
        update(o) - Updates a Prevention
    """
    
    def add(self, o):
        """
        Creates a new Prevention
        """
        rc = 0
        msg = ""
        try:
            print "PreventionSessionController...adding new PreventionSession: id="+str(o.patient_id)+" start="+o.start_time.isoformat()
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
        Updates a new Prevention
        """
        rc = 0
        msg = ""
        try:
            print "PreventionSessionController...updating PreventionSession: id="+str(o.patient_id)+" start="+o.start_time.isoformat()+" end="+o.end_time.isoformat()
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

