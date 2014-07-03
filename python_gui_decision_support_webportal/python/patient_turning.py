from sqlalchemy import Column, Integer, String, DateTime, Boolean
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PatientTurning(Base):
    """
    Definition of PatientTurning object. It will be used by SQLAlchemy's ORM to map the object to
    the patient_turning table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'patient_turning'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    session_id = Column('session_id',Integer)
    experiment_id = Column('experiment_id',Integer)
    turn_time = Column('turn_time',DateTime)
    final_position = Column('final_position',String)
    provider_present_flag = Column('provider_present_flag',Boolean)
    keyCol = 'id'
    editCols = ['turn_time','final_position','provider_present_flag']
    editColsLabels = ['Turn Time','Final Position','Provider Present?']
    editColsTypes = ['date','string','boolean']
    displayTableName = 'Patient Turning'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.session_id = data['session_id']
        self.experiment_id = data['experiment_id']
        self.turn_time = data['turn_time']
        self.final_position = data['final_position']
        self.provider_present_flag = data['provider_present_flag']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'session_id':self.session_id, 'experiment_id':self.experiment_id, 'turn_time':self.turn_time.isoformat(' '), 'final_position':self.final_position, 'provider_present_flag':str(self.provider_present_flag) }

    def __repr__(self):
        return "<PatientTurning(id='%d', patient_id='%d', session_id='%d', experiment_id='%d', turn_time='%s', final_position='%s', provider_present_flag='%s')>" % (
                self.id, self.patient_id, self.session_id, self.experiment_id, self.turn_time, self.final_position, self.provider_present_flag)

