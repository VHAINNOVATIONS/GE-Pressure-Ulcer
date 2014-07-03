from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AssessmentSession(Base):
    """
    Definition of AssessmentSession object. It will be used by SQLAlchemy's ORM to map the object to
    the system_assessment_experiment_measure table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_assessment_session'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    wound_id = Column('wound_id',Integer)
    assessment_id = Column('assessment_id',Integer)
    start_time = Column('start_time',DateTime)
    collection_status_visual = Column('collection_status_visual',String)
    collection_status_multi_spectral = Column('collection_status_multi_spectral',String)
    collection_status_chemical = Column('collection_status_chemical',String)
    collection_status_chemical_bl = Column('collection_status_chemical_bl',String)
    keyCol = 'id'
    editCols = ['start_time']
    editColsLabels = ['Start Time']
    editColsTypes = ['date']
    displayTableName = 'System Assessment Experiment Session'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.patient_id = data['patient_id']
        self.wound_id = data['wound_id']
        self.assessment_id = data['assessment_id']
        self.start_time = data['start_time']
        self.collection_status_visual = data['collection_status_visual']
        self.collection_status_multi_spectral = data['collection_status_multi_spectral']
        self.collection_status_chemical = data['collection_status_chemical']
        self.collection_status_chemical_bl = data['collection_status_chemical_bl']
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'wound_id':self.wound_id, 'assessment_id':self.assessment_id, 'start_time':self.start_time.isoformat(' '),
                'collection_status_visual':self.collection_status_visual, 'collection_status_multi_spectral':self.collection_status_multi_spectral,
                'collection_status_chemical':self.collection_status_chemical, 'collection_status_chemical_bl':self.collection_status_chemical_bl }

    def __repr__(self):
        return "<AssessmentSession(id='%d', patient_id='%d', wound_id='%d', assessment_id='%d', start_time='%s', collection_status_visual='%s', collection_status_multi_spectral='%s', collection_status_chemical='%s', collection_status_chemical_bl='%s')>" % (
                self.id, self.patient_id, self.wound_id, self.assessment_id, self.start_time,
                self.collection_status_visual, self.collection_status_multi_spectral, self.collection_status_chemical, self.collection_status_chemical_bl)

