from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AssessmentReconstruction(Base):
    """
    Definition of AssessmentReconstruction object. It will be used by SQLAlchemy's ORM to map the object to
    the system_assessment_experiment_recon table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_assessment_experiment_measure'
    id = Column('id',Integer, primary_key=True)
    session_id = Column('session_id',Integer)
    experiment_id = Column('experiment_id',Integer)
    start_time = Column('start_time',DateTime)
    reconstruction_data_directory = Column('reconstruction_data_directory',String)
    keyCol = 'id'
    editCols = ['start_time','reconstruction_data_directory' ]
    editColsLabels = ['Start Time','Reconstruction Data Directory']
    editColsTypes = ['date','string']
    displayTableName = 'System Assessment Experiment Reconstruction'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.session_id = data['session_id']
        self.experiment_id = data['experiment_id']
        self.start_time = data['start_time']
        self.reconstruction_data_directory = data['reconstruction_data_directory']
        
    
    def __json__(self, request):
        return {'id':self.id, 'session_id':self.session_id, 'experiment_id':self.experiment_id, 'start_time':self.start_time.isoformat(' '),
                'reconstruction_data_directory':self.reconstruction_data_directory  }

    def __repr__(self):
        return "<AssessmentReconstruction(id='%d', session_id='%d', experiment_id='%d', start_time='%s', reconstruction_data_directory='%d')>" % (
                self.id, self.session_id, self.experiment_id, self.start_time, self.reconstruction_data_directory)

