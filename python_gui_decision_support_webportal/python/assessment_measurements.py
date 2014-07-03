from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AssessmentMeasurements(Base):
    """
    Definition of AssessmentMeasurements object. It will be used by SQLAlchemy's ORM to map the object to
    the system_assessment_experiment_measure table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_assessment_experiment_measure'
    id = Column('id',Integer, primary_key=True)
    session_id = Column('session_id',Integer)
    experiment_id = Column('experiment_id',Integer)
    start_time = Column('start_time',DateTime)
    length = Column('length',Numeric)
    width = Column('width',Numeric)
    depth = Column('depth',Numeric)
    length_x_width = Column('length_x_width',Numeric)
    surface_area = Column('surface_area',Numeric)
    wound_volume = Column('wound_volume',Numeric)
    push_score = Column('push_score',Numeric)
    keyCol = 'id'
    editCols = ['start_time','length', 'width', 'depth', 'length_x_width', 'surface_area', 'wound_volume', 'push_score' ]
    editColsLabels = ['Start Time','Length', 'Width', 'Depth', 'Length*Width', 'Surface Area', 'Wound Volume', 'Push Score']
    editColsTypes = ['date','numeric','numeric','numeric','numeric','numeric','numeric','numeric']
    displayTableName = 'System Assessment Experiment Measurements'
    
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
        self.length = data['length']
        self.width = data['width']
        self.depth = data['depth']
        self.length_x_width = data['length_x_width']
        self.surface_area = data['surface_area']
        self.wound_volume = data['wound_volume']
        self.push_score = data['push_score']
        
    
    def __json__(self, request):
        return {'id':self.id, 'session_id':self.session_id, 'experiment_id':self.experiment_id, 'start_time':self.start_time.isoformat(' '),
                'length':self.length, 'width':self.width, 'depth':self.depth, 'length_x_width':self.length_x_width,
                'surface_area':self.surface_area, 'wound_volume':self.wound_volume,'push_score':self.push_score  }

    def __repr__(self):
        return "<AssessmentMeasurements(id='%d', session_id='%d', experiment_id='%d', start_time='%s', length='%d', width='%d', depth='%d', length_x_width='%d', surface_area='%d', wound_volume='%d', push_score='%d')>" % (
                self.id, self.session_id, self.experiment_id, self.start_time, self.length, self.width, self.depth, self.length_x_width, self.surface_area, self.wound_volume, self.push_score)

