from sqlalchemy import Column, Integer, String, DateTime, Numeric, Float
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AssessmentTemperature(Base):
    """
    Definition of AssessmentTemperature object. It will be used by SQLAlchemy's ORM to map the object to
    the system_assessment_experiment_temperature table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_assessment_experiment_temperature'
    id = Column('id',Integer, primary_key=True)
    session_id = Column('session_id',Integer)
    experiment_id = Column('experiment_id',Integer)
    start_time = Column('start_time',DateTime)
    max_temperature = Column('max_temperature',Numeric)
    max_temperature_loc_x = Column('max_temperature_loc_x',Float)
    max_temperature_loc_y = Column('max_temperature_loc_y',Float)
    min_temperature = Column('min_temperature',Numeric)
    min_temperature_loc_x = Column('min_temperature_loc_x',Float)
    min_temperature_loc_y = Column('min_temperature_loc_y',Float)
    baseline_temperature = Column('baseline_temperature',Numeric)
    baseline_description = Column('baseline_description',String)
    temperature_variation_sigma = Column('temperature_variation_sigma',Float)
    temperature_segment_1_percentage = Column('temperature_segment_1_percentage',Numeric)
    temperature_segment_2_percentage = Column('temperature_segment_2_percentage',Numeric)
    temperature_segment_3_percentage = Column('temperature_segment_3_percentage',Numeric)
    temperature_segment_4_percentage = Column('temperature_segment_4_percentage',Numeric)
    temperature_segment_5_percentage = Column('temperature_segment_5_percentage',Numeric)
    keyCol = 'id'
    editCols = ['start_time','max_temperature', 'max_temperature_loc_x', 'max_temperature_loc_y','min_temperature', 'min_temperature_loc_x', 'min_temperature_loc_y', 'baseline_temperature', 'baseline_description', 'temperature_variation_sigma', 'temperature_segment_1_percentage', 'temperature_segment_2_percentage', 'temperature_segment_3_percentage', 'temperature_segment_4_percentage', 'temperature_segment_5_percentage' ]
    editColsLabels = ['Start Time','Max. Temperature', 'Max. Temperature (x)', 'Max. Temperature (y)','Min. Temperature', 'Min. Temperature (x)', 'Min. Temperature (y)', 'Baseline Temperature', 'Baseline Description', 'Sigma', 'Segment 1 %', 'Segment 2 %', 'Segment 3 %', 'Segment 4 %', 'Segment 5 %']
    editColsTypes = ['date','numeric','float','float','numeric','float','float','numeric','string','float','numeric','numeric','numeric','numeric','numeric','numeric']
    displayTableName = 'System Assessment Experiment Temperature'
    
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
        self.max_temperature = data['max_temperature']
        self.max_temperature_loc_x = data['max_temperature_loc_x']
        self.max_temperature_loc_y = data['max_temperature_loc_y']
        self.max_temperature = data['min_temperature']
        self.max_temperature_loc_x = data['min_temperature_loc_x']
        self.max_temperature_loc_y = data['min_temperature_loc_y']
        self.baseline_temperature = data['baseline_temperature']
        self.baseline_description = data['baseline_description']
        self.temperature_variation_sigma = data['temperature_variation_sigma']
        self.temperature_segment_1_percentage = data['temperature_segment_1_percentage']
        self.temperature_segment_2_percentage = data['temperature_segment_2_percentage']
        self.temperature_segment_3_percentage = data['temperature_segment_3_percentage']
        self.temperature_segment_4_percentage = data['temperature_segment_4_percentage']
        self.temperature_segment_5_percentage = data['temperature_segment_5_percentage']
        
    
    def __json__(self, request):
        return {'id':self.id, 'session_id':self.session_id, 'experiment_id':self.experiment_id, 'start_time':self.start_time.isoformat(' '),
                'max_temperature':self.max_temperature, 'max_temperature_loc_x':self.max_temperature_loc_x, 'max_temperature_loc_y':self.max_temperature_loc_y,
                'min_temperature':self.min_temperature, 'min_temperature_loc_x':self.min_temperature_loc_x, 'min_temperature_loc_y':self.min_temperature_loc_y,
                'baseline_temperature':self.baseline_temperature, 'baseline_description':self.baseline_description,
                'temperature_variation_sigma':self.temperature_variation_sigma,
                'temperature_segment_1_percentage':self.temperature_segment_1_percentage, 'temperature_segment_2_percentage':self.temperature_segment_2_percentage,
                'temperature_segment_3_percentage':self.temperature_segment_3_percentage, 'temperature_segment_4_percentage':self.temperature_segment_4_percentage,
                'temperature_segment_5_percentage':self.temperature_segment_5_percentage  }

    def __repr__(self):
        return "<AssessmentTemperature(id='%d', session_id='%d', experiment_id='%d', start_time='%s', max_temperature='%d', max_temperature_loc_x='%d', max_temperature_loc_y='%d', min_temperature='%d', min_temperature_loc_x='%d', min_temperature_loc_y='%d', baseline_temperature='%d', baseline_description='%s', temperature_variation_sigma='%d', temperature_segment_1_percentage='%d', temperature_segment_2_percentage='%d', temperature_segment_3_percentage='%d', temperature_segment_4_percentage='%d', temperature_segment_5_percentage='%d')>" % (
                self.id, self.session_id, self.experiment_id, self.start_time, self.max_temperature, self.max_temperature_loc_x, self.max_temperature_loc_y, self.min_temperature, self.min_temperature_loc_x, self.min_temperature_loc_y, self.baseline_temperature, self.baseline_description, self.temperature_variation_sigma, self.temperature_segment_1_percentage, self.temperature_segment_2_percentage, self.temperature_segment_3_percentage, self.temperature_segment_4_percentage, self.temperature_segment_5_percentage)

