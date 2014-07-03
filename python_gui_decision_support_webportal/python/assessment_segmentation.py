from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AssessmentSegmentation(Base):
    """
    Definition of AssessmentSegmentation object. It will be used by SQLAlchemy's ORM to map the object to
    the system_assessment_experiment_segment table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_assessment_experiment_segment'
    id = Column('id',Integer, primary_key=True)
    session_id = Column('session_id',Integer)
    experiment_id = Column('experiment_id',Integer)
    start_time = Column('start_time',DateTime)
    image_label_map_file = Column('image_label_map_file',String)
    granulation_percentage = Column('granulation_percentage',Numeric)
    slough_percentage = Column('slough_percentage',Numeric)
    eschar_percentage = Column('eschar_percentage',Numeric)
    bone_percentage = Column('bone_percentage',Numeric)
    ulcer_stage = Column('ulcer_stage',Numeric)
    keyCol = 'id'
    editCols = ['start_time','image_label_map_file', 'granulation_percentage', 'slough_percentage', 'eschar_percentage', 'bone_percentage','ulcer_stage']
    editColsLabels = ['Start Time','image_label_map_file', 'Granulation Percentage', 'Slough Percentage', 'Eschar Percentage', 'Bone Percentage','Ulcer Stage']
    editColsTypes = ['date','string','numeric','numeric','numeric','numeric','numeric']
    displayTableName = 'System Assessment Experiment Segmentation'
    
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
        self.image_label_map_file = data['image_label_map_file']
        self.granulation_percentage = data['granulation_percentage']
        self.slough_percentage = data['slough_percentage']
        self.eschar_percentage = data['eschar_percentage']
        self.bone_percentage = data['bone_percentage']
        self.ulcer_stage = data['ulcer_stage']
    
    def __json__(self, request):
        return {'id':self.id, 'session_id':self.session_id, 'experiment_id':self.experiment_id, 'start_time':self.start_time.isoformat(' '),
                'image_label_map_file':self.image_label_map_file, 'granulation_percentage':self.granulation_percentage, 'slough_percentage':self.slough_percentage, 'eschar_percentage':self.eschar_percentage, 'bone_percentage':self.bone_percentage, 'ulcer_stage':self.ulcer_stage }

    def __repr__(self):
        return "<AssessmentSegmentation(id='%d', session_id='%d', experiment_id='%d', start_time='%s', image_label_map_file='%d', granulation_percentage='%d', slough_percentage='%d', eschar_percentage='%d', bone_percentage='%d', ulcer_stage='%d')>" % (
                self.id, self.session_id, self.experiment_id, self.start_time, self.image_label_map_file, self.granulation_percentage, self.slough_percentage, self.eschar_percentage, self.bone_percentage, self.ulcer_stage)

