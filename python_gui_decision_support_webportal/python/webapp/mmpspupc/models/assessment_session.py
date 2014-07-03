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
    depth_video_file_path = Column('depth_video_file_path',String)
    rgb_video_file_path = Column('rgb_video_file_path',String)
    hd_video_file_path = Column('hd_video_file_path',String)
    thermal_file_path = Column('thermal_file_path',String)
    biochemical_file_path = Column('biochemical_file_path',String)
    keyCol = 'id'
    editCols = ['start_time','depth_video_file_path', 'rgb_video_file_path', 'hd_video_file_path', 'thermal_file_path', 'biochemical_file_path']
    editColsLabels = ['Start Time','depth_video_file_path', 'rgb_video_file_path', 'hd_video_file_path', 'thermal_file_path', 'biochemical_file_path']
    editColsTypes = ['date','string','string','string','string','string']
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
        self.depth_video_file_path = data['depth_video_file_path']
        self.rgb_video_file_path = data['rgb_video_file_path']
        self.hd_video_file_path = data['hd_video_file_path']
        self.thermal_file_path = data['thermal_file_path']
        self.biochemical_file_path = data['biochemical_file_path']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'wound_id':self.wound_id, 'assessment_id':self.assessment_id, 'start_time':self.start_time.isoformat(' '),
                'depth_video_file_path':self.depth_video_file_path, 'rgb_video_file_path':self.rgb_video_file_path, 'hd_video_file_path':self.hd_video_file_path,
                'thermal_file_path':self.thermal_file_path, 'biochemical_file_path':self.biochemical_file_path }

    def __repr__(self):
        return "<AssessmentSession(id='%d', patient_id='%d', wound_id='%d', assessment_id='%d', start_time='%s', depth_video_file_path='%s', rgb_video_file_path='%s', hd_video_file_path='%s', thermal_file_path='%s', biochemical_file_path='%s')>" % (
                self.id, self.patient_id, self.wound_id, self.assessment_id, self.start_time, self.depth_video_file_path, self.rgb_video_file_path, self.hd_video_file_path, self.thermal_file_path, self.biochemical_file_path)

