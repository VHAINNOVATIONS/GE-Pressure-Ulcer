from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PreventionSession(Base):
    """
    Definition of PreventionSession object. It will be used by SQLAlchemy's ORM to map the object to
    the prevention_session table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'prevention_session'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    depth_video_file_directory = Column('depth_video_file_directory',String)
    start_time = Column('start_time',DateTime)
    end_time = Column('end_time',DateTime)
    keyCol = 'id'
    editCols = ['patient_id','depth_video_file_directory','start_time','end_time']
    editColsLabels = ['patient Id','Depth Video File Directory','Start Time','End Time']
    editColsTypes = ['number','string','date','date']
    displayTableName = 'Prevention Session'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.patient_id = data['patient_id']
        self.depth_video_file_directory = data['depth_video_file_directory']
        self.start_time = data['start_time']
        self.end_time = data['end_time']
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'depth_video_file_directory':self.depth_video_file_directory, 
                'start_time':self.start_time.isoformat(' '), 'end_time':self.end_time.isoformat(' ')}

    def __repr__(self):
        return "<PreventionSession(id='%d', patient_id='%d', depth_video_file_directory='%d', start_time='%s', end_time='%s')>" % (
                self.id, self.patient_id, self.depth_video_file_directory, self.start_time, self.end_time)

