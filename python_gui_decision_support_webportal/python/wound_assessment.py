from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class WoundAssessment(Base):
    """
    Definition of WoundAssessment object. It will be used by SQLAlchemy's ORM to map the object to
    the wound_assessment_map table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'wound_assessment_map'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    wound_id = Column('wound_id',Integer)
    wound_location_id = Column('wound_location_id',Integer)
    wound_location_description = Column('wound_location_description',String)
    keyCol = 'id'
    editCols = ['patient_id','wound_id','wound_location_id','wound_location_description']
    editColsLabels = ['Patient Id','Id','Location','Wound Location Description']
    editColsTypes = ['integer','integer','integer','string']
    displayTableName = 'Wound Assessment Map'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.patient_id = data['patient_id']
        self.wound_id = data['wound_id']
        self.wound_location_id = data['wound_location_id']
        self.wound_location_description = data['wound_location_description']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'wound_id':self.wound_id, 
                'wound_location_id':self.wound_location_id, 'wound_location_description':self.wound_location_description }

    def __repr__(self):
        return "<WoundAssessment(id='%d', patient_id='%d', wound_id='%d', wound_location_id='%d', wound_location_description='%s')>" % (
                self.id, self.patient_id, self.wound_id, self.wound_location_id, self.wound_location_description)
