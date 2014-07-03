from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class NutritionalStatus(Base):
    """
    Definition of NutritionalStatus object. It will be used by SQLAlchemy's ORM to map the object to
    the nutritional_status table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'nutritional_status'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    assessment_date = Column('assessment_date',DateTime)
    nutritional_notes = Column('nutritional_notes',String)
    keyCol = 'id'
    editCols = ['assessment_date','nutritional_notes']
    editColsLabels = ['Assessment Date','Nutritional Notes']
    editColsTypes = ['date','string']
    displayTableName = 'Nutritional Status'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.assessment_date = data['assessment_date']
        self.nutritional_notes = data['nutritional_notes']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'assessment_date':self.assessment_date.isoformat(' '), 'nutritional_notes':self.nutritional_notes }

    def __repr__(self):
        return "<NutritionalStatus(id='%d', patient_id='%d', assessment_date='%s', nutritional_notes='%s')>" % (
                self.id, self.patient_id, self.assessment_date, self.nutritional_notes)

