from sqlalchemy import Column, Integer, String, DateTime, Boolean
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PreventionClinicalData(Base):
    """
    Definition of PreventionClinicalData object. It will be used by SQLAlchemy's ORM to map the object to
    the patient_turning table.
    Methods:
        prevention_clinical_data(data) - sets all data fields
    """
    __tablename__ = 'prevention_clinical_data'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    clinical_rounding_time = Column('clinical_rounding_time',DateTime)
    repositioning_flag = Column('repositioning_flag',Boolean)
    final_position = Column('final_position',String)
    repositioning_description = Column('repositioning_description',String)
    keyCol = 'id'
    editCols = ['clinical_rounding_time','repositioning_flag','final_position','repositioning_description']
    editColsLabels = ['Rounding Time','Repositioned?','Final Position','Notes']
    editColsTypes = ['date','boolean','string','string']
    displayTableName = 'Patient Prevention Clinical Data'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.patient_id = data['patient_id']
        self.clinical_rounding_time = data['clinical_rounding_time']
        self.repositioning_flag = data['repositioning_flag']
        self.final_position = data['final_position']
        self.repositioning_description = data['repositioning_description']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'clinical_rounding_time':self.clinical_rounding_time.isoformat(' '),
                'repositioning_flag':str(self.repositioning_flag), 'final_position':self.final_position,
                'repositioning_description':self.repositioning_description }

    def __repr__(self):
        return "<PreventionClinicalData(id='%d', patient_id='%d', clinical_rounding_time='%s', repositioning_flag='%s', final_position='%s', repositioning_description='%s')>" % (
                self.id, self.patient_id, self.clinical_rounding_time, self.repositioning_flag,
                self.final_position, self.repositioning_description)

