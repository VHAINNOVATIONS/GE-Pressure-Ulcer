from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PatientAssessment(Base):
    """
    Definition of PatientAssessment object. It will be used by SQLAlchemy's ORM to map the object to
    the patient_assessment table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'patient_assessment'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    assessment_date = Column('assessment_date',DateTime)
    assessment_note = Column('assessment_note',String)
    education_notes = Column('education_notes',String)
    education_understanding = Column('education_understanding',String)
    education_evidenced_by = Column('education_evidenced_by',String)
    keyCol = 'id'
    editCols = ['assessment_date','assessment_note','education_notes','education_understanding','education_evidenced_by']
    editColsLabels = ['Assessment Date','Assessment Note','Education Notes','Education Understanding','Education Evidenced By']
    editColsTypes = ['date','string']
    displayTableName = 'Patient Assessment'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.assessment_date = data['assessment_date']
        self.assessment_note = data['assessment_note']
        self.education_notes = data['education_notes']
        self.education_understanding = data['education_understanding']
        self.education_evidenced_by = data['education_evidenced_by']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'assessment_date':self.assessment_date.isoformat(' '), 'assessment_note':self.assessment_note, 'education_notes':self.education_notes, 'education_understanding':self.education_understanding, 'education_evidenced_by':self.education_evidenced_by }

    def __repr__(self):
        return "<PatientAssessment(id='%d', patient_id='%d', assessment_date='%s', assessment_note='%s', education_notes='%s', education_understanding='%s', education_evidenced_by='%s')>" % (
                self.id, self.patient_id, self.assessment_date, self.assessment_note, self.education_notes, self.education_understanding, self.education_evidenced_by)

