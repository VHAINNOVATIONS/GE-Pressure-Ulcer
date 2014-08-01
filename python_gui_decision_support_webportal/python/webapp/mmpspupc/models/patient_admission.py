from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PatientAdmission(Base):
    """
    Definition of PatientAdmission object. It will be used by SQLAlchemy's ORM to map the object to
    the patient_admission table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'patient_admission'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    admission_date = Column('admission_date',DateTime)
    admission_note = Column('admission_note',String)
    factors_impairing_healing = Column('factors_impairing_healing',String)
    patient_group = Column('patient_group',String)
    keyCol = 'id'
    editCols = ['admission_date','admission_note','factors_impairing_healing','patient_group']
    editColsLabels = ['Admission Date','Admission Note','Factors Impairing Healing','Patient Group']
    editColsTypes = ['date','string','string']
    displayTableName = 'Patient Admission'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.admission_date = data['admission_date']
        self.admission_note = data['admission_note']
        self.factors_impairing_healing = data['factors_impairing_healing']
        self.patient_group = data['patient_group']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'admission_date':self.admission_date.isoformat(' '), 'admission_note':self.admission_note, 'factors_impairing_healing':self.factors_impairing_healing, 'patient_group':self.patient_group }

    def __repr__(self):
        return "<PatientAdmission(id='%d', patient_id='%d', admission_date='%s', admission_note='%s', factors_impairing_healing='%s', patient_group='%s')>" % (
                self.id, self.patient_id, self.admission_date, self.admission_note, self.factors_impairing_healing, self.patient_group)

