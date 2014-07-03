from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class TreatmentPlan(Base):
    """
    Definition of TreatmentPlan object. It will be used by SQLAlchemy's ORM to map the object to
    the treatment_plan table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'treatment_plan'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    plan_date = Column('plan_date',DateTime)
    plan_notes = Column('plan_notes',String)
    keyCol = 'id'
    editCols = ['plan_date','plan_notes']
    editColsLabels = ['Plan Date','Plan Notes']
    editColsTypes = ['date','string']
    displayTableName = 'Treatment Plan'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.plan_date = data['plan_date']
        self.plan_notes = data['plan_notes']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'plan_date':self.plan_date.isoformat(' '), 'plan_notes':self.plan_notes }

    def __repr__(self):
        return "<TreatmentPlan(id='%d', patient_id='%d', plan_date='%s', plan_notes='%s')>" % (
                self.id, self.patient_id, self.plan_date, self.plan_notes)

