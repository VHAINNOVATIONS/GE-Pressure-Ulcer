from sqlalchemy import Column, Integer, String, DateTime
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class BradenScores(Base):
    """
    Definition of BradenScores object. It will be used by SQLAlchemy's ORM to map the object to
    the braden_scores table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'braden_scores'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    braden_scoring_date = Column('braden_scoring_date',DateTime)
    sensory_perception_score = Column('sensory_perception_score',Integer)
    moisture_score = Column('moisture_score',Integer)
    activity_score = Column('activity_score',Integer)
    mobility_score = Column('mobility_score',Integer)
    nutrition_score = Column('nutrition_score',Integer)
    friction_shear_score = Column('friction_shear_score',Integer)
    keyCol = 'id'
    editCols = ['braden_scoring_date','sensory_perception_score','moisture_score','activity_score','mobility_score','nutrition_score','friction_shear_score']
    editColsLabels = ['Braden Scoring Date','Sensory Perception Score','Moisture Score','Activity Score','Mobility Score','Nutrition Score','Friction Shear Score']
    editColsTypes = ['date','int','int','int','int','int','int']
    displayTableName = 'Braden Scores'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.patient_id = data['patient_id']
        self.patient_id = data['patient_id']
        self.braden_scoring_date = data['braden_scoring_date']
        self.sensory_perception_score = data['sensory_perception_score']
        self.moisture_score = data['moisture_score']
        self.activity_score = data['activity_score']
        self.mobility_score = data['mobility_score']
        self.nutrition_score = data['nutrition_score']
        self.friction_shear_score = data['friction_shear_score']
        
    
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'braden_scoring_date':self.braden_scoring_date.isoformat(' '), 'sensory_perception_score':self.sensory_perception_score, 'moisture_score':self.moisture_score, 'activity_score':self.activity_score, 'mobility_score':self.mobility_score, 'nutrition_score':self.nutrition_score, 'friction_shear_score':self.friction_shear_score }

    def __repr__(self):
        return "<BradenScores(id='%d', patient_id='%d', braden_scoring_date='%s', sensory_perception_score='%d', moisture_score='%d', activity_score='%d', mobility_score='%d', nutrition_score='%d', friction_shear_score='%d')>" % (
                self.id, self.patient_id, self.braden_scoring_date, self.sensory_perception_score, self.moisture_score, self.activity_score, self.mobility_score, self.nutrition_score, self.friction_shear_score)

