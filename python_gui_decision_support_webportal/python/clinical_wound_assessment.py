from sqlalchemy import Column, Integer, String, DateTime, Numeric, Boolean
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class ClinicalWoundAssessment(Base):
    """
    Definition of ClinicalWoundAssessment object. It will be used by SQLAlchemy's ORM to map the object to
    the clinical_wound_assessment table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'clinical_wound_assessment'
    id = Column('id',Integer, primary_key=True)
    patient_id = Column('patient_id',Integer)
    wound_id = Column('wound_id',Integer)
    assessment_id = Column('assessment_id',Integer)
    assessment_date = Column('assessment_date',DateTime)
    length = Column('length',Numeric)
    width = Column('width',Numeric)
    depth = Column('depth',Numeric)
    undermining_depth = Column('undermining_depth',Numeric)
    undermining_description = Column('undermining_description',String)
    ulcer_stage = Column('ulcer_stage',Numeric)
    bed_color = Column('bed_color',String)
    exudate_amount = Column('exudate_amount',String)
    exudate_type = Column('exudate_type',String)
    granulation_percentage = Column('granulation_percentage',Numeric)
    slough_percentage = Column('slough_percentage',Numeric)
    eschar_percentage = Column('eschar_percentage',Numeric)
    bone_percentage = Column('bone_percentage',Numeric)
    peri_ulcer_area_description = Column('peri_ulcer_area_description',String)
    blanching_exists = Column('blanching_exists',Boolean)
    infection_process = Column('infection_process',String)
    odor_intensity = Column('odor_intensity',String)
    odor_description = Column('odor_description',String)
    keyCol = 'id'
    editCols = ['patient_id','wound_id','assessment_id','assessment_date','length', 'width', 'depth', 'undermining_depth', 'undermining_description', 'ulcer_stage', 'bed_color', 'exudate_amount', 'exudate_type', 'granulation_percentage', 'slough_percentage', 'eschar_percentage', 'bone_percentage', 'peri_ulcer_area_description','blanching_exists', 'infection_process', 'odor_intensity', 'odor_description']
    editColsLabels = ['Patient Id','Wound Id','Assessment Id','Assessment Time','Length', 'Width', 'Depth', 'Undermining Depth', 'Undermining Description', 'Ulcer Stage', 'Bed Color', 'Exudate Amount' , 'Exudate Type', 'Granulation Percentage', 'Slough Percentage', 'Eschar Percentage', 'Bone Percentage', 'Peri Ulcer Area Description', 'Blanching Exists', 'Infection Process', 'Odor Intensity', 'Odor Description']
    editColsTypes = ['numeric','numeric','numeric','date','numeric','numeric','numeric','numeric','string','numeric','string','string','string','numeric','numeric','numeric','numeric','string','string','string','string','string']
    displayTableName = 'Clinical Wound Assessment'
    
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
        self.assessment_date = data['assessment_date']
        self.length = data['length']
        self.width = data['width']
        self.depth = data['depth']
        self.undermining_depth = data['undermining_depth']
        self.undermining_description = data['undermining_description']
        self.ulcer_stage = data['ulcer_stage']
        self.bed_color = data['bed_color']
        self.exudate_amount = data['exudate_amount']
        self.exudate_type = data['exudate_type']
        self.granulation_percentage = data['granulation_percentage']
        self.slough_percentage = data['slough_percentage']
        self.eschar_percentage = data['eschar_percentage']
        self.bone_percentage = data['bone_percentage']
        self.peri_ulcer_area_description = data['peri_ulcer_area_description']
        self.blanching_exists = data['blanching_exists']
        self.infection_process = data['infection_process']
        self.odor_intensity = data['odor_intensity']
        self.odor_description = data['odor_description']
        
    def __json__(self, request):
        return {'id':self.id, 'patient_id':self.patient_id, 'wound_id':self.wound_id, 'assessment_id':self.assessment_id, 'assessment_date':self.assessment_date.isoformat(' '),
                'length':self.length, 'width':self.width, 'depth':self.depth, 
                'undermining_depth':self.undermining_depth, 'undermining_description':self.undermining_description,
                'ulcer_stage':self.ulcer_stage, 'bed_color':self.bed_color, 
                'exudate_amount':self.exudate_amount, 'exudate_type':self.exudate_type,
                'granulation_percentage':self.granulation_percentage, 'slough_percentage':self.slough_percentage, 'eschar_percentage':self.eschar_percentage, 'bone_percentage':self.bone_percentage,
                'peri_ulcer_area_description':self.peri_ulcer_area_description, 'blanching_exists':self.blanching_exists, 'infection_process':self.infection_process,
                 'odor_intensity':self.odor_intensity, 'odor_description':self.odor_description }

    def __repr__(self):
        return "<ClinicalWoundAssessment(id='%d', patient_id='%d', wound_id='%d', assessment_id='%d', assessment_date='%s', length='%d', width='%d', depth='%d', undermining depth='%d', undermining desc.='%s', stage='%s', bed color='%s', exudate amount='%s', exudate type='%s', granulation='%d', slough='%d', eschar='%d', bone='%d', peri. ulcer='%s', blanching?='%s', infection='%s', odor_intensity='%s', odor_description='%s')>" % (
                self.id, self.patient_id, self.wound_id, self.assessment_id, self.assessment_date,
                self.length, self.width, self.depth, self.undermining_depth, self.undermining_description,
                self.ulcer_stage, self.bed_color, self.exudate_amount, self.exudate_type,
                self.granulation_percentage, self.slough_percentage, self.eschar_percentage, self.bone_percentage,
                self.peri_ulcer_area_description, self.blanching_exists, self.infection_process,
                self.odor_intensity, self.odor_description)
