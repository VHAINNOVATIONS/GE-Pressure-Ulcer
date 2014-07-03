from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class PatientIdentification(Base):
	"""
	Definition of PatientIdentification object. It will be used by SQLAlchemy's ORM to map the object to
	the patient_identification table.
	Methods:
		setFromData(data) - sets all data fields
	"""
	__tablename__ = 'patient_identification'
	patient_id = Column('patient_id',Integer, primary_key=True)
	va_patient_id = Column('va_patient_id',String)
	patient_name = Column('patient_name',String)
	data_files_base_directory = Column('data_files_base_directory',String)
	camera_id = Column('camera_id',Integer)
	keyCol = 'patient_id'
	editCols = ['va_patient_id','patient_name','data_files_base_directory','camera_id']
	editColsLabels = ['VA Patient Id.','Patient Name','Data Files Base Directory','Camera Id.']
	editColsTypes = ['string','string','string','integer']
	displayTableName = 'Patient Identification'
	
	def setFromData(self,data):
		"""
		Sets all of the object fields
		Arguments:
			data - Dictionary containing the data
		"""
		# self.patient_id = data['patient_id']
		self.va_patient_id = data['va_patient_id']
		self.patient_name = data['patient_name']
		self.data_files_base_directory = data['data_files_base_directory']
		self.camera_id = data['camera_id']
		
	
	def __json__(self, request):
		return {'patient_id':self.patient_id, 'va_patient_id':self.va_patient_id, 'patient_name':self.patient_name, 'data_files_base_directory':self.data_files_base_directory, 'camera_id':self.camera_id }

	def __repr__(self):
		return "<PatientIdentification(patient_id='%d', va_patient_id='%s', patient_name='%s', data_files_base_directory='%s', camera_id='%d')>" % (
				self.patient_id, self.va_patient_id, self.patient_name, self.data_files_base_directory, self.camera_id)

