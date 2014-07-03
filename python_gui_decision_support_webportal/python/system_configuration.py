from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class SystemConfiguration(Base):
    """
    Definition of SystemConfiguration object. It will be used by SQLAlchemy's ORM to map the object to
    the system_configuration table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'system_configuration'
    id = Column('id',Integer, primary_key=True)
    parameter_name = Column('parameter_name',String)
    parameter_value = Column('parameter_value',String)
    keyCol = 'id'
    editCols = ['parameter_name', 'parameter_value']
    editColsLabels = ['Parameter Name', 'Parameter Value']
    editColsTypes = ['string', 'string']
    displayTableName = 'System Configuration'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.parameter_name = data['parameter_name']
        self.parameter_value = data['parameter_value']
        
    
    def __json__(self, request):
        return {'id':self.id, 'parameter_name':self.parameter_name, 'parameter_value':self.parameter_value }

    def __repr__(self):
        return "<SystemConfiguration(id='%d', parameter_name='%s', parameter_value='%s')>" % (self.id, self.parameter_name, self.parameter_value)

