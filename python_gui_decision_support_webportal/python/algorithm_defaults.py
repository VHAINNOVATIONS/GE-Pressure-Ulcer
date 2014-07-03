from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class AlgorithmDefaults(Base):
    """
    Definition of AlgorithmDefaults object. It will be used by SQLAlchemy's ORM to map the object to
    the algorithm_defaults table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'algorithm_defaults'
    id = Column('id',Integer, primary_key=True)
    algorithm_id = Column('algorithm_id',Integer)
    parameter_name = Column('parameter_name',String)
    default_value = Column('default_value',String)
    keyCol = 'id'
    editCols = ['parameter_name', 'default_value']
    editColsLabels = ['Parameter Name', 'Default Value']
    editColsTypes = ['string', 'string']
    displayTableName = 'Algorithm Defaults'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.algorithm_id = data['algorithm_id']
        self.parameter_name = data['parameter_name']
        self.default_value = data['default_value']
        
    
    def __json__(self, request):
        return {'id':self.id, 'algorithm_id':self.algorithm_id, 'parameter_name':self.parameter_name, 'default_value':self.default_value }

    def __repr__(self):
        return "<Algorithm(id='%d', algorithm_id='%s', parameter_name='%s', default_value='%s')>" % (self.id, self.algorithm_id, self.parameter_name, self.default_value)

