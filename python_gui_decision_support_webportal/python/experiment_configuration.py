from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class ExperimentConfiguration(Base):
    """
    Definition of ExperimentConfiguration object. It will be used by SQLAlchemy's ORM to map the object to
    the experiment_configuration table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'experiment_configuration'
    id = Column('id',Integer, primary_key=True)
    experiment_id = Column('experiment_id',Integer)
    parameter_name = Column('parameter_name',String)
    parameter_value = Column('parameter_value',String)
    keyCol = 'id'
    editCols = ['parameter_name', 'parameter_value']
    editColsLabels = ['Parameter Name', 'Parameter Value']
    editColsTypes = ['string', 'string']
    displayTableName = 'Experiment Configuration'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.experiment_id = data['experiment_id']
        self.parameter_name = data['parameter_name']
        self.parameter_value = data['parameter_value']
        
    
    def __json__(self, request):
        return {'id':self.id, 'experiment_id':self.experiment_id, 'parameter_name':self.parameter_name, 'parameter_value':self.parameter_value }

    def __repr__(self):
        return "<Algorithm(id='%d', experiment_id='%s', parameter_name='%s', parameter_value='%s')>" % (self.id, self.experiment_id, self.parameter_name, self.parameter_value)

