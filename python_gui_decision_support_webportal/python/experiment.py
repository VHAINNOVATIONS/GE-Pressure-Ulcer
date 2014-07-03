from sqlalchemy import Column, Integer, String, Boolean
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class Experiment(Base):
    """
    Definition of Experiment object. It will be used by SQLAlchemy's ORM to map the object to
    the experiment table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'experiment'
    id = Column('id',Integer, primary_key=True)
    experiment_name = Column('experiment_name',String)
    algorithm_id = Column('algorithm_id',Integer)
    default_flag = Column('default_flag',Boolean)
    experiment_description = Column('experiment_description',String)
    keyCol = 'id'
    editCols = ['experiment_name', 'default_flag', 'experiment_description']
    editColsLabels = ['Experiment Name', 'Default Flag', 'Description']
    editColsTypes = ['string','boolean','string']
    displayTableName = 'Experiment'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.experiment_name = data['experiment_name']
        self.algorithm_id = data['algorithm_id']
        self.default_flag = data['default_flag']
        self.experiment_description = data['experiment_description']
        
    
    def __json__(self, request):
        return {'id':self.id, 'experiment_name':self.experiment_name, 'algorithm_id':self.algorithm_id, 'default_flag':self.default_flag, 'experiment_description':self.experiment_description }

    def __repr__(self):
        return "<Experiment(id='%d', experiment_name='%s', algorithm_id='%s', default_flag='%s', experiment_description='%s')>" % (
                    self.id, self.experiment_name, self.algorithm_id, self.default_flag, self.experiment_description)

