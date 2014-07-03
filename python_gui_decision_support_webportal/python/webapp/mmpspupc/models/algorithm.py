from sqlalchemy import Column, Integer, String
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class Algorithm(Base):
    """
    Definition of Algorithm object. It will be used by SQLAlchemy's ORM to map the object to
    the algorithm table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'algorithm'
    id = Column('id',Integer, primary_key=True)
    algorithm_name = Column('algorithm_name',String)
    keyCol = 'id'
    editCols = ['algorithm_name']
    editColsLabels = ['Algorithm Name']
    editColsTypes = ['string']
    displayTableName = 'Algorithm'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.algorithm_name = data['algorithm_name']
        
    
    def __json__(self, request):
        return {'id':self.id, 'algorithm_name':self.algorithm_name }

    def __repr__(self):
        return "<Algorithm(id='%d', algorithm_name='%s')>" % (self.id, self.algorithm_name)

