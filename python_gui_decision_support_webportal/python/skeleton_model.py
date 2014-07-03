from sqlalchemy import Column, Integer, String, DateTime, Numeric
from sqlalchemy.ext.declarative import declarative_base

Base = declarative_base()

class SkeletonModel(Base):
    """
    Definition of SkeletonModel object. It will be used by SQLAlchemy's ORM to map the object to
    the skeleton_model table.
    Methods:
        setFromData(data) - sets all data fields
    """
    __tablename__ = 'skeleton_model'
    location_id = Column('location_id',Integer, primary_key=True)
    body_part_code = Column('body_part_code',String)
    description = Column('description',String)
    keyCol = 'location_id'
    editCols = ['body_part_code','description']
    editColsLabels = ['Body Part Code','Description']
    editColsTypes = ['string','string']
    displayTableName = 'Skeleton Model'
    
    def setFromData(self,data):
        """
        Sets all of the object fields
        Arguments:
            data - Dictionary containing the data
        """
        # self.id = data['id']
        self.location_id = data['location_id']
        self.body_part_code = data['body_part_code']
        self.description = data['description']
    
    def __json__(self, request):
        return {'location_id':self.location_id, 'body_part_code':self.body_part_code, 'description':self.description }

    def __repr__(self):
        return "<SkeletonModel(location_id='%d', body_part_code='%s', description='%s')>" % (self.location_id, self.body_part_code, self.description)
