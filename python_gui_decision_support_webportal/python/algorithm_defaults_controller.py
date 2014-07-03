from  olv_dialog_controller import OlvDialogController
from algorithm_defaults import AlgorithmDefaults
from sqlalchemy import and_

class AlgorithmDefaultsController(OlvDialogController):
    """
    Controller class for Wound Assessment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllForOLView(algorithmId) - Gets all records for an algorithm
        getAll(algorithmId) - Gets all records for an algorithm as AlgorithmDefaults
    """
    def getAllForOLView(self, algorithmId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if algorithmId < 0:
            result = self.db.session.query(AlgorithmDefaults).all()
        else:
            result = self.db.session.query(AlgorithmDefaults).filter(AlgorithmDefaults.algorithm_id == algorithmId).all()
        olvResults = self.convertResults(result)
        return olvResults

    def getAll(self, algorithmId):
        """
        Gets all records 
        """
        result = None
        if algorithmId < 0:
            result = self.db.session.query(AlgorithmDefaults).all()
        else:
            result = self.db.session.query(AlgorithmDefaults).filter(AlgorithmDefaults.algorithm_id == algorithmId).all()
        return result

    def getAllAsLists(self, algorithmId):
        """
        Gets all records and return as lists
        """
        names = []
        values = []
        if algorithmId < 0:
            result = self.db.session.query(AlgorithmDefaults).all()
        else:
            result = self.db.session.query(AlgorithmDefaults).filter(AlgorithmDefaults.algorithm_id == algorithmId).all()
        for r in result:
            names.append(str(r.parameter_name))
            values.append(str(r.default_value))
        return (names,values)

    def getDefaultByName(self, pname, algorithmId):
        """
        Gets a default value given its name
        """
        result = self.db.session.query(AlgorithmDefaults.default_value).filter(and_(AlgorithmDefaults.algorithm_id == algorithmId,AlgorithmDefaults.parameter_name == pname)).scalar()
        return result

