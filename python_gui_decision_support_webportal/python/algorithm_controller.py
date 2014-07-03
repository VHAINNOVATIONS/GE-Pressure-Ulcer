from  olv_dialog_controller import OlvDialogController
from algorithm import Algorithm

class AlgorithmController(OlvDialogController):
    """
    Controller class for Wound Assessment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAll - Gets all records
    """
    def getAll(self):
        """
        Gets all records and return them
        """
        result = self.db.session.query(self.obj).all()
        return result
    
    def getByName(self, algorithm_name):
        """
        Gets an algorithm by name
        """
        result = self.db.session.query(Algorithm).filter(Algorithm.algorithm_name == algorithm_name).first()
        return result