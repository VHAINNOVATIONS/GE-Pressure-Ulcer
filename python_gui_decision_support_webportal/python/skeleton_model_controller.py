from  olv_dialog_controller import OlvDialogController
from skeleton_model import SkeletonModel

class SkeletonModelController(OlvDialogController):
    """
    Controller class for Skeleton Model Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAll() - Gets all records for a patient
    """
    def getAll(self):
        """
        Gets all records and return them
        """
        result = self.db.session.query(self.obj).all()
        return result
