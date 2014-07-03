from  olv_dialog_controller import OlvDialogController
from system_configuration import SystemConfiguration

class SystemConfigurationController(OlvDialogController):
    """
    Controller class for Experiment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAll() - Gets all records
    """
    def getAll(self):
        """
        Gets all records and return them
        """
        result = self.db.session.query(self.obj).all()
        return result
