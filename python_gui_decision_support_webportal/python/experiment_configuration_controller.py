from  olv_dialog_controller import OlvDialogController
from experiment_configuration import ExperimentConfiguration

class ExperimentConfigurationController(OlvDialogController):
    """
    Controller class for Experiment Configuration Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAllForOLView(experimentId) - Gets all records for an experiment
        getAll(experimentId) - Gets all records for an experiment
    """
    def getAllForOLView(self, experimentId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if experimentId < 0:
            result = self.db.session.query(ExperimentConfiguration).all()
        else:
            result = self.db.session.query(ExperimentConfiguration).filter(ExperimentConfiguration.experiment_id == experimentId).all()
        olvResults = self.convertResults(result)
        return olvResults

    def getAll(self, experimentId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if experimentId < 0:
            result = self.db.session.query(ExperimentConfiguration).all()
        else:
            result = self.db.session.query(ExperimentConfiguration).filter(ExperimentConfiguration.experiment_id == experimentId).all()
        return result
