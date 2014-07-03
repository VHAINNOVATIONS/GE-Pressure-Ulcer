from  olv_dialog_controller import OlvDialogController
from experiment import Experiment
from algorithm import Algorithm
from sqlalchemy import and_

class ExperimentController(OlvDialogController):
    """
    Controller class for Experiment Dialogs which performs various queries,
    adds, updates, and deletes records from a database table. 
    The class extends the OlvDialogController class. 
    Methods:
        getAll() - Gets all records
        getAllByAlgorithm(algorithmId) - Gets all experiments for an algorithm
        getAllForOLView(algorithmId) - Gets all experiments for an algorithm for OLV display
        getDefaultExperiment(algorithmName) - Gets the default experiment for an algorithm by name
    """
    def getAll(self):
        """
        Gets all records and return them
        """
        result = self.db.session.query(self.obj).all()
        return result
    
    def getAllByAlgorithm(self, algorithmId):
        """
        Gets all records by an algorithm
        """
        wounds = self.db.session.query(Experiment).filter(Experiment.algorithm_id == algorithmId).order_by(Experiment.experiment_name).all()
        return wounds

    def getAllForOLView(self, algorithmId):
        """
        Gets records for display in an Object List View
        """
        result = None
        if algorithmId < 0:
            result = self.db.session.query(Experiment.id,Experiment.experiment_name,Experiment.algorithm_id,Experiment.default_flag,Experiment.experiment_description,Algorithm.algorithm_name).join(Algorithm, Experiment.algorithm_id == Algorithm.id).all()
        else:
            result = self.db.session.query(Experiment.id,Experiment.experiment_name,Experiment.algorithm_id,Experiment.default_flag,Experiment.experiment_description,Algorithm.algorithm_name).join(Algorithm, Experiment.algorithm_id == Algorithm.id).filter(Experiment.algorithm_id == algorithmId).all()
        olvResults = self.convertResults(result)
        print olvResults
        return olvResults

    def getDefaultExperiment(self, algorithmName):
        result = self.db.session.query(Experiment).join(Algorithm, Experiment.algorithm_id == Algorithm.id).filter(and_(Algorithm.algorithm_name == algorithmName, Experiment.default_flag == True)).first()
        return result
