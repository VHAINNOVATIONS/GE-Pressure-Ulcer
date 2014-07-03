import gevxlpy.util.configuration as configuration
from algorithm_defaults import AlgorithmDefaults
from algorithm_defaults_controller import AlgorithmDefaultsController
from experiment import Experiment
from experiment_controller import ExperimentController
from experiment_configuration import ExperimentConfiguration
from experiment_configuration_controller import ExperimentConfigurationController

class ExperimentConfig(configuration.cfg):
    """
    Definition of ExperimentConfig object. It will be loaded from both the algorithm_defaults and
    the experiment_configuration tables.
    Methods:
        setFromData(data) - sets all data fields
    """
    def SetFromDb(self, db, experimentId):
        try:
            experimentController = ExperimentController(db, Experiment, None, None)
            experiment = experimentController.getRecordByKey(experimentId)
            defaultsController = AlgorithmDefaultsController(db, AlgorithmDefaults, None, None)
            defaults = defaultsController.getAll(experiment.algorithm_id)
            for d in defaults:
                self[str(d.parameter_name)] = str(d.default_value)
            configController = ExperimentConfigurationController(db, ExperimentConfiguration, None, None)
            configs = configController.getAll(experimentId)
            for c in configs:
                self[str(c.parameter_name)] = str(c.parameter_value)
            config_file = self.as_config_file(silent=False)
            return config_file
        except Exception as e:
            print e
            # rc = e.errno
            # msg = e.strerror
            # print msg
            #return rc
            # return 4
