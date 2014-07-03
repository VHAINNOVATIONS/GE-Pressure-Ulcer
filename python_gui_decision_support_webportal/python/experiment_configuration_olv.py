from ObjectListView import ColumnDefn

class OlvExperimentConfiguration(object):
    """
    Definition of ExperimentConfiguration object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the ExperimentConfiguration object
        getKey() - Gets the value of the key field
    """
    def __init__(self, ec):
        """
        Constructor which initializes the ObjectListView object from the ExperimentConfiguration object.
        Arguments:
            ec - ExperimentConfiguration object
        """
        self.id = ec.id
        self.experiment_id = ec.experiment_id
        self.parameter_name = ec.parameter_name
        self.parameter_value = ec.parameter_value
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvExperimentConfigurationCols(object):
    """
    Definition of algorithm_defaults columns used by ObjectListView
    Methods:
        __init__() - Constructor which initializes the ObjectListViewCols object
        getColumnDefinitions() - Gets the column definitions
        getTotalColumnWidth() - Gets the total column width
    """
    def __init__(self):
        """
        Constructor which initializes the ObjectListViewCols object
        """
        self.columns = [
                ColumnDefn("Parameter Name","left",300,"parameter_name"),
                ColumnDefn("Default Value","left",400,"parameter_value")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 300 + 400 + 20   # sum of column widths + spacer
        return width