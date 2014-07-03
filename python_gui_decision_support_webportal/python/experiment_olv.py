from ObjectListView import ColumnDefn

class OlvExperiment(object):
    """
    Definition of Experiment object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the Experiment object
        getKey() - Gets the value of the key field
    """
    def __init__(self, e):
        """
        Constructor which initializes the ObjectListView object from the Experiment object.
        Methods:
            e - Experiment object
        """
        print "OlvExperiment constructor: "
        print e
        self.id = e.id
        self.experiment_name = e.experiment_name
        self.algorithm_name = e.algorithm_name
        self.default_flag = e.default_flag
        self.experiment_description = e.experiment_description
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvExperimentCols(object):
    """
    Definition of algorithm columns used by ObjectListView
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
                ColumnDefn("Internal Id","right",150,"id"),
                ColumnDefn("Experiment","left",300,"experiment_name"),
                ColumnDefn("Algorithm","left",300,"algorithm_name"),
                ColumnDefn("Default","left",80,"default_flag"),
                ColumnDefn("Description","left",400,"experiment_description"),
                ]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 150 + 300 + 300 + 80 + 400 + 20   # sum of column widths + spacer
        return width