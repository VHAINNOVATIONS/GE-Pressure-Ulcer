from ObjectListView import ColumnDefn

class OlvAlgorithmDefaults(object):
    """
    Definition of AlgorithmDefaults object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the AlgorithmDefaults object
        getKey() - Gets the value of the key field
    """
    def __init__(self, ad):
        """
        Constructor which initializes the ObjectListView object from the AlgorithmDefaults object.
        Arguments:
            ad - AlgorithmDefaults object
        """
        self.id = ad.id
        self.algorithm_id = ad.algorithm_id
        self.parameter_name = ad.parameter_name
        self.default_value = ad.default_value
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAlgorithmDefaultsCols(object):
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
                ColumnDefn("Default Value","left",400,"default_value")]
        
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