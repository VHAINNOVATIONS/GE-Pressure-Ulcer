from ObjectListView import ColumnDefn

class OlvSystemConfiguration(object):
    """
    Definition of SystemConfiguration object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the SystemConfiguration object
        getKey() - Gets the value of the key field
    """
    def __init__(self, sc):
        """
        Constructor which initializes the ObjectListView object from the SystemConfiguration object.
        Arguments:
            sc - SystemConfiguration object
        """
        self.id = sc.id
        self.parameter_name = sc.parameter_name
        self.parameter_value = sc.parameter_value
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvSystemConfigurationCols(object):
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
                ColumnDefn("Parameter Value","left",400,"parameter_value")]
        
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