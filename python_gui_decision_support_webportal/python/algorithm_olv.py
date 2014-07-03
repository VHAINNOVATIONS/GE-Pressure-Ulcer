from ObjectListView import ColumnDefn

class OlvAlgorithm(object):
    """
    Definition of Algorithm object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the Algorithm object
        getKey() - Gets the value of the key field
    """
    def __init__(self, al):
        """
        Constructor which initializes the ObjectListView object from the Algorithm object.
        Methods:
            al - Algorithm object
        """
        self.id = al.id
        self.algorithm_name = al.algorithm_name
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAlgorithmCols(object):
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
                ColumnDefn("Algorithm Name","left",300,"algorithm_name")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 150 + 300 + 100   # sum of column widths + spacer
        return width