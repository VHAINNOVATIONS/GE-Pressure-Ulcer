from ObjectListView import ColumnDefn

class OlvAssessmentMeasurements(object):
    """
    Definition of AssessmentMeasurements object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the AssessmentMeasurements object
        getKey() - Gets the value of the key field
    """
    def __init__(self, am):
        """
        Constructor which initializes the ObjectListView object from the AssessmentMeasurements object.
        Methods:
            am - AssessmentMeasurements object
        """
        self.id = am.id
        self.session_id = am.session_id
        self.experiment_id = am.experiment_id
        self.start_time = am.start_time
        self.length = am.length
        self.width = am.width
        self.depth = am.depth
        self.length_x_width = am.length_x_width
        self.surface_area = am.surface_area
        self.wound_volume = am.wound_volume
        self.push_score = am.push_score
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAssessmentMeasurementsCols(object):
    """
    Definition of AssessmentMeasurements columns used by ObjectListView
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
                ColumnDefn("Assessment Date/Time","left",130,"start_time"),
                ColumnDefn("Length (cm.)","right",80,"length"),
                ColumnDefn("Width (cm.)","right",80,"width"),
                ColumnDefn("Length*Width (cm.)","right",120,"length_x_width"),
                ColumnDefn("Area (cm**2)","right",80,"surface_area"),
                ColumnDefn("Volume (cm**3)","right",100,"wound_volume"),
                ColumnDefn("PUSH Score","right",80,"push_score")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 130 + 80 + 80 + 120 + 80 + 100 + 80 + 100   # sum of column widths + spacer
        return width