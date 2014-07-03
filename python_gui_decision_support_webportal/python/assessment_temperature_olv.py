from ObjectListView import ColumnDefn

class OlvAssessmentTemperature(object):
    """
    Definition of AssessmentTemperature object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the AssessmentTemperature object
        getKey() - Gets the value of the key field
    """
    def __init__(self, at):
        """
        Constructor which initializes the ObjectListView object from the AssessmentTemperature object.
        Methods:
            at - AssessmentTemperature object
        """
        self.id = at.id
        self.session_id = at.session_id
        self.experiment_id = at.experiment_id
        self.start_time = at.start_time
        self.max_temperature = at.max_temperature
        self.max_temperature_loc_x = at.max_temperature_loc_x
        self.max_temperature_loc_y = at.max_temperature_loc_y
        self.min_temperature = at.min_temperature
        self.min_temperature_loc_x = at.min_temperature_loc_x
        self.min_temperature_loc_y = at.min_temperature_loc_y
        self.baseline_temperature = at.baseline_temperature
        self.baseline_description = at.baseline_description
        self.temperature_variation_sigma = at.temperature_variation_sigma
        self.temperature_segment_1_percentage = at.temperature_segment_1_percentage
        self.temperature_segment_2_percentage = at.temperature_segment_2_percentage
        self.temperature_segment_3_percentage = at.temperature_segment_3_percentage
        self.temperature_segment_4_percentage = at.temperature_segment_4_percentage
        self.temperature_segment_5_percentage = at.temperature_segment_5_percentage

    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAssessmentTemperatureCols(object):
    """
    Definition of AssessmentTemperature columns used by ObjectListView
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
                ColumnDefn("Max. (C)","right",60,"max_temperature", stringConverter="%.1f"),
                ColumnDefn("Min. (C)","right",60,"min_temperature", stringConverter="%.1f"),
                ColumnDefn("Baseline (C)","right",80,"baseline_temperature", stringConverter="%.1f"),
                ColumnDefn("Sigma","right",60,"temperature_variation_sigma", stringConverter="%.3f"),
                ColumnDefn("Segment #1 (%)","right",100,"temperature_segment_1_percentage", stringConverter="%.1f"),
                ColumnDefn("Segment #2 (%)","right",100,"temperature_segment_2_percentage", stringConverter="%.1f"),
                ColumnDefn("Segment #3 (%)","right",100,"temperature_segment_3_percentage", stringConverter="%.1f"),
                ColumnDefn("Segment #4 (%)","right",100,"temperature_segment_4_percentage", stringConverter="%.1f"),
                ColumnDefn("Segment #5 (%)","right",100,"temperature_segment_5_percentage", stringConverter="%.1f")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 130 + 60 + 60 + 80 + 60 + 100 + 100 + 100 + 100 + 100 + 100   # sum of column widths + spacer
        return width