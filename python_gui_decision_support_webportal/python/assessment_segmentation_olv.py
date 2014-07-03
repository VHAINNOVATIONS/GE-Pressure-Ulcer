from ObjectListView import ColumnDefn

class OlvAssessmentSegmentation(object):
    """
    Definition of AssessmentSegmentation object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the AssessmentSegmentation object
        getKey() - Gets the value of the key field
    """
    def __init__(self, ag):
        """
        Constructor which initializes the ObjectListView object from the AssessmentSegmentation object.
        Methods:
            as - AssessmentSegmentation object
        """
        self.id = ag.id
        self.session_id = ag.session_id
        self.experiment_id = ag.experiment_id
        self.start_time = ag.start_time
        self.image_label_map_file = ag.image_label_map_file
        self.granulation_percentage = ag.granulation_percentage
        self.slough_percentage = ag.slough_percentage
        self.eschar_percentage = ag.eschar_percentage
        self.bone_percentage = ag.bone_percentage
        self.ulcer_stage = ag.ulcer_stage

    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAssessmentSegmentationCols(object):
    """
    Definition of AssessmentSegmentation columns used by ObjectListView
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
                ColumnDefn("Granulation (%)","right",100,"granulation_percentage", stringConverter="%.1f"),
                ColumnDefn("Slough (%)","right",80,"slough_percentage", stringConverter="%.1f"),
                ColumnDefn("Eschar (%)","right",80,"eschar_percentage", stringConverter="%.1f"),
                ColumnDefn("Bone/Tendon (%)","right",100,"bone_percentage", stringConverter="%.1f"),
                ColumnDefn("Ulcer Stage","right",90,"ulcer_stage", stringConverter="%.0f")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 130 + 100 + 80 + 80 + 100 + 90 + 100   # sum of column widths + spacer
        return width