from ObjectListView import ColumnDefn

class OlvWoundAssessment(object):
    """
    Definition of WoundAssessment object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the WoundAssessment object
        getKey() - Gets the value of the key field
    """
    def __init__(self, wa):
        """
        Constructor which initializes the ObjectListView object from the PatientIdentification object.
        Arguments:
            wa - WoundAssessment object
        """
        print "entering OlvWoundAssessment.__init__"
        self.id = wa.id
        self.patient_name = wa.patient_name
        self.body_part_code = wa.body_part_code
        self.description = wa.description
        self.wound_location_description = wa.wound_location_description
        print "exiting OlvWoundAssessment.__init__"
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvWoundAssessmentCols(object):
    """
    Definition of patient_identification columns used by ObjectListView
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
                ColumnDefn("Patient Name","left",200,"patient_name"),
                ColumnDefn("Wound Body Part Code","right",150,"body_part_code"),
                ColumnDefn("Wound Location","left",200,"description"),
                ColumnDefn("Wound Location Description","left",400,"wound_location_description")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 200 + 150 + 200 + 400 + 20   # sum of column widths + spacer
        return width