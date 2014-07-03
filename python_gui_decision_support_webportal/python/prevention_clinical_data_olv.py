from ObjectListView import ColumnDefn

class OlvPreventionClinicalData(object):
    """
    Definition of PreventionClinicalData object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the PreventionClinicalData object
        getKey() - Gets the value of the key field
    """
    def __init__(self, pc):
        """
        Constructor which initializes the ObjectListView object from the PatientIdentification object.
        Arguments:
            pc - PreventionClinicalData object
        """
        # print "entering OlvPreventionClinicalData.__init__"
        self.id = pc.id
        self.patient_id = pc.patient_id
        self.clinical_rounding_time = pc.clinical_rounding_time
        self.repositioning_flag = pc.repositioning_flag
        self.final_position = pc.final_position
        self.repositioning_description = pc.repositioning_description
        # print "exiting OlvPreventionClinicalData.__init__"
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvPreventionClinicalDataCols(object):
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
                ColumnDefn("Rounding Time","left",200,"clinical_rounding_time"),
                ColumnDefn("Repositioned?","left",100,"repositioning_flag"),
                ColumnDefn("Final Position","left",200,"final_position"),
                ColumnDefn("Notes","left",400,"repositioning_description")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 200 + 100 + 200 + 400 + 20   # sum of column widths + spacer
        return width