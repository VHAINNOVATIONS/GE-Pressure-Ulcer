from ObjectListView import ColumnDefn

class OlvPatientTurning(object):
    """
    Definition of PatientTurning object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the PatientTurning object
        getKey() - Gets the value of the key field
    """
    def __init__(self, pt):
        """
        Constructor which initializes the ObjectListView object from the PatientIdentification object.
        Arguments:
            pt - PatientTurning object
        """
        # print "entering OlvPatientTurning.__init__"
        self.id = pt.id
        self.patient_id = pt.patient_id
        self.session_id = pt.session_id
        self.experiment_id = pt.experiment_id
        self.turn_time = pt.turn_time
        self.final_position = pt.final_position
        self.provider_present_flag = pt.provider_present_flag
        # print "exiting OlvPatientTurning.__init__"
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvPatientTurningCols(object):
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
                ColumnDefn("Turn Time","left",200,"turn_time"),
                ColumnDefn("Final Position","left",150,"final_position"),
                ColumnDefn("Provider present?","left",100,"provider_present_flag")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 200 + 150 + 100 + 20   # sum of column widths + spacer
        return width