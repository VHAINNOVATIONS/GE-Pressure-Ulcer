from ObjectListView import ColumnDefn

class OlvPatientIdentification(object):
    """
    Definition of PatientIdentification object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the PatientIdentification object
        getKey() - Gets the value of the key field
    """
    def __init__(self, pi):
        """
        Constructor which initializes the ObjectListView object from the PatientIdentification object.
        Methods:
            pi - PatientIdentification object
        """
        self.patient_id = pi.patient_id
        self.va_patient_id = pi.va_patient_id
        self.patient_name = pi.patient_name
        self.camera_id = pi.camera_id
        print "exiting OlvPatientIdentification.__init__"
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.patient_id
        
class OlvPatientIdentificationCols(object):
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
                ColumnDefn("Internal Id","right",100,"patient_id"),
                ColumnDefn("VA Patient Id","left",100,"va_patient_id"),
                ColumnDefn("Name","left",200,"patient_name"),
                ColumnDefn("Camera Id","right",100,"camera_id")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 100 + 100 + 200 + 100 + 20   # sum of column widths + spacer
        return width