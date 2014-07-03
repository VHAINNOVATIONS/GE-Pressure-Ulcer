from ObjectListView import ColumnDefn

class OlvAssessmentSession(object):
    """
    Definition of AssessmentSession object used by ObjectListView.
    Methods:
        __init__(pi) - Constructor which initializes the ObjectListView object from the AssessmentSession object
        getKey() - Gets the value of the key field
    """
    def __init__(self, sas):
        """
        Constructor which initializes the ObjectListView object from the AssessmentSession object.
        Methods:
            as - AssessmentSession object
        """
        self.id = sas.id
        self.patient_id = sas.patient_id
        self.wound_id = sas.wound_id
        self.assessment_id = sas.assessment_id
        self.start_time = sas.start_time
        self.collection_status_visual = sas.collection_status_visual
        self.collection_status_multi_spectral = sas.collection_status_multi_spectral
        self.collection_status_chemical = sas.collection_status_chemical
        self.collection_status_chemical_bl = sas.collection_status_chemical_bl
        
    def getKey(self):
        """
        Gets the value of the key field.
        """
        return self.id
        
class OlvAssessmentSessionCols(object):
    """
    Definition of assessment_session columns used by ObjectListView
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
                ColumnDefn("Internal Patient Id","right",150,"patient_id"),
                ColumnDefn("Internal Wound Id","right",150,"wound_id"),
                ColumnDefn("Assessment Id","right",150,"assessment_id"),
                ColumnDefn("Start Date/Time","left",200,"start_time"),
                ColumnDefn("collection_status_visual","left",200,"collection_status_visual"),
                ColumnDefn("collection_status_multi_spectral","left",200,"collection_status_multi_spectral"),
                ColumnDefn("collection_status_chemical","left",200,"collection_status_chemical"),
                ColumnDefn("collection_status_chemical_bl","left",200,"collection_status_chemical_bl")]
        
    def getColumnDefinitions(self):
        """
        Constructor which initializes the ObjectListViewCols object.
        """
        return self.columns
    
    def getTotalColumnWidth(self):
        """
        Gets the total column width.
        """
        width = 150 + 150 + 150 + 150 + 200 + 200 + 200 + 200 + 200 + 20   # sum of column widths + spacer
        return width