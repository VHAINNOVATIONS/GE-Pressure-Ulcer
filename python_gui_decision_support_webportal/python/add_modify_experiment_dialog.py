import util
import wx
from algorithm import Algorithm
from algorithm_controller import AlgorithmController

########################################################################
class AddModifyExperimentDialog(wx.Dialog):
    """
    This class implements a modal dialog to add and modify wound_assessment database records.
    The row to be processed is passed in as an object instance. In the spirit of MVC
    a controller class performs the actual addition or update of the object.
    Methods:
        __init__(controller, obj, row=None, title="Add", addRecord=True) - initializes the dialog
        getData - Gets the data from the text control
        onAdd - Adds the record to the database and shows a confirmation message dialog
        onClose - Cancels the dialog
        onEdit - Modifies the record and shows a confirmation message dialog
        onRecord - Handler for the OK button. It will either call onAdd or onEdit as required.
        rowBuilder - Creates a single row of the data entry form
    """

    def __init__(self, controller, obj, db, row=None, title="Add", addRecord=True, algorithmId=None):
        """
        Initializes the add/modify dialog. This consists of constructing an input form
        which has field names and a text field for entering field values.
        Arguments:
            controller - class performing the object creation, update, or deletion
            obj - object class to be processed
            row - row selected containing the object
            title - title to be displayed on form
            addRecord - flag indicating that a new record is to be created
        """
        # set up controllers
        algorithmController = AlgorithmController(db, Algorithm, None, None)
        
        wx.Dialog.__init__(self, None, title="%s Experiment Record" % title)
        self.controller = controller
        if row:
            key = row.getKey()
            self.objInstance = controller.getRecordByKey(key)
            if addRecord:
                curAlgorithmtId = self.objInstance.algorithm_id
                self.objInstance = obj()
                self.objInstance.patient_id = curAlgorithmtId
        else:
            self.objInstance = obj()
            self.objInstance.algorithm_id = algorithmId
        self.addRecord = addRecord
        self.selectedRow = row
        size = (80, -1)
        font = wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.BOLD) 
        
        # create the sizers
        mainSizer = wx.BoxSizer(wx.VERTICAL)
        btnSizer = wx.BoxSizer(wx.HORIZONTAL)
                
        # create some widgets
        lbl = wx.StaticText(self, label=self.objInstance.displayTableName)
        lbl.SetFont(font)
        mainSizer.Add(lbl, 0, wx.CENTER)
        font = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        
        self.ctls = []
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Experiment Name:")
        lbl.SetFont(font)
        if row:
            self.experimentTextCtrl = wx.TextCtrl(self, value=row.experiment_name)
        else:
            self.experimentTextCtrl = wx.TextCtrl(self, value="")
        mainSizer.Add(self.rowBuilder([lbl, self.experimentTextCtrl]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Algorithm:")
        lbl.SetFont(font)
        self.algorithm_name = wx.ComboBox(self, wx.ID_ANY, choices=[], style=wx.CB_DROPDOWN | wx.CB_DROPDOWN)
        self.algorithm_name.SetFont(font)
        algorithms = algorithmController.getAll()
        for s in algorithms:
            self.algorithm_name.Append(s.algorithm_name, str(s.id))
        if row:
            n = 0
            for s in algorithms:
                if self.objInstance.algorithm_id == s.id:
                    self.algorithm_name.Select(n)
                    break
                n = n + 1
        else:
            self.algorithm_name.Select(0)
        mainSizer.Add(self.rowBuilder([lbl, self.algorithm_name]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Default?:")
        lbl.SetFont(font)
        self.defCheckbox = wx.CheckBox(self)
        if row and not(self.addRecord) and self.objInstance.default_flag == 1:
            self.defCheckbox.SetValue(True)
        mainSizer.Add(self.rowBuilder([lbl, self.defCheckbox]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Description:")
        lbl.SetFont(font)
        if row and not(self.addRecord) and not(self.objInstance.experiment_description is None):
            self.descTextCtrl = wx.TextCtrl(self, value=self.objInstance.experiment_description)
        else:
            self.descTextCtrl = wx.TextCtrl(self, value="")
        mainSizer.Add(self.rowBuilder([lbl, self.descTextCtrl]), 0, wx.EXPAND)
        
        okBtn = wx.Button(self, label="%s" % title)
        okBtn.Bind(wx.EVT_BUTTON, self.onRecord)
        btnSizer.Add(okBtn, 0, wx.ALL, 5)
        cancelBtn = wx.Button(self, label="Close")
        cancelBtn.Bind(wx.EVT_BUTTON, self.onClose)
        btnSizer.Add(cancelBtn, 0, wx.ALL, 5)
        
        mainSizer.Add(btnSizer, 0, wx.CENTER)
        self.SetSizer(mainSizer)
        self.Refresh()

    def getData(self):
        """
        Gets the data from the text controls.
        """
        data = {}
        data['experiment_name'] = self.experimentTextCtrl.GetValue()
        data['algorithm_id'] = self.algorithm_name.GetClientData(self.algorithm_name.GetSelection())
        data['default_flag'] = self.defCheckbox.GetValue()
        data['experiment_description'] = self.descTextCtrl.GetValue()
        return data
    

    def onAdd(self):
        """
        Adds the record to the database and shows a confirmation message dialog.
        """
        data = self.getData()
        (rc, msg) = self.controller.addRecord(data)
        
        # show dialog upon completion
        if rc == 0:
            util.showMessageDialog("Record Added", "Success!", wx.ICON_INFORMATION)
        else:
            util.showMessageDialog(msg, "Failure!", wx.ICON_INFORMATION)
        return rc
    
        
    def onClose(self, event):
        """
        Cancels the dialog.
        """
        self.Destroy()
        

    def onEdit(self):
        """
        Modifies the record and shows a confirmation message dialog.
        """
        data = self.getData()
        (rc, msg) = self.controller.editRecord(self.selectedRow.getKey(), data)
        # Check return code from above and put up appropriate message dialog
        if rc == 0:
            util.showMessageDialog("Record Edited Successfully!", "Success!", wx.ICON_INFORMATION)
        else:
            util.showMessageDialog(msg, "Failure!", wx.ICON_INFORMATION)
        return rc
        

    def onRecord(self, event):
        """
        Handler for the OK button. It will either call onAdd or onEdit as required.
        """
        rc = 0
        if self.addRecord:
            rc = self.onAdd()
        else:
            rc = self.onEdit()
        self.SetReturnCode(rc)
        if rc == 0:
            self.Destroy()
        

    def rowBuilder(self, widgets):
        """
        Creates a single row of the data entry form.
        """
        sizer = wx.BoxSizer(wx.HORIZONTAL)
        lbl, txt = widgets
        sizer.Add(lbl, 1, wx.ALL, 5)
        sizer.Add(txt, 2, wx.EXPAND|wx.ALL, 5)
        return sizer
            
