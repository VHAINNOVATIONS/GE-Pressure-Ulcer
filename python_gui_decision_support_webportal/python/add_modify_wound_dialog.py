import util
import wx
from patient_identification import PatientIdentification
from patient_identification_controller import PatientIdentificationController
from skeleton_model import SkeletonModel
from skeleton_model_controller import SkeletonModelController


########################################################################
class AddModifyWoundDialog(wx.Dialog):
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

    def __init__(self, controller, obj, db, row=None, title="Add", addRecord=True, patientId=None):
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
        patientController = PatientIdentificationController(db, PatientIdentification, None, None)
        skeletonController = SkeletonModelController(db, SkeletonModel, None, None)
        
        wx.Dialog.__init__(self, None, title="%s Wound Record" % title)
        self.controller = controller
        if row:
            key = row.getKey()
            self.objInstance = controller.getRecordByKey(key)
            if addRecord:
                curPatientId = self.objInstance.patient_id
                self.objInstance = obj()
                self.objInstance.patient_id = curPatientId
        else:
            self.objInstance = obj()
            self.objInstance.patient_id = patientId
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
        lbl.SetLabel("Patient:")
        lbl.SetFont(font)
        if row:
            patientTextCtrl = wx.TextCtrl(self, value=row.patient_name)
        else:
            patient = patientController.getRecordByKey(patientId)
            patientName = patient.patient_name
            print "add_modify_wound_dialog.... patient: "+patientName
            patientTextCtrl = wx.TextCtrl(self, value=patientName)
        patientTextCtrl.Enable(False)
        mainSizer.Add(self.rowBuilder([lbl, patientTextCtrl]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Wound Id:")
        lbl.SetFont(font)
        if row and not(self.addRecord):
            self.woundIdTextCtrl = wx.TextCtrl(self, value=str(self.objInstance.wound_id))
        else:
            woundId = controller.getMaxWoundId(self.objInstance.patient_id)
            if woundId != None:
                woundId = woundId + 1
            else:
                woundId = 1
            self.woundIdTextCtrl = wx.TextCtrl(self, value=str(woundId))
        self.woundIdTextCtrl.Enable(False)
        mainSizer.Add(self.rowBuilder([lbl, self.woundIdTextCtrl]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Location (ICD_10):")
        lbl.SetFont(font)
        self.wound_loc = wx.ComboBox(self, wx.ID_ANY, choices=[], style=wx.CB_DROPDOWN | wx.CB_DROPDOWN)
        self.wound_loc.SetFont(font)
        bodyParts = skeletonController.getAll()
        for s in bodyParts:
            self.wound_loc.Append(s.description, str(s.location_id))
        if row:
            n = 0
            for s in bodyParts:
                if self.objInstance.wound_location_id == s.location_id:
                    self.wound_loc.Select(n)
                    break
                n = n + 1
        else:
            self.wound_loc.Select(0)
        mainSizer.Add(self.rowBuilder([lbl, self.wound_loc]), 0, wx.EXPAND)
        
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Location Description:")
        lbl.SetFont(font)
        if row and not(self.addRecord):
            self.descTextCtrl = wx.TextCtrl(self, value=self.objInstance.wound_location_description)
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
        self.SetMinSize(wx.Size(400,20))
        

    def getData(self):
        """
        Gets the data from the text controls.
        """
        data = {}
        data['patient_id'] = self.objInstance.patient_id
        data['wound_id'] = self.woundIdTextCtrl.GetValue()
        data['wound_location_id'] = self.wound_loc.GetClientData(self.wound_loc.GetSelection())
        data['wound_location_description'] = self.descTextCtrl.GetValue()
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
        # Perform validation checks
        if self.descTextCtrl.GetValue() == "":
            util.showMessageDialog("Wound description cannot be empty!", "Failure!", wx.ICON_ERROR)
            return
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
        sizer.Add(txt, 1, wx.EXPAND|wx.ALL, 5)
        return sizer
            
