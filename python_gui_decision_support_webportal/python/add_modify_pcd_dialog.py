import util
import wx
import wx.lib.masked.timectrl
import wx.calendar
from datetime import datetime
from datetime import time
from patient_identification import PatientIdentification
from patient_identification_controller import PatientIdentificationController

########################################################################
class AddModifyPCDDialog(wx.Dialog):
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
        
        wx.Dialog.__init__(self, None, size=wx.Size(500,300), title="%s Record" % title)
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
        lblfont = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD)
        txtfont = wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL)
        
        self.ctls = []
        lbl = wx.StaticText(self, size=size)
        lbl.SetLabel("Patient:")
        lbl.SetFont(lblfont)
        if row:
            patient = patientController.getRecordByKey(row.patient_id)
        else:
            patient = patientController.getRecordByKey(patientId)
        patientName = patient.patient_name
        patientTextCtrl = wx.TextCtrl(self, value=patientName)
        patientTextCtrl.SetFont(txtfont)
        patientTextCtrl.Enable(False)
        mainSizer.Add(self.rowBuilder([lbl, patientTextCtrl]), 0, wx.EXPAND)
        
        lblrt = wx.StaticText(self, size=size)
        lblrt.SetLabel("Rounding Time:")
        lblrt.SetFont(lblfont)
        dtSizer = wx.BoxSizer(wx.HORIZONTAL)
        self.cc_datepicker = wx.DatePickerCtrl(self, wx.ID_ANY, style=wx.DP_DROPDOWN)
        self.cc_timepicker = wx.lib.masked.timectrl.TimeCtrl(self, wx.ID_ANY)
        self.cc_datepicker.SetFont(txtfont)
        self.cc_timepicker.SetFont(txtfont)
        self.cc_timepicker.SetMinSize((100, 25))
        dtSizer.Add(self.cc_datepicker, 0, 0, 0)
        dtSizer.Add(self.cc_timepicker, 1, 0, 0)
        if row and not(self.addRecord):
            wxrt = wx.calendar._pydate2wxdate(self.objInstance.clinical_rounding_time)
            wxrt.SetHMS(self.objInstance.clinical_rounding_time.hour,self.objInstance.clinical_rounding_time.minute,self.objInstance.clinical_rounding_time.second)
            self.cc_datepicker.SetValue(wxrt)
            self.cc_timepicker.SetValue(wxrt)
        else:
            now = datetime.now()
            wxnow = wx.calendar._pydate2wxdate(now)
            wxnow.SetHMS(now.hour,now.minute,now.second)
            self.cc_datepicker.SetValue(wxnow)
            self.cc_timepicker.SetValue(wxnow)
        mainSizer.Add(self.rowBuilder([lblrt, dtSizer]), 0, wx.EXPAND)
        
        lblpr = wx.StaticText(self, size=size)
        lblpr.SetLabel("Patient Repositioned?:")
        lblpr.SetFont(lblfont)
        self.repositioning_flag = wx.CheckBox(self, wx.ID_ANY)
        self.repositioning_flag.SetFont(txtfont)
        if row and not(self.addRecord):
            self.repositioning_flag.SetValue(self.objInstance.repositioning_flag)
        else:
            self.repositioning_flag.SetValue(False)
        mainSizer.Add(self.rowBuilder([lblpr, self.repositioning_flag]), 0, wx.EXPAND)

        lblfp = wx.StaticText(self, size=size)
        lblfp.SetLabel("Final Position:")
        lblfp.SetFont(lblfont)
        self.final_position = wx.ComboBox(self, wx.ID_ANY, choices=[_("Back"), _("Left Side"), _("Right Side"), _("Stomach")], style=wx.CB_DROPDOWN | wx.CB_DROPDOWN)
        self.final_position.SetFont(txtfont)
        if row and not(self.addRecord):
            self.final_position.SetStringSelection(self.objInstance.final_position)
        else:
            self.final_position.SetSelection(0)
        mainSizer.Add(self.rowBuilder([lblfp, self.final_position]), 0, wx.EXPAND)
        
        lbln = wx.StaticText(self, size=size)
        lbln.SetLabel("Notes:")
        lbln.SetFont(lblfont)
        if row and not(self.addRecord):
            self.repositioning_description = wx.TextCtrl(self, style=wx.TE_MULTILINE, value=self.objInstance.repositioning_description)
        else:
            self.repositioning_description = wx.TextCtrl(self, style=wx.TE_MULTILINE, value="")
        self.repositioning_description.SetFont(txtfont)
        mainSizer.Add(self.rowBuilder([lbln, self.repositioning_description]), 0, wx.EXPAND)
        
        okBtn = wx.Button(self, label="%s" % title)
        okBtn.Bind(wx.EVT_BUTTON, self.onRecord)
        btnSizer.Add(okBtn, 0, wx.ALL, 5)
        cancelBtn = wx.Button(self, label="Close")
        cancelBtn.Bind(wx.EVT_BUTTON, self.onClose)
        btnSizer.Add(cancelBtn, 0, wx.ALL, 5)
        
        mainSizer.Add(btnSizer, 0, wx.CENTER)
        self.SetSizer(mainSizer)
        

    def getData(self):
        """
        Gets the data from the text controls.
        """
        data = {}
        data['patient_id'] = self.objInstance.patient_id
        adate_wx = self.cc_datepicker.GetValue()
        atime_wx = self.cc_timepicker.GetValue(as_wxDateTime=True)
        adate = wx.calendar._wxdate2pydate(adate_wx)
        atime = time(atime_wx.GetHour(),atime_wx.GetMinute(),atime_wx.GetSecond())
        self.clinical_rounding_time = datetime.combine(adate,atime)
        data['clinical_rounding_time'] = self.clinical_rounding_time
        data['repositioning_flag'] = self.repositioning_flag.GetValue()
        data['final_position'] = self.final_position.GetValue()
        data['repositioning_description'] = self.repositioning_description.GetValue()
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
        lbl.SetMinSize(wx.Size(150,-1))
        sizer.Add(lbl, 0, wx.ALL, 5)
        sizer.Add(txt, 1, wx.EXPAND|wx.ALL, 5)
        return sizer
            
