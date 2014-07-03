import util
import wx
from pprint import pprint

########################################################################
class AddModifyExperimentConfigurationDialog(wx.Dialog):
    """
    This class implements a modal dialog to add and modify experiment_configuration database records.
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

    def __init__(self, controller, obj, db, row=None, title="Add", addRecord=True, experimentId=None):
        """
        Initializes the add/modify dialog. This consists of constructing an input form
        which has field names and a text field for entering field values.
        Arguments:
            controller - class performing the object creation, update, or deletion
            obj - object class to be processed
            row - row selected containing the object
            title - title to be displayed on form
            addRecord - flag indicating that a new record is to be created
            experimentId - algorithm Id of the parent record
        """
        wx.Dialog.__init__(self, None, title="%s ExperimentConfiguration Record" % title)
        self.controller = controller
        if row:
            key = row.getKey()
            self.objInstance = controller.getRecordByKey(key)
        else:
            self.objInstance = obj()
            self.objInstance.experiment_id = experimentId
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
        for i in range(len(self.objInstance.editCols)):
            lblText = self.objInstance.editColsLabels[i]+":"
            print lblText
            # lbl = wx.StaticText(self, label=lblText, size=size)
            lbl = wx.StaticText(self, size=size)
            lbl.SetLabel(lblText)
            lbl.SetFont(font)
            if row:
                x = getattr(self.objInstance,self.objInstance.editCols[i])
                print x
                self.ctls.append(wx.TextCtrl(self, value=str(x)))
            else:
                self.ctls.append(wx.TextCtrl(self, value=""))
            mainSizer.Add(self.rowBuilder([lbl, self.ctls[i]]), 0, wx.EXPAND)
        
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
        data['experiment_id'] = self.objInstance.experiment_id
        for i in range(len(self.objInstance.editCols)):
            data[self.objInstance.editCols[i]] = self.ctls[i].GetValue()
        return data
    

    def onAdd(self):
        """
        Adds the record to the database and shows a confirmation message dialog.
        """
        data = self.getData()
        # print "AddModifyExperimentConfigurationDialog.onAdd - data = "
        # pprint(data)
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
        sizer.Add(txt, 1, wx.EXPAND|wx.ALL, 5)
        return sizer
            
