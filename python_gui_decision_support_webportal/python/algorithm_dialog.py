import add_modify_dialog
import util
import olv_dialog_controller
from olv_dialog import OLVDialog
from olv_dialog_controller import OlvDialogController
import wx
from ObjectListView import ObjectListView, Filter
from algorithm import Algorithm
from algorithm_olv import OlvAlgorithm, OlvAlgorithmCols
from algorithm_controller import AlgorithmController
from algorithm_defaults import AlgorithmDefaults
from algorithm_defaults_olv import OlvAlgorithmDefaults, OlvAlgorithmDefaultsCols
from algorithm_defaults_dialog import AlgorithmDefaultsDialog
from experiment_dialog import ExperimentDialog

########################################################################
class AlgorithmDialog(wx.Dialog):
    """
    This class implements the generic select/add/update/delete dialog for a database object.
    It constructs the list of objects and places them in an ObjectListView widget.
    It then implements the button handlers for calling the add_modify_dialog to add or modify
    the object. Selection and deletion are handled in this dialog by calling the
    olv_dialog_controller controller.
    Methods:
        __init__(parent, db, obj, objOlv, objOlvCols, mode) - creates the widgets in the panel and performs initialization
        getSelectedObject() - Gets the selected object in the ObjectListView
        onAddRecord(event) - Button handler to add a record to the database
        onEditRecord(event) - Button handler to edit a record
        onDeleteRecord(event) - Button handler to delete a record
        onSearch(event) - Search field handler to search database based on the user's filter choice and keyword
        onSelectRecord(event) - Button handler to select a record
        onShowAllRecord(event) - Button handler to update the record list to show all of them
        setResultsOlv() - Sets the columns and objects in the ObjectListView
        showAllRecords() - Shows all records in the object list view control
    """

    #----------------------------------------------------------------------
    def __init__(self, parent, db, mode="Add-Update-Delete"):
        """
        Constructor which creates the modal dialog and its widgets, instantiates an
        ObjectlistView and populates it with the results from a query containing all
        database objects in a class.
        Arguments:
            parent - Parent window
            db - Database connection object
            mode - Dialog mode which can be either "Add-Update-Delete" or "Select"
        """
        self.db = db
        self.obj = Algorithm
        self.objOlv = OlvAlgorithm
        self.objOlvCols = OlvAlgorithmCols()
        width = self.objOlvCols.getTotalColumnWidth()
        wx.Dialog.__init__(self, parent, size=wx.Size(width,500))
        self.controller = AlgorithmController(db, self.obj, self.objOlv, self.objOlvCols)
        try:
            self.results = self.controller.getAllRecords()
        except:
            self.results = []
        
        font = wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.BOLD) 
        lbl = wx.StaticText(self, label=self.obj.displayTableName)
        lbl.SetFont(font)

        mainSizer = wx.BoxSizer(wx.VERTICAL)
        searchSizer = wx.BoxSizer(wx.HORIZONTAL)
        btnSizer = wx.BoxSizer(wx.HORIZONTAL)
        font = wx.Font(10, wx.SWISS, wx.NORMAL, wx.BOLD) 
        
        # create the search related widgets
        searchByLbl = wx.StaticText(self, label="Search By:")
        searchByLbl.SetFont(font)
        searchSizer.Add(searchByLbl, 0, wx.ALL, 5)
        
        self.search = wx.SearchCtrl(self, style=wx.TE_PROCESS_ENTER)
        self.search.Bind(wx.EVT_TEXT_ENTER, self.onSearch)
        searchSizer.Add(self.search, 0, wx.ALL, 5)
        
        self.resultsOlv = ObjectListView(self, style=wx.LC_REPORT
                                                        |wx.SUNKEN_BORDER)
        self.resultsOlv.SetEmptyListMsg("No Records Found")
        self.setResultsOlv()
        
        # create the button row
        if mode == "Select-Only":
            selectRecordBtn = wx.Button(self, label="Select")
            selectRecordBtn.Bind(wx.EVT_BUTTON, self.onSelectRecord)
            btnSizer.Add(selectRecordBtn, 0, wx.ALL, 5)
        
        if mode == "Add-Update-Delete":
            addRecordBtn = wx.Button(self, label="Add")
            addRecordBtn.Bind(wx.EVT_BUTTON, self.onAddRecord)
            btnSizer.Add(addRecordBtn, 0, wx.ALL, 5)
        
            editRecordBtn = wx.Button(self, label="Edit")
            editRecordBtn.Bind(wx.EVT_BUTTON, self.onEditRecord)
            btnSizer.Add(editRecordBtn, 0, wx.ALL, 5)
        
            deleteRecordBtn = wx.Button(self, label="Delete")
            deleteRecordBtn.Bind(wx.EVT_BUTTON, self.onDelete)
            btnSizer.Add(deleteRecordBtn, 0, wx.ALL, 5)
        
        showAllBtn = wx.Button(self, label="Show All")
        showAllBtn.Bind(wx.EVT_BUTTON, self.onShowAllRecord)
        btnSizer.Add(showAllBtn, 0, wx.ALL, 5)
        
        configBtn = wx.Button(self, label="Configuration")
        configBtn.Bind(wx.EVT_BUTTON, self.onConfiguration)
        btnSizer.Add(configBtn, 0, wx.ALL, 5)
        
        experimentsBtn = wx.Button(self, label="Experiments")
        experimentsBtn.Bind(wx.EVT_BUTTON, self.onExperiments)
        btnSizer.Add(experimentsBtn, 0, wx.ALL, 5)
        
        mainSizer.Add(lbl, 0, wx.CENTER)
        mainSizer.Add(searchSizer)
        mainSizer.Add(self.resultsOlv, 1, wx.ALL|wx.EXPAND, 5)
        mainSizer.Add(btnSizer, 0, wx.CENTER)
        self.SetSizer(mainSizer)
        # mainSizer.Fit(self)
        
    #----------------------------------------------------------------------
    def getSelectedObject(self):
        """
        Gets the selected object in the ObjectListView
        """
        return self.selectedObject
    
    #----------------------------------------------------------------------
    def onAddRecord(self, event):
        """
        Button handler to add a record to the database
        """
        dlg = add_modify_dialog.AddModifyDialog(self.controller, self.obj, title="Add", addRecord=True)
        rc = dlg.ShowModal()
        if rc == 0:
            self.showAllRecords()
        
    #----------------------------------------------------------------------
    def onEditRecord(self, event):
        """
        Button handler to edit a record
        """
        selectedRow = self.resultsOlv.GetSelectedObject()
        if selectedRow == None:
            util.showMessageDialog("No row selected!", "Error")
            return
        dlg = add_modify_dialog.AddModifyDialog(self.controller, self.obj, row=selectedRow, title="Modify",
                                           addRecord=False)
        rc = dlg.ShowModal()
        if rc == 0:
            self.showAllRecords()
        
    #----------------------------------------------------------------------
    def onDelete(self, event):
        """
        Button handler to delete a record
        """
        selectedRow = self.resultsOlv.GetSelectedObject()
        if selectedRow == None:
            util.showMessageDialog("No row selected!", "Error")
            return
        (rc, msg) = self.controller.deleteRecord(selectedRow.getKey())
        # Check return code from above and put up appropriate message dialog
        if rc == 0:
            util.showMessageDialog("Record Deleted Successfully!", "Success!", wx.ICON_INFORMATION)
        else:
            util.showMessageDialog(msg, "Failure!", wx.ICON_INFORMATION)
        self.showAllRecords()
        
    #----------------------------------------------------------------------
    def onConfiguration(self, event):
        """
        Button handler to show configuration parameters
        """
        selectedRow = self.resultsOlv.GetSelectedObject()
        if selectedRow == None:
            util.showMessageDialog("No row selected!", "Error")
            return
        algorithmId = selectedRow.getKey()
        olvDialog = AlgorithmDefaultsDialog(None, self.db, algorithmId)
        rc = olvDialog.ShowModal()
        olvDialog.Destroy()
        self.Enable()
        self.showAllRecords()

    #----------------------------------------------------------------------
    def onExperiments(self, event):
        """
        Button handler to show experiments for an algorithm
        """
        selectedRow = self.resultsOlv.GetSelectedObject()
        if selectedRow == None:
            util.showMessageDialog("No row selected!", "Error")
            return
        algorithmId = selectedRow.getKey()
        olvDialog = ExperimentDialog(None, self.db, algorithmId=algorithmId)
        rc = olvDialog.ShowModal()
        olvDialog.Destroy()
        self.Enable()
        self.showAllRecords()
        
    #----------------------------------------------------------------------
    def onSearch(self, event):
        """
        Search field handler to search database based on the user's filter choice and keyword
        """
        keyword = self.search.GetValue()
        Filter.TextSearch(self.resultsOlv,columns=(), text=keyword)
        
    #----------------------------------------------------------------------
    def onSelectRecord(self, event):
        """
        Button handler to select a record
        """
        selectedRow = self.resultsOlv.GetSelectedObject()
        if selectedRow == None:
            util.showMessageDialog("No row selected!", "Error")
            return
        key = selectedRow.getKey()
        self.selectedObject = self.controller.getRecordByKey(key)
        self.EndModal(0)
        
    #----------------------------------------------------------------------
    def onShowAllRecord(self, event):
        """
        Button handler to update the record list to show all of them
        """
        self.showAllRecords()
        
    #----------------------------------------------------------------------
    def setResultsOlv(self):
        """
        Sets the columns and objects in the ObjectListView
        """
        cd = self.objOlvCols.getColumnDefinitions()
        # print len(cd)
        self.resultsOlv.SetColumns(self.objOlvCols.getColumnDefinitions())
        self.resultsOlv.SetObjects(self.results)
        
    #----------------------------------------------------------------------
    def showAllRecords(self):
        """
        Shows all records in the object list view control
        """
        self.results = self.controller.getAllRecords()
        self.setResultsOlv()
        
