#!/usr/bin/env python
# -*- coding: ISO-8859-1 -*-
#
# generated by wxGlade 0.6.8 (standalone edition) on Sun Feb 09 14:09:15 2014
#

import wx

# begin wxGlade: dependencies
import gettext
# end wxGlade

# begin wxGlade: extracode
# end wxGlade


class LogonDialog(wx.Dialog):
    """
    This class implements the logon dialog for the Pressure Ulcer System GUI.
    It requests the users' id and password and then logs the user into the database.
    Methods:
        __init__(*args, **kwds) - creates the widgets in the panel and performs initialization
        __set_properties() - set various properties of the widgets
        __do_layout() - lays out the widgets
        __doLogon - Button handler for performing the login
        SetDb - Sets the database object
    """
    def __init__(self, *args, **kwds):
        """
        Creates the widgets in the panel and performs initialization
        """
        # begin wxGlade: LogonDialog.__init__
        kwds["style"] = wx.DEFAULT_DIALOG_STYLE
        wx.Dialog.__init__(self, *args, **kwds)
        self.bitmap_1 = wx.StaticBitmap(self, wx.ID_ANY, wx.Bitmap("icons\\header-logo.png", wx.BITMAP_TYPE_ANY))
        self.mmps_label = wx.StaticText(self, wx.ID_ANY, _("Multi-Modality Portable System for \nPressure Ulcer Prevention and Care"), style=wx.ALIGN_CENTRE)
        self.login_label = wx.StaticText(self, wx.ID_ANY, _("Please Log On"), style=wx.ALIGN_RIGHT)
        self.username_label = wx.StaticText(self, wx.ID_ANY, _("Username:"))
        self.username = wx.TextCtrl(self, wx.ID_ANY, "")
        self.password_label = wx.StaticText(self, wx.ID_ANY, _("Password:"))
        self.password = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_PASSWORD)
        self.error_text = wx.TextCtrl(self, wx.ID_ANY, "", style=wx.TE_READONLY | wx.NO_BORDER)
        self.login_button = wx.Button(self, wx.ID_ANY, _("Logon"))
        self.cancel_button = wx.Button(self, wx.ID_CANCEL, "")

        self.__set_properties()
        self.__do_layout()
        # end wxGlade
        self.login_button.Bind(wx.EVT_BUTTON, self.__doLogon)

    def __set_properties(self):
        """
        Sets various properties of the widgets
        """
        # begin wxGlade: LogonDialog.__set_properties
        self.SetTitle(_("Logon"))
        self.SetSize(wx.DLG_SZE(self, (476, 218)))
        self.mmps_label.SetMinSize((200, 80))
        self.mmps_label.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.BOLD, 0, ""))
        self.login_label.SetMinSize((115, 19))
        self.login_label.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.BOLD, 0, ""))
        self.username_label.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, ""))
        self.username.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD, 0, ""))
        self.password_label.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, ""))
        self.password.SetFont(wx.Font(10, wx.DEFAULT, wx.NORMAL, wx.BOLD, 0, ""))
        self.error_text.SetBackgroundColour(wx.Colour(240, 240, 240))
        self.error_text.SetForegroundColour(wx.Colour(255, 0, 0))
        self.error_text.SetFont(wx.Font(12, wx.DEFAULT, wx.NORMAL, wx.NORMAL, 0, ""))
        self.login_button.SetDefault()
        # end wxGlade

    def __do_layout(self):
        """
        Lays out the widgets in the frame
        """
        # begin wxGlade: LogonDialog.__do_layout
        sizer_1 = wx.BoxSizer(wx.VERTICAL)
        sizer_2 = wx.BoxSizer(wx.HORIZONTAL)
        sizer_5 = wx.BoxSizer(wx.HORIZONTAL)
        grid_sizer_1 = wx.GridSizer(3, 2, 8, 8)
        sizer_6 = wx.BoxSizer(wx.VERTICAL)
        sizer_6.Add(self.bitmap_1, 0, 0, 0)
        sizer_6.Add((20, 20), 0, 0, 0)
        sizer_6.Add(self.mmps_label, 0, wx.EXPAND | wx.ALIGN_CENTER_HORIZONTAL, 0)
        sizer_5.Add(sizer_6, 1, wx.EXPAND, 0)
        grid_sizer_1.Add(self.login_label, 0, wx.ALIGN_RIGHT | wx.ALIGN_CENTER_VERTICAL, 0)
        grid_sizer_1.Add((20, 15), 0, 0, 0)
        grid_sizer_1.Add(self.username_label, 0, wx.ALIGN_RIGHT, 0)
        grid_sizer_1.Add(self.username, 0, 0, 0)
        grid_sizer_1.Add(self.password_label, 0, wx.ALIGN_RIGHT, 0)
        grid_sizer_1.Add(self.password, 0, 0, 0)
        sizer_5.Add(grid_sizer_1, 1, wx.EXPAND, 0)
        sizer_1.Add(sizer_5, 1, wx.EXPAND, 0)
        sizer_1.Add(self.error_text, 0, wx.EXPAND, 0)
        sizer_2.Add(self.login_button, 0, 0, 0)
        sizer_2.Add(self.cancel_button, 0, wx.LEFT, 10)
        sizer_1.Add(sizer_2, 0, wx.ALL | wx.ALIGN_RIGHT, 5)
        self.SetSizer(sizer_1)
        self.Layout()
        # end wxGlade

    def __doLogon(self,event):
        """
        Button handler for performing the login.
        """
        user = self.username.GetValue()
        pw = self.password.GetValue()
        (rc,msg) = self.db.Logon(user,pw)
        if rc != 0:
            print "Error in logon, rc = %d" % rc
            print msg
            self.error_text.SetValue(str(msg))
        else:
            self.EndModal(1)

    def SetDb(self,db):
        """
        Sets the database object.
        """
        self.db = db
        
# end of class LogonDialog
#class LogonDialog(wx.App):
#    def OnInit(self):
#        wx.InitAllImageHandlers()
#        logon_dialog = LogonDialog(None, wx.ID_ANY, "")
#        self.SetTopWindow(logon_dialog)
#        logon_dialog.Show()
#        return 1

# end of class LogonDialog

if __name__ == "__main__":
    gettext.install("logonDialog") # replace with the appropriate catalog name

    logonDialog = LogonDialog(0)
    logonDialog.MainLoop()
