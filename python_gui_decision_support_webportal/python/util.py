import wx
import ctypes
import os
import platform


def showMessageDialog(message, caption, flag=wx.ICON_ERROR):
    """"""
    msg = wx.MessageDialog(None, message=message,
                           caption=caption, style=flag)
    rc = msg.ShowModal()
    # msg.Destroy()
    return rc

def GetFreeSpaceGB(folder):
    """ 
    Return folder/drive free space (in bytes)
    """
    if platform.system() == 'Windows':
        print "folder: " + folder
        free_bytes = ctypes.c_ulonglong(0)
        ctypes.windll.kernel32.GetDiskFreeSpaceExW(ctypes.c_wchar_p(folder), None, None, ctypes.pointer(free_bytes))
        return free_bytes.value/1024/1024/1024
    elif hasattr(os,'statvfs'):
        st = os.statvfs(folder)
        return st.f_bavail * st.f_frsize/1024/1024/1024
    else:
        return 0