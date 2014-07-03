# Copyright (C) 2009 General Electric Company
#
# This software is intellectual property of General Electric Co.
# and may not be copied or redistributed without express written consent.
#
# python -c "import gevxlpy.security.proc.test.proc_aui_test"

import gevxlpy.util.configuration as cfg
import gevxl.gui_wx as gui_wx
import gevxl.img as img
import gevxl.util as util
import gevxl.framework as framework
import vxl.vil as vil
import vxl.vcl as vcl
import wx
from wx import glcanvas
import wx.aui
import gevxl.vid as vid
import gevxl.security.pressure_ulcer.pressure_ulcer as pu

class MyFrame(wx.Frame):
    def __init__(self, parent, ID, title):
        wx.Frame.__init__(self, parent, ID, title,
                          wx.DefaultPosition, wx.Size(640, 480),
                          style=wx.DEFAULT_FRAME_STYLE | wx.SUNKEN_BORDER | wx.CLIP_CHILDREN)

        path = 'C:/Barry/projects/mmpspupcva/src/python/resources/test_depth_camera_config.py'
        
        print 'Importing configuration from '+path

        c = cfg.config_file_from_python_script(path)

        self.proc_ = pu.pu_camera_source_proc()
        if not self.proc_.configure(c[0]): 
            print 'Configure failed.'
            return
        if not self.proc_.initialize():
            print 'Initialize failed.'
            return
 
        # from wx gui frame --------------------------------------------------------------------------
        self.control_panel_id_ = 33       
        self.enable_status_bar_ = True
        self.enable_menu_bar_ = True
        self.enable_process_control_panel_ = True
        self.enable_process_control_client_ = True
        self.enable_scroll_bar_ = False
        self.scroll_bar_ = None
        
        # Set up timer on frame
        # self.Connect(self, wx.ID_ANY, -1, wx.EVT_TIMER, self.TimerCallback)
               
        self.panel_ = wx.Panel(self, wx.ID_ANY, wx.DefaultPosition, wx.DefaultSize)
        # --------------------------------------------------------------------------------------------
        
        self.ctrl_ = framework.process_control()
        self.ctrl_.set_process(self.proc_)
        self.ctrl_.initialize_control(self.ctrl_.ASYNCHRONOUS);
        self.ctrl_.execute_one_step(True)
        self.ctrl_.execute_continously()
        
        # from wx gui frame (init)--------------------------------------------------------------------       
        if self.enable_status_bar_:
            self.CreateStatusBar()
            self.SetStatusText("This is the statusbar")

        if self.enable_menu_bar_:
            self.menu_bar_ = wx.MenuBar(wx.MB_DOCKABLE)
        if self.ctrl_:
            if self.enable_process_control_panel_:
                self.control_panel_ = gui_wx.process_control_panel(self.panel_, self.control_panel_id_, wx.DefaultPosition, wx.Size(-1,20))
                self.control_panel_.set_process_control(self.ctrl_)
            if self.enable_process_control_client_ and self.enable_menu_bar_:
                self.process_control_client_ = gui_wx.process_control_client()
                self.process_control_client_.set_process_control(self.ctrl_)
                self.process_control_client_.set_parent(self)
                self.process_control_client_.update_menu_as_submenu(self.menu_bar_, "&Process Control")
        self.top_sizer_ = wx.BoxSizer(wx.VERTICAL)
        self.control_sizer_ = None
        if self.enable_process_control_panel_ or self.enable_scroll_bar_:
            self.control_sizer_ = wx.BoxSizer(wx.HORIZONTAL)
            if self.enable_process_control_panel_ and self.control_panel_:
                control_panel_window = self.FindWindowById(self.control_panel_id_)
                self.control_sizer_.Add(control_panel_window, 1, wx.ALIGN_CENTER_VERTICAL, 0)
        if self.enable_scroll_bar_:
            self.top_sizer_.Add(self.scroll_bar_, 0, wx.GROW, 0)
        if self.control_sizer_:
            self.top_sizer_.Add(self.control_sizer_, 0, wx.TOP or wx.EXPAND, 2)
        self.panel_.SetAutoLayout(True )
        # self.panel_.SetSizer(self.top_sizer_)
        self.top_sizer_.SetSizeHints(self.panel_)
        if self.enable_menu_bar_:
            self.SetMenuBar(self.menu_bar_ )
         
        # --------------------------------------------------------------------------------------------

        attribList = (glcanvas.WX_GL_RGBA, # RGBA
                      glcanvas.WX_GL_DOUBLEBUFFER, # Double Buffered
                      glcanvas.WX_GL_BUFFER_SIZE, 32)
                      
        self.canvas_ = gui_wx.canvas(self.panel_, 40, wx.DefaultPosition, wx.Size(640, 480), 0, 'PUDepthCanvas', attribList)
        
        self.canvas_.set_process_control(self.ctrl_)
        canvas_window = self.FindWindowById(40)
        canvas_window.SetMinSize(wx.Size(640,480))
        self.top_sizer_.Add(canvas_window, 0, wx.EXPAND, 0, self.canvas_)
        self.panel_.SetSizer(self.top_sizer_)
        self.panel_.Layout()
        wx.Sleep(2)
        print "Canvas ready?  %s" % self.canvas_.is_ready()
        # Create visualizers
        self.viz_ = gui_wx.direct_visualizer()
        # self.overlay_viz_ = img.visualizer_2d_buffered()
        # Visualizer visualizes into canvas
        self.viz_.set_output_canvas(self.canvas_)
        # Tell canvas about visualizer (needed for resizing when proc is paused)
        self.canvas_.set_proc_visualizer(self.viz_)
        # Tell OpenGL visualizer owned by proc about a secondary visualizer 
        # used by clients for overlays.
        # self.viz_.add_visualizer(self.overlay_viz_);
        # Tell canvas about this overlay visualizer. This visualizer can be 
        # used by clients during events such as on_mouse().
        # self.canvas_.set_visualizer(self.overlay_viz_);
        # Don't use swap buffer if we don't double buffer.
        self.viz_.use_swap_buffer_on_flush(True);
        # Add a canvas view client.
        # This is actually a handler client.
        self.canvas_.add_canvas_view_client();
  
        # self.pixel_picking_client_ = new pu_pixel_picking_client();
        # self.pixel_picking_client_.set_proc(self.proc_);
        # self.add_client(self.pixel_picking_client_);
        # self.canvas_.add_client(self.pixel_picking_client_);

        # if self.enable_menu_bar_:
        # update menu with clients
            
        # Tell process about visualizer.
        self.proc_.set_visualizer(self.viz_);
        # Set frame attributes
        self.Bind(wx.EVT_CLOSE, self.TimeToQuit)
        # self.set_process(self.proc_)
        # self.set_process_control(self.ctrl_)
        # self.enable_scroll_bar(True)
        # self.init()
        # self.activate_scroll_bar(False)
        self.Show()
        print "Canvas ready?  %s" % self.canvas_.is_ready()

        return
        
        
    def TimeToQuit(self, event):
        self.ctrl_.terminate_execution()
        self.ctrl_.wait_until_paused()
        self.proc_=None
        self.viz_.set_output_canvas(None)
        self.viz_=None
        self.ctrl_=None
        self.canvas_=None
        self.Unbind(wx.EVT_CLOSE)
        self.Close(True)
        
      
class MyApp(wx.App):
    def OnInit(self):
        fre = MyFrame(None, -1, "GUI for Depth Camera")
        fre.Show(True)
        self.SetTopWindow(fre)
        print "Canvas ready?  %s" % fre.canvas_.is_ready()
        return True

app = MyApp(0)
app.MainLoop()

print 'Cya!!'
