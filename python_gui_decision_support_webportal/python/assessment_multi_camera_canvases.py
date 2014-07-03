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
from experiment_config import ExperimentConfig

class AssessmentMultiCameraCanvases:
    """
    This class creates the OpenGL canvas to be used for display depth camera video.
    It also, creates and runs the C++ class which controls the camera and the processing
    algorithms.
    Methods:
        __init__(parent, ID, title) - initializes the OpenGL canvas
        GetWindow() - gets the wxWidget window that is part of the canvas
    """
    def __init__(self, parent_3d, ID_3d, title_3d, parent_rgb, ID_rgb, title_rgb, parent_depth, ID_depth, title_depth,
                 parent_thermal, ID_thermal, title_thermal, parent_multispectral, ID_multispectral, title_multispectral, 
                 parent_rgb_copy, ID_rgb_copy, title_rgb_copy,
                 parent_rgb_touch, ID_rgb_touch, title_rgb_touch,
                 parent_thermal_touch, ID_thermal_touch, title_thermal_touch,
                 parent_multispectral_touch, ID_multispectral_touch, title_multispectral_touch, 
                 db, experimentId):
        """
        This method creates and initializes the OpenGL canvas and creates the camera
        procedure for controlling the camera and running the image processing algorithms.
        The camera procedure is a C++ class that is exposed to Python via SWIG.
        """
        self.db = db
        self.parent_3d = parent_3d
        self.window_id_3d = ID_3d
        self.parent_rgb = parent_rgb
        self.window_id_rgb = ID_rgb
        self.parent_depth = parent_depth
        self.window_id_depth = ID_depth
        self.parent_thermal = parent_thermal
        self.window_id_thermal = ID_thermal
        self.parent_multispectral = parent_multispectral
        self.window_id_multispectral = ID_multispectral
        self.parent_rgb_copy = parent_rgb_copy
        self.window_id_rgb_copy = ID_rgb_copy
        self.parent_rgb_touch = parent_rgb_touch
        self.window_id_rgb_touch = ID_rgb_touch
        self.parent_thermal_touch = parent_thermal_touch
        self.window_id_thermal_touch = ID_thermal_touch
        self.parent_multispectral_touch = parent_multispectral_touch
        self.window_id_multispectral_touch = ID_multispectral_touch        
        
        print "AssessmentMultiCameraCanvases constructor: experimentId = %d" % experimentId
        #return # BRH
        c = ExperimentConfig()
        self.config_file = c.SetFromDb(self.db, experimentId)

        self.proc_ = pu.pu_assessment_system_proc()
        if not self.proc_.configure(self.config_file): 
            print 'Configure failed.'
            return
        if not self.proc_.initialize():
            print 'Initialize failed.'
            return

        self.ctrl_ = framework.process_control()
        self.ctrl_.set_process(self.proc_)
        self.ctrl_.initialize_control(self.ctrl_.ASYNCHRONOUS);
        self.ctrl_.execute_one_step(True)
        self.ctrl_.execute_continously()
        
        attribList = (glcanvas.WX_GL_RGBA, # RGBA
                      glcanvas.WX_GL_DOUBLEBUFFER, # Double Buffered
                      glcanvas.WX_GL_BUFFER_SIZE, 32)
        
        self.canvas_rgb = gui_wx.canvas(self.parent_rgb, self.window_id_rgb, wx.DefaultPosition, wx.Size(640, 360), 0, title_rgb, attribList)
#        self.canvas_3d = gui_wx.canvas(self.parent_3d, self.window_id_3d, wx.DefaultPosition, wx.Size(320, 240), 0, title_3d, attribList)
        self.canvas_depth = gui_wx.canvas(self.parent_depth, self.window_id_depth, wx.DefaultPosition, wx.Size(320, 240), 0, title_depth, attribList)
        self.canvas_thermal = gui_wx.canvas(self.parent_thermal, self.window_id_thermal, wx.DefaultPosition, wx.Size(320, 240), 0, title_thermal, attribList)
        self.canvas_multispectral = gui_wx.canvas(self.parent_multispectral, self.window_id_multispectral, wx.DefaultPosition, wx.Size(256, 256), 0, title_multispectral, attribList)
        self.canvas_rgb_copy = gui_wx.canvas(self.parent_rgb_copy, self.window_id_rgb_copy, wx.DefaultPosition, wx.Size(640, 360), 0, title_rgb_copy, attribList)
        if self.parent_rgb_touch:
            self.canvas_rgb_touch = gui_wx.canvas(self.parent_rgb_touch, self.window_id_rgb_touch, wx.DefaultPosition, wx.Size(640, 360), 0, title_rgb_touch, attribList)
            self.canvas_thermal_touch = gui_wx.canvas(self.parent_thermal_touch, self.window_id_thermal_touch, wx.DefaultPosition, wx.Size(160, 120), 0, title_thermal_touch, attribList)
            self.canvas_multispectral_touch = gui_wx.canvas(self.parent_multispectral_touch, self.window_id_multispectral_touch, wx.DefaultPosition, wx.Size(160, 160), 0, title_multispectral_touch, attribList)
        
        self.canvas_rgb.set_process_control(self.ctrl_)
#        self.canvas_3d.set_process_control(self.ctrl_)
        self.canvas_depth.set_process_control(self.ctrl_)
        self.canvas_thermal.set_process_control(self.ctrl_)
        self.canvas_multispectral.set_process_control(self.ctrl_)
        self.canvas_rgb_copy.set_process_control(self.ctrl_)
        if self.parent_rgb_touch:
            self.canvas_rgb_touch.set_process_control(self.ctrl_)
            self.canvas_thermal_touch.set_process_control(self.ctrl_)
            self.canvas_multispectral_touch.set_process_control(self.ctrl_)
        # Create visualizers
        self.viz_rgb = gui_wx.direct_visualizer()
#        self.viz_3d = gui_wx.direct_visualizer()
        self.viz_depth = gui_wx.direct_visualizer()
        self.viz_thermal = gui_wx.direct_visualizer()
        self.viz_multispectral = gui_wx.direct_visualizer()
        self.viz_rgb_copy = gui_wx.direct_visualizer()
        if self.parent_rgb_touch:
            self.viz_rgb_touch = gui_wx.direct_visualizer()
            self.viz_thermal_touch = gui_wx.direct_visualizer()
            self.viz_multispectral_touch = gui_wx.direct_visualizer()
        # Visualizer visualizes into canvas
        self.viz_rgb.set_output_canvas(self.canvas_rgb)
#        self.viz_3d.set_output_canvas(self.canvas_3d)
        self.viz_depth.set_output_canvas(self.canvas_depth)
        self.viz_thermal.set_output_canvas(self.canvas_thermal)
        self.viz_multispectral.set_output_canvas(self.canvas_multispectral)
        self.viz_rgb_copy.set_output_canvas(self.canvas_rgb_copy)
        if self.parent_rgb_touch:
            self.viz_rgb_touch.set_output_canvas(self.canvas_rgb_touch)
            self.viz_thermal_touch.set_output_canvas(self.canvas_thermal_touch)
            self.viz_multispectral_touch.set_output_canvas(self.canvas_multispectral_touch)
        # Tell canvas about visualizer (needed for resizing when proc is paused)
        self.canvas_rgb.set_proc_visualizer(self.viz_rgb)
#        self.canvas_3d.set_proc_visualizer(self.viz_3d)
        self.canvas_depth.set_proc_visualizer(self.viz_depth)
        self.canvas_thermal.set_proc_visualizer(self.viz_thermal)
        self.canvas_multispectral.set_proc_visualizer(self.viz_multispectral)
        self.canvas_rgb_copy.set_proc_visualizer(self.viz_rgb_copy)
        if self.parent_rgb_touch:
            self.canvas_rgb_touch.set_proc_visualizer(self.viz_rgb_touch)
            self.canvas_thermal_touch.set_proc_visualizer(self.viz_thermal_touch)
            self.canvas_multispectral_touch.set_proc_visualizer(self.viz_multispectral_touch)
        self.viz_rgb.use_swap_buffer_on_flush(True);
#        self.viz_3d.use_swap_buffer_on_flush(True);
        self.viz_depth.use_swap_buffer_on_flush(True);
        self.viz_thermal.use_swap_buffer_on_flush(True);
        self.viz_multispectral.use_swap_buffer_on_flush(True);
        self.viz_rgb_copy.use_swap_buffer_on_flush(True);
        if self.parent_rgb_touch:
            self.viz_rgb_touch.use_swap_buffer_on_flush(True);
            self.viz_thermal_touch.use_swap_buffer_on_flush(True);
            self.viz_multispectral_touch.use_swap_buffer_on_flush(True);
        # Add a canvas view client.
        # This is actually a handler client.
        self.canvas_rgb.add_canvas_view_client();
#       self.canvas_3d.add_canvas_view_client();
        self.canvas_depth.add_canvas_view_client();
        self.canvas_thermal.add_canvas_view_client();
        self.canvas_multispectral.add_canvas_view_client();
        self.canvas_rgb_copy.add_canvas_view_client();
        if self.parent_rgb_touch:
            self.canvas_rgb_touch.add_canvas_view_client();
            self.canvas_thermal_touch.add_canvas_view_client();
            self.canvas_multispectral_touch.add_canvas_view_client();
        self.proc_.set_rgb_visualizer(self.viz_rgb);
#        self.proc_.set_3d_visualizer(self.viz_3d);       
        self.proc_.set_depth_visualizer(self.viz_depth);
        self.proc_.set_thermal_visualizer(self.viz_thermal);
        self.proc_.set_hyper_spectral_visualizer(self.viz_multispectral);
        self.proc_.set_rgb_copy_visualizer(self.viz_rgb_copy);
        if self.parent_rgb_touch:
            self.proc_.set_rgb_touch_visualizer(self.viz_rgb_touch);
            self.proc_.set_thermal_touch_visualizer(self.viz_thermal_touch);
            self.proc_.set_hyper_spectral_touch_visualizer(self.viz_multispectral_touch);
            #print "I am here."
        return

        
    def GetRgbWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the RGB camera.
        """
        return self.parent_rgb.FindWindowById(self.window_id_rgb)
 
    def Get3dWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the 3-D reconstruction.
        """
        return self.parent_3d.FindWindowById(self.window_id_3d)
 
    def GetDepthWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the depth camera.
        """
        return self.parent_depth.FindWindowById(self.window_id_depth)
 
    def GetThermalWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the thermal camera.
        """
        return self.parent_thermal.FindWindowById(self.window_id_thermal)
           
    def GetMultispectralWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the multi-spectral camera.
        """
        return self.parent_multispectral.FindWindowById(self.window_id_multispectral)
           
    def GetRgbCopyWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the RGB camera on the Multi-Spectral pane.
        """
        return self.parent_rgb_copy.FindWindowById(self.window_id_rgb_copy)
 
    def GetRgbTouchWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the RGB camera on the Multi-Spectral pane on the touchscreen.
        """
        return self.parent_rgb_touch.FindWindowById(self.window_id_rgb_touch)
 
    def GetThermalTouchWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the thermal camera on the touchscreen.
        """
        return self.parent_thermal_touch.FindWindowById(self.window_id_thermal_touch)
           
    def GetMultispectralTouchWindow(self):
        """
        This method returns the wxWindow associated with the OpenGL canvas for the multi-spectral camera on the touchscreen.
        """
        return self.parent_multispectral_touch.FindWindowById(self.window_id_multispectral_touch)
           
    def SetupRecording(self, rgb_folder, depth_folder, thermal_folder, multispectral_folder):
        """
        This method sets up the recording of camera data to the various files 
        """
        #return # BRH
        self.ctrl_.execute_one_step(True)
        self.proc_.set_rgb_file_directory(str(rgb_folder))
        self.proc_.set_depth_file_directory(str(depth_folder))
        self.proc_.set_thermal_file_directory(str(thermal_folder))
        self.proc_.set_hyperspectral_file_directory(str(multispectral_folder))
        self.ctrl_.execute_continously()
        
        return
            
    def StartRecording(self, activeVisualBtn):
        """
        This method starts the recording of camera data to the various files 
        """
        #return  # BRH
        self.ctrl_.execute_one_step(True)        
        self.proc_.start_recording(activeVisualBtn)
        self.ctrl_.execute_continously()
        
        return
            
    def StopRecording(self):
        """
        This method stops the recording of camera data to the various files 
        """
        #return  # BRH
        self.ctrl_.execute_one_step(True)
        self.proc_.stop_recording()
        self.ctrl_.execute_continously()
        
        return
            
    def SetupPlayback(self, rgb_folder, depth_folder, thermal_folder, multispectral_folder):
        """
        This method sets up the playback of camera data from the various files 
        """
        #return  # BRH
        self.ctrl_.execute_one_step(True)
        self.proc_.set_rgb_file_directory(str(rgb_folder))
        self.proc_.set_depth_file_directory(str(depth_folder))
        self.proc_.set_thermal_file_directory(str(thermal_folder))
        self.proc_.set_hyperspectral_file_directory(str(multispectral_folder))
        self.ctrl_.execute_continously()
        
        return
            
    def StartPlayback(self, activeVisualBtn):
        """
        This method starts the playback of camera data from the various files 
        """
        #return  # BRH
        self.ctrl_.execute_one_step(True)
        self.proc_.playback_recording(activeVisualBtn)
        self.ctrl_.execute_continously()
        
        return 
            
    def StopPlayback(self):
        """
        This method stops the playback recorded camera data 
        """
        # self.proc_.stop_playback()    # BRH
        
        return
            
    def TimeToQuit(self, event):
        """
        This method terminates the visualizations.
        """
        #return  # BRH
        self.ctrl_.terminate_execution()
        self.ctrl_.wait_until_paused()
        self.proc_ = None
        self.viz_rgb.set_output_canvas(None)
        self.viz_rgb = None
#        self.viz_3d.set_output_canvas(None)
        self.viz_3d = None
        self.viz_depth.set_output_canvas(None)
        self.viz_depth = None
        self.viz_thermal.set_output_canvas(None)
        self.viz_thermal = None
        self.viz_multispectral.set_output_canvas(None)
        self.viz_multispectral = None
        self.viz_rgb_copy.set_output_canvas(None)
        self.viz_rgb_copy = None
        self.viz_rgb_touch.set_output_canvas(None)
        self.viz_rgb_touch = None
        self.viz_thermal_touch.set_output_canvas(None)
        self.viz_thermal_touch = None
        self.viz_multispectral_touch.set_output_canvas(None)
        self.viz_multispectral_touch = None
        self.ctrl_ = None
        self.canvas_ = None
        self.Unbind(wx.EVT_CLOSE)
        self.Close(True)
        
