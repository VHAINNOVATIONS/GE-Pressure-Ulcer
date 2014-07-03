# Copyright (C) 2009 General Electric Company
#
# This software is intellectual property of General Electric Co.
# and may not be copied or redistributed without express written consent.

import gevxlpy.util.configuration
import os

cfg=gevxlpy.util.configuration.cfg()

print '################ config file start ################'
print 'THIS_FILE = \'test_depth_camera.cfg\'   ##'

# changeable params
cfg['vid::source_process::live'] = True
cfg['vid::source_process::oni_filename'] = '//nsk1nas02/vai2pu/developments/patient_turning/grc_data/2014_0116/Peter_02_assisted_turn.oni'
cfg['vid::source_process::depth_to_color_registration'] = False

####################################################################
# pre-fixed params, openni2, micro_epsilon_socket, pcsdk

cfg['vid::source_process::type'] = 'openni2'
cfg['gevxl::pressure_ulcer::pu_camera_source_proc::source_process_type'] = 'openni2'

####################################################################
# for openni2 camera's intrinsic parameter

# openni2 rgb camera
cfg['vid::source_process::camera_intrinsic_vector'] = '[532.69041 0 328.73274 0 530.63846 254.07008 0 0 1]'

# openni2 depth camera
#cfg['vid::source_process::camera_intrinsic_vector'] = '[593.08673 0 319.15995 0 591.60550 246.86875 0 0 1]'

####################################################################
# for 3D depth camera, output type = "rgb" and "depth" and "rgb_in_depth_view"
# for thermal camera, output type = "grayscale" and "color"
cfg['gevxl::pressure_ulcer::pu_camera_source_proc::source_output_type'] = 'depth'

#cfg['vid::source_process::camera_id'] = 0
#cfg['gevxl::pressure_ulcer::pu_thermal_analysis_proc::b_ratio'] = 0.100000

print '###################################################'
print '# Config file end.                                #'
print '###################################################'
print '################ config file end ##################'
