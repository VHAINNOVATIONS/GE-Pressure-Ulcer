insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('ASSESSMENT_DEPTH_VIDEO_FILE_SUB_DIR','depth_files');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('ASSESSMENT_RGB_VIDEO_FILE_SUB_DIR','rgb_files');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('ASSESSMENT_HD_RGB_VIDEO_FILE_SUB_DIR','hd_files');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('ASSESSMENT_THERMAL_VIDEO_FILE_SUB_DIR','thermal_files');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('ASSESSMENT_BIOCHEMICAL_VIDEO_FILE_SUB_DIR','biochemical_files');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('CHEMICAL_SAMPLE_TIME','120');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('CHEMICAL_BASELINE_SAMPLE_TIME','60');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('CHEMICAL_SENSOR_PORT','COM4');
insert into `va_pupc`.`system_configuration` (`parameter_name`, `parameter_value`) VALUES ('BASE_FILE_DIRECTORY','C:/tmp/pu');

INSERT INTO `va_pupc`.`patient_identification` (`va_patient_id`, `patient_name`, `patient_gender`, `data_files_base_directory`, `camera_id`) VALUES ('VA1234567890', 'Barry', 'male', '/tmp/barry', '1');
INSERT INTO `va_pupc`.`patient_identification` (`va_patient_id`, `patient_name`, `patient_gender`, `data_files_base_directory`, `camera_id`) VALUES ('VA1111111111', 'Ting', 'male','/tmp/ting', '2');
INSERT INTO `va_pupc`.`patient_identification` (`va_patient_id`, `patient_name`, `patient_gender`, `data_files_base_directory`, `camera_id`) VALUES ('VA2222222222', 'Vrinda', 'female', '/tmp/vrinda', '3');

INSERT INTO `va_pupc`.`braden_scores` (`patient_id`, `braden_scoring_date`, `sensory_perception_score`, `moisture_score`, `activity_score`, `mobility_score`, `nutrition_score`, `friction_shear_score`) VALUES ('1', '2014-03-14 09:24:00', '1', '2', '3', '4', '5', '6');
INSERT INTO `va_pupc`.`braden_scores` (`patient_id`, `braden_scoring_date`, `sensory_perception_score`, `moisture_score`, `activity_score`, `mobility_score`, `nutrition_score`, `friction_shear_score`) VALUES ('1', '2014-03-07 10:12:00', '6', '5', '4', '3', '2', '1');

INSERT INTO `va_pupc`.`patient_admission` (`patient_id`, `admission_date`, `admission_note`, `factors_impairing_healing`) VALUES ('1', '2014-03-03 07:00', 'LOI: 74 year old white male who is a T7 myelopathy ASIA B secondary to thoracic spinal ostemyelitis.  Transferred from MCG to SCIU for continued rehab on 8/2/13. Last seen with Plastic Surgery team o 8/28/13 at which time wound was debrided gently with currette and pressure dressing applied. Per Plastic Surgery recs. treatment for sacral wound was Santyl in am and clorpactin moist dressing in evening.', 'Diabetes. SCI. AKI');

INSERT INTO `va_pupc`.`patient_assessment` (`patient_id`, `assessment_date`, `assessment_note`) VALUES ('1', '2014-03-03 09:53', 'LOI: 74 year old white male who is a T7 myelopathy ASIA B secondary to thoracic spinal ostemyelitis.');

INSERT INTO `va_pupc`.`nutritional_status` (`patient_id`, `assessment_date`, `nutritional_notes`) VALUES ('1', '2014-03-03 10:22', 'Fair. Patient with PEG in place.');

INSERT INTO `va_pupc`.`treatment_plan` (`patient_id`, `plan_date`, `plan_notes`) VALUES ('1', '2014-03-03 09:59', 'Wound care as ordered. Scheduled bowel care. Foley cath to BSD. PEG feedings with possible protein supplementation if indicated (dialysis patient). Daily hygeine and application of moisturizing lotion as needed for dry skin. Turn and reposition every 2-3 hours while in bed. Offload boots to protect heels, feet, and ankles.');

INSERT INTO `va_pupc`.`algorithm` (`algorithm_name`) VALUES ('prevention pose');
INSERT INTO `va_pupc`.`algorithm` (`algorithm_name`) VALUES ('assessment measure');
INSERT INTO `va_pupc`.`algorithm` (`algorithm_name`) VALUES ('biochemical sensor');
INSERT INTO `va_pupc`.`algorithm` (`algorithm_name`) VALUES ('assessment camera');

INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'startFrequency','12000000.0');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'incrementFrequency','20000.0');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'offsetFrequency','12207.05');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'numberSamples','128');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'numberPoints','128');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'DDSDAC','257');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'calibrationSwitch','0');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'numberReps','17');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'numberSkips','64');
INSERT INTO `va_pupc`.`algorithm_defaults` (`algorithm_id`,`parameter_name`,`default_value`) VALUES (3,'flag','0');

INSERT INTO `va_pupc`.`experiment` (`experiment_name`, `algorithm_id`, `default_flag`) VALUES ('peter\'s pose', 1, 1);
INSERT INTO `va_pupc`.`experiment` (`experiment_name`, `algorithm_id`, `default_flag`) VALUES ('measure 1', 2, 1);
INSERT INTO `va_pupc`.`experiment` (`experiment_name`, `algorithm_id`, `default_flag`) VALUES ('assessment camera 1', 4, 1);

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::live', 'True');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::oni_filename', 'd:/pressure_ulcer/dump/test.oni');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::depth_to_color_registration', 'False');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::mirroring', 'False');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::type', 'openni2');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'vid::pu_prevention_source_process::camera_intrinsic_vector', '[532.69041 0 328.73274 0 530.63846 254.07008 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prevention_chaining_process::cur_frame_output_type', 'depth');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process::enabled', 'False');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process::depth_img_seg_root_folder', 'd:/pressure_ulcer/prevention');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prevention_videoarchive_writer_process::time_window_in_millisecond_in_subfolder', '600000');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process::plane_fitting_three_img_points_coordinates', '[85 64 131 442 352 48]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process::min_height_thresh', '0.40');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process::max_height_thresh', '1.50');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process::min_depth_thresh', '1.20');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_rectify_kinect_process::max_depth_thresh', '2.70');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process::motion_history_viz_width', '200');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process::motion_history_threshold', '5');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process::enable_morphology_noise_removal', 'True');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (1, 'gevxl::pressure_ulcer::prevention::pu_prv_motion_estimate_process::motion_histogram_morphology_se_radius', '3.0');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::camera_live', 'True');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::type', 'pcsdk');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::max_frame_waiting_time_in_ms', '30');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::depth_camera_intrinsic_vector', '[225.56560 0 158.40835 0 221.36116 127.66800 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::depth_camera_rotation_vector', '[1 0 0 0 1 0 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::depth_camera_translation_vector', '[0 0 0]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::rgb_camera_intrinsic_vector', '[990.22379 0 602.02912 0 988.50572 350.25879 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::rgb_camera_rotation_vector', '[0.9983 0.0031 0.0576 -0.0068 0.9979 0.0647 -0.0573 -0.0649 0.9962]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::rgb_camera_translation_vector', '[17.8268 -10.2620 13.1742]');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::file_storage_directory', 'd:/pressure_ulcer/assessment/assessment_rgbd');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::capture_mode', 'camera');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_rgbd_cam_source_process::save_out', 'False');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::live', 'True');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::type', 'micro_epsilon_socket');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::max_frame_waiting_time_in_ms', '30');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::thermal_camera_intrinsic_vector', '[705.98246 0 268.11744 0 710.09162 134.67785 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::thermal_camera_rotation_vector', '[0.9994 -0.0258 -0.0213 0.0255 0.9996 -0.0125 0.0216 0.0120 0.9997]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::thermal_camera_translation_vector', '[-45.3389 -27.3844 136.9797]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::color_palette_filepath', 'd:/pressure_ulcer/resource/Rainbow240RGB.dat');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::file_storage_directory', 'd:/pressure_ulcer/assessment/assessment_thermal');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::capture_mode', 'camera');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_thermal_cam_source_process::save_out', 'False');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::pu_thermal_analysis_proc::thermal_camera_intrinsic_vector', '[705.98246 0 191 0 710.09162 144 0 0 1]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::pu_thermal_analysis_proc::thermal_camera_rotation_vector', '[0.9994 -0.0258 -0.0213 0.0255 0.9996 -0.0125 0.0216 0.0120 0.9997]');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::pu_thermal_analysis_proc::thermal_camera_translation_vector', '[-45.3389 -27.3844 136.9797]');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_hyperspectral_cam_source_process::type', 'bayspec_socket');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_hyperspectral_cam_source_process::port', '27015');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_hyperspectral_cam_source_process::file_storage_directory', 'd:/pressure_ulcer/assessment/assessment_hyperspectral');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_hyperspectral_cam_source_process::white_ref_folder', 'd:/pressure_ulcer/resource/WhiteRefDarkBkg/12-24-59RES2048_EXP0030_BIT08-White');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'vid::pu_assessment_hyperspectral_cam_source_process::dark_bkg_folder', 'd:/pressure_ulcer/resource/WhiteRefDarkBkg/12-26-37RES2048_EXP0030_BIT08-Dark');

INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::optimal_snapshot_distance', '0.4');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::optimal_snapshot_distance_tolerance', '0.02');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::max_color_offset', '250');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::viz_outer_rect_to_image_border', '60');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::viz_inner_rect_to_image_border', '80');
INSERT INTO `va_pupc`.`experiment_configuration` (`experiment_id`, `parameter_name`, `parameter_value`) VALUES (3, 'gevxl::pressure_ulcer::assessment::pu_assessment_system_proc::viz_depth_or_color_in_depth', 'color_in_depth_view');

INSERT INTO `va_pupc`.`prevention_session` (`patient_id`, `depth_video_file_directory`, `start_time`, `end_time`) VALUES ('1', '/tmp/barry/session/1', '2013-12-18 9:30', '2013-12-19 9:45');

INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 9:31', '0', 'Right side');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 10:01', '1', 'Left side');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 12:30', '0', 'Back');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 15:22', '1', 'Right side');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 19:44', '1', 'Back');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-18 23:55', '0', 'Left side');
INSERT INTO `va_pupc`.`patient_turning` (`patient_id`, `session_id`, `experiment_id`, `turn_time`, `provider_present_flag`, `final_position`) VALUES ('1', '1', '1', '2013-12-19 2:59', '0', 'Right side');

INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('1', '0', 'Skin, Scalp');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('2', '1', 'Skin, Face');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('3', '2', 'Skin, Right Ear');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('4', '3', 'Skin, Left Ear');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('5', '4', 'Skin, Neck');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('6', '5', 'Skin, Chest');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('7', '6', 'Skin, Back');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('8', '7', 'Skin, Abdomen');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('9', '8', 'Skin, Buttock');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('10', '9', 'Skin, Perineum');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('11', 'A', 'Skin, Genitalia');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('12', 'B', 'Skin, Right Upper Arm');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('13', 'C', 'Skin, Left Upper Arm');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('14', 'D', 'Skin, Right Lower Arm');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('15', 'E', 'Skin, Left Lower Arm');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('16', 'F', 'Skin, Right Hand');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('17', 'G', 'Skin, Left Hand');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('18', 'H', 'Skin, Right Upper Leg');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('19', 'J', 'Skin, Left Upper Leg');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('20', 'K', 'Skin, Right Lower Leg');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('21', 'L', 'Skin, Left Lower Leg');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('22', 'M', 'Skin, Right Foot');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('23', 'N', 'Skin, Left Foot');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('24', 'P', 'Skin');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('25', 'Q', 'Finger Nail');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('26', 'R', 'Toe Nail');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('27', 'S', 'Hair');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('28', 'T', 'Breast, Right');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('29', 'U', 'Breast, Left');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('30', 'V', 'Breast, Bilateral');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('31', 'W', 'Nipple, Right');
INSERT INTO `va_pupc`.`skeleton_model` (`location_id`, `body_part_code`, `description`) VALUES ('32', 'X', 'Nipple, Left');

INSERT INTO `va_pupc`.`wound_assessment_map` (`patient_id`, `wound_id`, `wound_location_id`, `wound_location_description`) VALUES ('1', '1', '7', 'Lower left back just left of spine');
INSERT INTO `va_pupc`.`wound_assessment_map` (`patient_id`, `wound_id`, `wound_location_id`, `wound_location_description`) VALUES ('1', '2', '12', 'Right shoulder near collarbone');
INSERT INTO `va_pupc`.`wound_assessment_map` (`patient_id`, `wound_id`, `wound_location_id`, `wound_location_description`) VALUES ('3', '1', '9', 'Buttock, Right Center');

INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('1', '1', '1', '2014-01-02 09:45');
INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('1', '1', '2', '2014-01-09 09:32');
INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('1', '1', '3', '2014-01-16 09:00');
INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('3', '1', '1', '2014-01-02 10:45');
INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('3', '1', '2', '2014-01-09 10:32');
INSERT INTO `va_pupc`.`system_assessment_session` (`patient_id`, `wound_id`, `assessment_id`, `start_time`) VALUES ('3', '1', '3', '2014-01-16 10:00');

INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('1', '2', '2014-01-02 09:45:00', '5', '3', '0.5', '15', '11.25', '5.45', '10');
INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('2', '2', '2014-01-09 09:32:00', '5.4', '3.1', '0.6', '16.74', '12.10', '5.90', '11');
INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('3', '2', '2014-01-16 09:00:00', '5.1', '2.9', '0.5', '14.79', '11.30', '5.5', '10');
INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('4', '2', '2014-01-02 10:45:00', '4', '2', '0.5', '8', '7.25', '5.15', '10');
INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('5', '2', '2014-01-09 10:32:00', '4.4', '2.1', '0.6', '9.24', '8.10', '4.62', '11');
INSERT INTO `va_pupc`.`system_assessment_experiment_measure` (`session_id`, `experiment_id`, `start_time`, `length`, `width`, `depth`, `length_x_width`, `surface_area`, `wound_volume`, `push_score`) VALUES ('6', '2', '2014-01-16 10:00:00', '4.1', '1.9', '0.5', '7.79', '7.30', '4.5', '10');

INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('1', '2', '2014-01-02 09:45:00', '/tmp/ilm.map', '80.0', '13.3', '6.7', '0.0', '3');
INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('2', '2', '2014-01-09 09:32:00', '/tmp/ilm.map', '78.5', '13.5', '7.0', '1.0', '4');
INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('3', '2', '2014-01-16 09:00:00', '/tmp/ilm.map', '76.3', '14', '7.0', '3.7', '4');
INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('4', '2', '2014-01-02 10:45:00', '/tmp/ilm.map', '90.0', '5.3', '4.7', '0.0', '2');
INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('5', '2', '2014-01-09 10:32:00', '/tmp/ilm.map', '88.5', '5.25', '5.25', '1.0', '2');
INSERT INTO `va_pupc`.`system_assessment_experiment_segment` (`session_id`, `experiment_id`, `start_time`, `image_label_map_file`, `granulation_percentage`, `slough_percentage`, `eschar_percentage`, `bone_percentage`, `ulcer_stage`) VALUES ('6', '2', '2014-01-16 10:00:00', '/tmp/ilm.map', '89.0', '5', '5.3', '0.7', '2');

INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('1', '2', '2014-01-02 09:45:00', '90.0', '123', '59', '87.5', '239', '200', '87.0', 'Surrounding skin', '1.59', '20', '20', '20', '20', '20');
INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('2', '2', '2014-01-09 09:32:00', '90.7', '145', '66', '87.7', '240', '201', '87.1', 'Surrounding skin', '1.77', '20', '20', '20', '15', '25');
INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('3', '2', '2014-01-16 09:00:00', '91.5', '147', '70', '87.6', '244', '199', '87.2', 'Surrounding skin', '1.95', '20', '18', '22', '14', '26');
INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('4', '2', '2014-01-02 10:45:00', '91.0', '123', '59', '88.5', '239', '200', '88.0', 'Surrounding skin', '1.69', '10', '30', '20', '20', '20');
INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('5', '2', '2014-01-09 10:32:00', '92.7', '145', '66', '88.7', '240', '201', '88.1', 'Surrounding skin', '1.87', '10', '30', '20', '15', '25');
INSERT INTO `va_pupc`.`system_assessment_experiment_temperature` (`session_id`, `experiment_id`, `start_time`, `max_temperature`, `max_temperature_loc_x`, `max_temperature_loc_y`, `min_temperature`, `min_temperature_loc_x`, `min_temperature_loc_y`, `baseline_temperature`, `baseline_description`, `temperature_variation_sigma`, `temperature_segment_1_percentage`, `temperature_segment_2_percentage`, `temperature_segment_3_percentage`, `temperature_segment_4_percentage`, `temperature_segment_5_percentage`) VALUES ('6', '2', '2014-01-16 10:00:00', '92.5', '147', '70', '88.6', '244', '199', '88.2', 'Surrounding skin', '1.99', '10', '28', '22', '14', '26');
