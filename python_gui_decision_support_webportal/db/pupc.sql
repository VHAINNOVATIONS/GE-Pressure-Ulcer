SET @OLD_UNIQUE_CHECKS=@@UNIQUE_CHECKS, UNIQUE_CHECKS=0;
SET @OLD_FOREIGN_KEY_CHECKS=@@FOREIGN_KEY_CHECKS, FOREIGN_KEY_CHECKS=0;
SET @OLD_SQL_MODE=@@SQL_MODE, SQL_MODE='TRADITIONAL,ALLOW_INVALID_DATES';

CREATE SCHEMA IF NOT EXISTS `VA_PUPC` ;
USE `VA_PUPC` ;

-- -----------------------------------------------------
-- Table `VA_PUPC`.`patient_identification`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`patient_identification` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`patient_identification` (
  `patient_id` INT NOT NULL AUTO_INCREMENT,
  `va_patient_id` VARCHAR(45) NOT NULL,
  `patient_name` VARCHAR(128) NOT NULL,
  `patient_gender` VARCHAR(20) NULL,
  `data_files_base_directory` VARCHAR(128) NULL,
  `camera_id` INT NULL,
  `room_number` VARCHAR(20) NULL,
  `room_type` VARCHAR(20) NULL,
  PRIMARY KEY (`patient_id`),
  UNIQUE INDEX `va_patient_id_UNIQUE` (`va_patient_id` ASC))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`patient_admission`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`patient_admission` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`patient_admission` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `admission_date` DATETIME NOT NULL,
  `admission_note` VARCHAR(2000) NULL,
  `patient_is_diabetic` VARCHAR(20) NULL,
  `factors_impairing_healing` VARCHAR(512) NULL,
  PRIMARY KEY (`id`),
  INDEX `p_a_id_date` (`id` ASC, `admission_date` ASC),
  CONSTRAINT `patient_admission_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE CASCADE
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`patient_assessment`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`patient_assessment` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`patient_assessment` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `assessment_date` DATETIME NOT NULL,
  `assessment_note` VARCHAR(2000) NULL,
  PRIMARY KEY (`id`),
  INDEX `p_as_id_date` (`id` ASC, `assessment_date` ASC),
  CONSTRAINT `patient_assessment_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`braden_scores`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`braden_scores` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`braden_scores` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `braden_scoring_date` DATETIME NOT NULL,
  `sensory_perception_score` INT NOT NULL,
  `moisture_score` INT NOT NULL,
  `activity_score` INT NOT NULL,
  `mobility_score` INT NOT NULL,
  `nutrition_score` INT NOT NULL,
  `friction_shear_score` INT NOT NULL,
  PRIMARY KEY (`id`),
  INDEX `braden_id_date` (`id` ASC, `braden_scoring_date` ASC),
  CONSTRAINT `braden_scores_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`nutritional_status`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`nutritional_status` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`nutritional_status` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `assessment_date` DATETIME NOT NULL,
  `nutritional_notes` VARCHAR(512) NULL,
  INDEX `n_s_id_date` (`id` ASC, `assessment_date` ASC),
  PRIMARY KEY (`id`),
  CONSTRAINT `nutritional_status_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`treatment_plan`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`treatment_plan` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`treatment_plan` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `plan_date` DATETIME NOT NULL,
  `plan_notes` VARCHAR(2000) NULL,
  PRIMARY KEY (`id`),
  INDEX `tp_id_date` (`id` ASC, `plan_date` ASC),
  CONSTRAINT `treatment_plan_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`skeleton_model`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`skeleton_model` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`skeleton_model` (
  `location_id` INT NOT NULL,
  `body_part_code` CHAR NOT NULL,
  `description` VARCHAR(128) NOT NULL,
  PRIMARY KEY (`location_id`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`wound_assessment_map`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`wound_assessment_map` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`wound_assessment_map` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `wound_id` INT NOT NULL,
  `wound_location_id` INT NOT NULL,
  `wound_location_description` VARCHAR(128) NULL,
  INDEX `wound_assessment_fk_sm_idx` (`wound_location_id` ASC),
  PRIMARY KEY (`id`),
  INDEX `wa_map_idx_wound_id` (`wound_id` ASC),
  CONSTRAINT `wound_assessment_map_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `wound_assessment_map_fk_sm`
    FOREIGN KEY (`wound_location_id`)
    REFERENCES `VA_PUPC`.`skeleton_model` (`location_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`clinical_wound_assessment`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`clinical_wound_assessment` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`clinical_wound_assessment` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `wound_id` INT NOT NULL,
  `assessment_id` INT NOT NULL,
  `assessment_date` DATETIME NOT NULL,
  `length` DOUBLE NOT NULL,
  `width` DOUBLE NOT NULL,
  `depth` DOUBLE NOT NULL,
  `undermining_depth` DOUBLE NOT NULL DEFAULT '0',
  `undermining_description` VARCHAR(128) NULL,
  `ulcer_stage` INT NOT NULL,
  `bed_color` VARCHAR(45) NULL,
  `exudate_amount` VARCHAR(45) NULL,
  `exudate_type` VARCHAR(45) NULL,
  `granulation_percentage` DOUBLE NULL,
  `slough_percentage` DOUBLE NULL,
  `eschar_percentage` DOUBLE NULL,
  `bone_percentage` DOUBLE NULL DEFAULT NULL,
  `peri_ulcer_area_description` VARCHAR(128) NULL,
  `blanching_exists` TINYINT(1) NULL,
  `infection_process` VARCHAR(128) NULL,
  `odor_intensity` VARCHAR(20) NULL,
  `odor_description` VARCHAR(128) NULL,
  PRIMARY KEY (`id`),
  INDEX `clinical_wound_fk_patient_idx` (`patient_id` ASC),
  INDEX `clinical_wound_fk_wound_idx` (`wound_id` ASC),
  CONSTRAINT `clinical_wound_fk_patient`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `clinical_wound_fk_wound`
    FOREIGN KEY (`wound_id`)
    REFERENCES `VA_PUPC`.`wound_assessment_map` (`wound_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`prevention_clinical_data`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`prevention_clinical_data` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`prevention_clinical_data` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `clinical_rounding_time` DATETIME NOT NULL,
  `repositioning_flag` TINYINT(1) NOT NULL,
  `final_position` VARCHAR(32) NOT NULL,
  `repositioning_description` VARCHAR(256) NULL,
  INDEX `prevention_clinical_data_fk_pi_idx` (`patient_id` ASC),
  PRIMARY KEY (`id`),
  CONSTRAINT `prevention_clinical_data_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE CASCADE
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_assessment_session`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_assessment_session` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_assessment_session` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `wound_id` INT NOT NULL,
  `assessment_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `collection_status_visual` VARCHAR(80) NOT NULL DEFAULT 'Not collected',
  `collection_status_chemical` VARCHAR(80) NOT NULL DEFAULT 'Not collected',
  `collection_status_chemical_bl` VARCHAR(80) NOT NULL DEFAULT 'Not collected',
  `collection_status_multi_spectral` VARCHAR(80) NOT NULL DEFAULT 'Not collected',
  PRIMARY KEY (`id`),
  INDEX `sa_session_patient_idx` (`patient_id` ASC),
  CONSTRAINT `sa_session_patient`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`algorithm`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`algorithm` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`algorithm` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `algorithm_name` VARCHAR(64) NOT NULL,
  PRIMARY KEY (`id`),
  UNIQUE INDEX `algorithm_name_UNIQUE` (`algorithm_name` ASC))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`experiment`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`experiment` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`experiment` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `experiment_name` VARCHAR(128) NOT NULL,
  `algorithm_id` INT NOT NULL,
  `default_flag` TINYINT(1) NOT NULL DEFAULT 0,
  `experiment_description` VARCHAR(256) NULL,
  PRIMARY KEY (`id`),
  UNIQUE INDEX `experiment_name_UNIQUE` (`experiment_name` ASC),
  INDEX `experiment_fk_algor_idx` (`algorithm_id` ASC),
  CONSTRAINT `experiment_fk_algor`
    FOREIGN KEY (`algorithm_id`)
    REFERENCES `VA_PUPC`.`algorithm` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_assessment_experiment_recon`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_assessment_experiment_recon` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_assessment_experiment_recon` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `reconstruction_data_directory` VARCHAR(128) NULL,
  PRIMARY KEY (`id`),
  INDEX `sa_ex_session_idx` (`session_id` ASC),
  INDEX `sa_ex_experiment_idx` (`experiment_id` ASC),
  CONSTRAINT `sa_ex_r_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`system_assessment_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `sa_ex_r_experiment`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`tissue_type_label`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`tissue_type_label` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`tissue_type_label` (
  `label_index` INT NOT NULL,
  `tissue_type` VARCHAR(45) NULL,
  PRIMARY KEY (`label_index`))
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`prevention_session`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`prevention_session` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`prevention_session` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `depth_video_file_directory` VARCHAR(128) NOT NULL,
  `start_time` DATETIME NULL,
  `end_time` DATETIME NULL,
  PRIMARY KEY (`id`),
  CONSTRAINT `prevention_session_fk_pi`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`prevention_experiment`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`prevention_experiment` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`prevention_experiment` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `feature_data_directory` VARCHAR(128) NULL,
  `track_data_directory` VARCHAR(128) NULL,
  `activity_indicator_directory` VARCHAR(128) NULL,
  PRIMARY KEY (`id`),
  INDEX `prev_ex_fk_session_idx` (`session_id` ASC),
  INDEX `prev_ex_fk_ex_idx` (`experiment_id` ASC),
  CONSTRAINT `prev_ex_fk_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`prevention_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `prev_ex_fk_ex`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`patient_turning`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`patient_turning` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`patient_turning` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `patient_id` INT NOT NULL,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `turn_time` DATETIME NOT NULL,
  `provider_present_flag` TINYINT(1) NOT NULL DEFAULT false,
  `final_position` VARCHAR(32) NOT NULL,
  PRIMARY KEY (`id`),
  INDEX `pat_turn_fk_patient_idx` (`patient_id` ASC),
  INDEX `pat_turn_fk_session_idx` (`session_id` ASC),
  INDEX `pat_turn_fk_ex_idx` (`experiment_id` ASC),
  CONSTRAINT `pat_turn_fk_patient`
    FOREIGN KEY (`patient_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `pat_turn_fk_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`prevention_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `pat_turn_fk_ex`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`frame_table`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`frame_table` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`frame_table` (
  `session_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `end_time` DATETIME NOT NULL,
  `frame_id` INT NOT NULL,
  PRIMARY KEY (`session_id`, `start_time`, `end_time`),
  INDEX `frame_table_fk_s_idx` (`frame_id` ASC),
  CONSTRAINT `frame_table_fk_s`
    FOREIGN KEY (`frame_id`)
    REFERENCES `VA_PUPC`.`patient_identification` (`patient_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`acl_role`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`acl_role` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`acl_role` (
  `role_id` INT NOT NULL AUTO_INCREMENT,
  `role_name` TEXT NOT NULL,
  PRIMARY KEY (`role_id`));


-- -----------------------------------------------------
-- Table `VA_PUPC`.`user`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`user` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`user` (
  `user_id` INT NOT NULL AUTO_INCREMENT,
  `user_name` TEXT NOT NULL,
  `user_pw` TEXT NOT NULL,
  `realname` TEXT NOT NULL,
  `status` CHAR(1) NOT NULL DEFAULT 'A',
  `admin` INT NOT NULL DEFAULT '0',
  `add_date` INT NOT NULL DEFAULT '0',
  `challenge_q` INT NOT NULL DEFAULT '1',
  `challenge_ans` TEXT NOT NULL,
  `role_id` INT NOT NULL DEFAULT '0',
  PRIMARY KEY (`user_id`),
  INDEX `user_fk_role_idx` (`role_id` ASC),
  CONSTRAINT `user_fk_role`
    FOREIGN KEY (`role_id`)
    REFERENCES `VA_PUPC`.`acl_role` (`role_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION);


-- -----------------------------------------------------
-- Table `VA_PUPC`.`grants`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`grants` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`grants` (
  `object_id` INT NOT NULL,
  `object_type` TEXT NOT NULL,
  `owner_id` INT NOT NULL,
  `grant_type` TEXT NOT NULL,
  `grantee_id` INT NOT NULL,
  `granted_perm` TEXT NOT NULL,
  INDEX `grants_fk_owner_idx` (`owner_id` ASC),
  CONSTRAINT `grants_fk_owner`
    FOREIGN KEY (`owner_id`)
    REFERENCES `VA_PUPC`.`user` (`user_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION);


-- -----------------------------------------------------
-- Table `VA_PUPC`.`acl_operation`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`acl_operation` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`acl_operation` (
  `operation_id` INT NOT NULL AUTO_INCREMENT,
  `operation_name` TEXT NOT NULL,
  `operation_desc` TEXT NOT NULL,
  `display_order` INT NOT NULL,
  PRIMARY KEY (`operation_id`));


-- -----------------------------------------------------
-- Table `VA_PUPC`.`acl_role_mgr`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`acl_role_mgr` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`acl_role_mgr` (
  `manager_id` INT NOT NULL,
  `role_id` INT NOT NULL,
  INDEX `acl_role_mgr_fk_1_idx` (`manager_id` ASC),
  INDEX `acl_role_mgr_fk_2_idx` (`role_id` ASC),
  CONSTRAINT `acl_role_mgr_fk_1`
    FOREIGN KEY (`manager_id`)
    REFERENCES `VA_PUPC`.`acl_role` (`role_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `acl_role_mgr_fk_2`
    FOREIGN KEY (`role_id`)
    REFERENCES `VA_PUPC`.`acl_role` (`role_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION);


-- -----------------------------------------------------
-- Table `VA_PUPC`.`acl`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`acl` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`acl` (
  `operation_id` INT NOT NULL,
  `role_id` INT NOT NULL,
  INDEX `acl_fk_operation_idx` (`operation_id` ASC),
  INDEX `acl_fk_role_idx` (`role_id` ASC),
  CONSTRAINT `acl_fk_operation`
    FOREIGN KEY (`operation_id`)
    REFERENCES `VA_PUPC`.`acl_operation` (`operation_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `acl_fk_role`
    FOREIGN KEY (`role_id`)
    REFERENCES `VA_PUPC`.`acl_role` (`role_id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION);


-- -----------------------------------------------------
-- Table `VA_PUPC`.`experiment_configuration`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`experiment_configuration` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`experiment_configuration` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `experiment_id` INT NOT NULL,
  `parameter_name` VARCHAR(128) NOT NULL,
  `parameter_value` VARCHAR(128) NULL,
  PRIMARY KEY (`id`),
  INDEX `exconfig_fk_ex_idx` (`experiment_id` ASC),
  CONSTRAINT `exconfig_fk_ex`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE CASCADE
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`algorithm_defaults`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`algorithm_defaults` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`algorithm_defaults` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `algorithm_id` INT NOT NULL,
  `parameter_name` VARCHAR(128) NOT NULL,
  `default_value` VARCHAR(128) NULL,
  PRIMARY KEY (`id`),
  INDEX `algor_def_fk_algor_idx` (`algorithm_id` ASC),
  CONSTRAINT `algor_def_fk_algor`
    FOREIGN KEY (`algorithm_id`)
    REFERENCES `VA_PUPC`.`algorithm` (`id`)
    ON DELETE CASCADE
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_assessment_experiment_measure`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_assessment_experiment_measure` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_assessment_experiment_measure` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `length` DOUBLE NULL,
  `width` DOUBLE NULL,
  `depth` DOUBLE NULL,
  `length_x_width` DOUBLE NULL,
  `surface_area` DOUBLE NULL,
  `wound_volume` DOUBLE NULL,
  `push_score` DOUBLE NULL,
  PRIMARY KEY (`id`),
  INDEX `sa_ex_session_idx` (`session_id` ASC),
  INDEX `sa_ex_experiment_idx` (`experiment_id` ASC),
  CONSTRAINT `sa_ex_m_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`system_assessment_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `sa_ex_m_experiment`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_assessment_experiment_segment`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_assessment_experiment_segment` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_assessment_experiment_segment` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `image_label_map_file` VARCHAR(128) NULL,
  `granulation_percentage` DOUBLE NULL,
  `slough_percentage` DOUBLE NULL,
  `eschar_percentage` DOUBLE NULL,
  `bone_percentage` DOUBLE NULL,
  `ulcer_stage` INT NULL,
  PRIMARY KEY (`id`),
  INDEX `sa_ex_session_idx` (`session_id` ASC),
  INDEX `sa_ex_experiment_idx` (`experiment_id` ASC),
  CONSTRAINT `sa_ex_s_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`system_assessment_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `sa_ex_s_experiment`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_assessment_experiment_temperature`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_assessment_experiment_temperature` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_assessment_experiment_temperature` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `session_id` INT NOT NULL,
  `experiment_id` INT NOT NULL,
  `start_time` DATETIME NOT NULL,
  `max_temperature` DOUBLE NULL,
  `max_temperature_loc_x` DOUBLE NULL,
  `max_temperature_loc_y` DOUBLE NULL,
  `min_temperature` DOUBLE NULL,
  `min_temperature_loc_x` DOUBLE NULL,
  `min_temperature_loc_y` DOUBLE NULL,
  `baseline_temperature` DOUBLE NULL,
  `baseline_description` VARCHAR(128) NULL,
  `temperature_variation_sigma` DOUBLE NULL,
  `temperature_segment_1_percentage` DOUBLE NULL,
  `temperature_segment_2_percentage` DOUBLE NULL,
  `temperature_segment_3_percentage` DOUBLE NULL,
  `temperature_segment_4_percentage` DOUBLE NULL,
  `temperature_segment_5_percentage` DOUBLE NULL,
  PRIMARY KEY (`id`),
  INDEX `sa_ex_session_idx` (`session_id` ASC),
  INDEX `sa_ex_experiment_idx` (`experiment_id` ASC),
  CONSTRAINT `sa_ex_t_session`
    FOREIGN KEY (`session_id`)
    REFERENCES `VA_PUPC`.`system_assessment_session` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION,
  CONSTRAINT `sa_ex_t_experiment`
    FOREIGN KEY (`experiment_id`)
    REFERENCES `VA_PUPC`.`experiment` (`id`)
    ON DELETE NO ACTION
    ON UPDATE NO ACTION)
ENGINE = InnoDB;


-- -----------------------------------------------------
-- Table `VA_PUPC`.`system_configuration`
-- -----------------------------------------------------
DROP TABLE IF EXISTS `VA_PUPC`.`system_configuration` ;

CREATE TABLE IF NOT EXISTS `VA_PUPC`.`system_configuration` (
  `id` INT NOT NULL AUTO_INCREMENT,
  `parameter_name` VARCHAR(128) NOT NULL,
  `parameter_value` VARCHAR(128) NULL,
  PRIMARY KEY (`id`))
ENGINE = InnoDB;


SET SQL_MODE=@OLD_SQL_MODE;
SET FOREIGN_KEY_CHECKS=@OLD_FOREIGN_KEY_CHECKS;
SET UNIQUE_CHECKS=@OLD_UNIQUE_CHECKS;
