// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#include "segment_database_table.h"

using namespace gevxl::pressure_ulcer::assessment;
using namespace gevxl::pressure_ulcer::assessment::database;

/*
 Table columns from segment_database_entry:

	gevxl::database::identity_field id_;

	gevxl::database::identity_field session_id_;
	gevxl::database::identity_field experiment_id_;

  gevxl::database::date_time start_time_;

  vcl_string image_label_map_filename_;

	double granulation_percentage_;
	double slough_percentage_;
	double eschar_percentage_;
*/

segment_database_table::segment_database_table(void) 
: table_exists_(false)
{
  set_table_name("system_assessment_experiment_segment");

  set_insert_statement("INSERT INTO %s (session_id, experiment_id, start_time, image_label_map_file, granulation_percentage, slough_percentage, eschar_percentage) VALUES (:1, :2, :3, :4, :5, :6, :7)");
  
	find_statement_ = "SELECT id, session_id, experiment_id, start_time, image_label_map_file, granulation_percentage, slough_percentage, eschar_percentage FROM %s";
  set_find_statement(find_statement_);
  
	set_update_statement("UPDATE %s SET session_id=:1, experiment_id=:2, start_time=:3, image_label_map_file=:4, granulation_percentage=:5, slough_percentage=:6, eschar_percentage=:7 WHERE id=:8");
  
  set_delete_statement("DELETE FROM %s WHERE id=:1");

  set_synchronize_interval_in_ms(11000);
}

void segment_database_table::set_find_condition(const vcl_string &condition)
{
  set_find_statement(find_statement_ + " WHERE " + condition);
}

void segment_database_table::set_command(gevxl::database::command_sptr cmd)
{
	gevxl::database::data_table<gevxl::pressure_ulcer::assessment::database::segment_database_entry>::set_command(cmd);

  if(!cmd || table_exists_) return;

  vcl_string sql;

  if(cmd->get_database_type()=="SQLITE") {
		//sql = "CREATE TABLE IF NOT EXISTS " + get_table_name() + " (id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, session_id INTEGER NOT NULL, experiment_id INTEGER NOT NULL, start_time CHAR(32), image_label_map_file VARCHAR(255), granulation_percentage DOUBLE, slough_percentage DOUBLE, eschar_percentage DOUBLE)";
		sql = "CREATE TABLE IF NOT EXISTS " + get_table_name() + " (id INTEGER NOT NULL PRIMARY KEY AUTOINCREMENT, session_id INTEGER NOT NULL, experiment_id INTEGER NOT NULL, start_time DATETIME, image_label_map_file VARCHAR(128), granulation_percentage DOUBLE, slough_percentage DOUBLE, eschar_percentage DOUBLE)";
    if(cmd->set_command(sql) && cmd->execute()) cmd->commit();
  }
  else if(cmd->get_database_type()=="SQLAPI") {
    //sql = "CREATE TABLE IF NOT EXISTS " + get_table_name() + " (id BIGINT NOT NULL PRIMARY KEY AUTO_INCREMENT, session_id BIGINT NOT NULL, experiment_id BIGINT NOT NULL, start_time CHAR(32), image_label_map_file VARCHAR(255), granulation_percentage DOUBLE, slough_percentage DOUBLE, eschar_percentage DOUBLE) ENGINE = MyISAM";
		sql = "CREATE TABLE IF NOT EXISTS " + get_table_name() + " (id BIGINT NOT NULL PRIMARY KEY AUTO_INCREMENT, session_id BIGINT NOT NULL, experiment_id BIGINT NOT NULL, start_time DATETIME, image_label_map_file VARCHAR(128), granulation_percentage DOUBLE, slough_percentage DOUBLE, eschar_percentage DOUBLE) ENGINE = MyISAM";
    if(cmd->set_command(sql) && cmd->execute()) cmd->commit();
  }

  table_exists_ = true;
}
