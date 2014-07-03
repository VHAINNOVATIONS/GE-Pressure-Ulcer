// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#include "assessment_database_writer.h"

#include <database/connection_factory.h>
#include <database/command_factory.h>

#include <util/time/global_time.h>
#include <util/string.h>
#include <util/config_file.h>

#include <vcl_algorithm.h>
#include <vul/vul_file.h>

using namespace gevxl::util;
using namespace gevxl::database;
using namespace gevxl::pressure_ulcer::assessment;
using namespace gevxl::pressure_ulcer::assessment::database;

assessment_database_writer::assessment_database_writer(char const *name)
: name_(name),
  enabled_(false),
  connected_(false),
  db_config_(name),
  con_(NULL),
	cmd_(NULL)
{

}

assessment_database_writer::~assessment_database_writer(void)
{
  if(connected_) disconnect();
}

bool assessment_database_writer::configure( gevxl::util::config_file &config )
{
  if(!db_config_.configure(config)) return false;

  config.get_bool(name_+"::enabled", enabled_);

  return true;
}

bool assessment_database_writer::connect(void)
{
  if(!enabled_) return true;
  disconnect();

  if(db_config_.is_enabled()) {
		con_ = gevxl::database::connection_sptr(gevxl::database::connection_factory::create_connection(db_config_.get_api()));
    cmd_ = gevxl::database::command_sptr(gevxl::database::command_factory::create_command(db_config_.get_api()));

    if(!con_ || !cmd_) {
      vcl_cerr << "assessment_database_writer::connect(): Error, failed to create database connection.\n";
      return false;
    }
  }
  else {
    vcl_cerr << "assessment_database_writer::connect(): Error, no API specified.\n";
    return false;
  }

  if(!con_->connect(db_config_.get_name(), db_config_.get_user(), db_config_.get_password(), db_config_.get_aux())) {

    vcl_cerr << "assessment_database_writer::connect(): Error, failed to connect to database.\n";
    vcl_cerr << "  name   = " << db_config_.get_name() << "\n";
    vcl_cerr << "  user   = " << db_config_.get_user() << "\n";
    vcl_cerr << "  passwd = " << db_config_.get_password() << "\n";
    vcl_cerr << "  aux    = " << db_config_.get_aux() << "\n";
    return false;
  }
  else {
    vcl_cout << "assessment_database_writer connected to database.\n";
  }

  cmd_->set_connection(con_);

  connected_ = true;

  return true;
}

void assessment_database_writer::disconnect(void)
{
  if(!enabled_) return;

  if(!connected_) return;

  if(cmd_) {

    segment_table_.clear();
    segment_table_.set_command(gevxl::database::command_sptr());
  }

  if(con_) {
    if(con_->is_connected()) con_->disconnect();
  }

  cmd_ = gevxl::database::command_sptr();
  con_ = gevxl::database::connection_sptr();

  connected_ = false;
}

/// Insert a new segment_database_entry into the segment_database_table
int assessment_database_writer::insert_segment_database_entry(int session_id, int experiment_id, const vcl_string &image_label_map_filename, 
																															double granulation_percentage, double slough_percentage, double eschar_percentage)
{
	if (!enabled_) return true;

  segment_table_.set_command(cmd_);

	segment_entry_.set_session_id((vxl_int_64)session_id);
	segment_entry_.set_experiment_id((vxl_int_64)experiment_id);

	segment_entry_.set_image_label_map_filename(image_label_map_filename);
	
	segment_entry_.set_granulation_percentage(granulation_percentage);
	segment_entry_.set_slough_percentage(slough_percentage);
	segment_entry_.set_eschar_percentage(eschar_percentage);

	segment_entry_.set_start_time(gevxl::util::time::global_time());

	segment_table_.add_entry(segment_entry_);

	return (int)(segment_entry_.get_id());
}

/// Update a segment_database_entry in the segment_database_table 
int assessment_database_writer::update_segment_database_entry(int id, int session_id, int experiment_id, const vcl_string &image_label_map_filename, 
																															double granulation_percentage, double slough_percentage, double eschar_percentage)
{
	if (!enabled_) return true;

  segment_table_.set_command(cmd_);

	segment_entry_.set_session_id((vxl_int_64)session_id);
	segment_entry_.set_experiment_id((vxl_int_64)experiment_id);

	segment_entry_.set_image_label_map_filename(image_label_map_filename);
	
	segment_entry_.set_granulation_percentage(granulation_percentage);
	segment_entry_.set_slough_percentage(slough_percentage);
	segment_entry_.set_eschar_percentage(eschar_percentage);

	segment_entry_.set_start_time(gevxl::util::time::global_time());

	segment_table_.update_entry(segment_entry_);	

	return (int)(segment_entry_.get_id());
}
