// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#include "segment_database_entry.h"

using namespace gevxl::pressure_ulcer::assessment;
using namespace gevxl::pressure_ulcer::assessment::database;

segment_database_entry::segment_database_entry(void)
{
  
}

bool segment_database_entry::write_to_cmd(gevxl::database::command &cmd) const
{
  bool r = true;

	r=r && cmd.set_as_integer_64(session_id_);
  r=r && cmd.set_as_integer_64(experiment_id_);

  r=r && cmd.set_as_date_time(start_time_);
  
	r=r && cmd.set_as_string(image_label_map_filename_);
 
	r=r && cmd.set_as_double(granulation_percentage_);
  r=r && cmd.set_as_double(slough_percentage_);
	r=r && cmd.set_as_double(eschar_percentage_);

  return r;
}

bool segment_database_entry::read_from_cmd(gevxl::database::command &cmd)
{
  id_ = cmd.get_as_integer_64();
  
	session_id_ = cmd.get_as_integer_64();
	experiment_id_ = cmd.get_as_integer_64();
  
  start_time_ = cmd.get_as_date_time();
 
	image_label_map_filename_ = cmd.get_as_string();
  
	granulation_percentage_ = cmd.get_as_double();
	slough_percentage_ = cmd.get_as_double();
	eschar_percentage_ = cmd.get_as_double();
  
  return true;
}
