// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_assessment_database_writer_h
#define gevxl_pressure_ulcer_pu_assessment_database_writer_h

// Put includes here.
#include <database/configuration.h>
#include <database/connection.h>
#include <database/command.h>

#include "segment_database_entry.h"
#include "segment_database_table.h"

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {
			namespace database {

class assessment_database_writer
{
public:

  assessment_database_writer(char const *name = "assessment_database_writer");

  virtual ~assessment_database_writer();

  /// Configure this process.
  virtual bool configure(gevxl::util::config_file &config);

  /// Connect to the database.
  bool connect(void);

  /// Disconnect from the database.
  void disconnect(void);

	/// Insert a new segment_database_entry into the segment_database_table
	int insert_segment_database_entry(int session_id, int experiment_id, const vcl_string &image_label_map_filename, 
																		 double granulation_percentage, double slough_percentage, double eschar_percentage);

	/// Update a segment_database_entry in the segment_database_table 
	int update_segment_database_entry(int id, int session_id, int experiment_id, const vcl_string &image_label_map_filename, 
																		 double granulation_percentage, double slough_percentage, double eschar_percentage);

private:

  /// Name of this writer.
  vcl_string name_;

  /// Is this archiving process enabled?
  bool enabled_;

  /// Is this process connected to the database?
  bool connected_;

  /// Configuration of database.
  gevxl::database::configuration db_config_;

  /// Database connection and command objects.
  gevxl::database::connection_sptr con_;
  gevxl::database::command_sptr cmd_;

  /// Table that stores information about the assessment system's segment_database_table.
	gevxl::pressure_ulcer::assessment::database::segment_database_table segment_table_;

  /// Entry that stores information about the assessment system's segment_database_entry.
  gevxl::pressure_ulcer::assessment::database::segment_database_entry segment_entry_;
};

typedef vbl_shared_pointer<assessment_database_writer> assessment_database_writer_sptr;

}}}} // end namespaces

#endif
