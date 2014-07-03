// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_assessment_segment_database_table_h
#define gevxl_pressure_ulcer_pu_assessment_segment_database_table_h

// Put includes here.
#include <database/identity_field.h>
#include <database/data_table.h>

#include "segment_database_entry.h"

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {
			namespace database {

class segment_database_table : 
	public gevxl::database::data_table<gevxl::pressure_ulcer::assessment::database::segment_database_entry>
{
public:

  segment_database_table(void);

	virtual ~segment_database_table(void) { }

  void set_command(gevxl::database::command_sptr cmd);

  void set_find_condition(const vcl_string &condition);

protected:

  /// Raw find statement.
  vcl_string find_statement_;

  /// Does the table exists?
  bool table_exists_;
};

}}}} // end namespaces

#endif
