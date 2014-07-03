// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 3/24/2014
/// \par Modifications:
/// - Original version (file created with VS macro)

#ifndef gevxl_pressure_ulcer_pu_assessment_segment_database_entry_h
#define gevxl_pressure_ulcer_pu_assessment_segment_database_entry_h

// Put includes here.
#include <database/identity_field.h>
#include <database/command.h>
#include <database/date_time.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace assessment {
			namespace database {

class segment_database_entry
{
public:
  segment_database_entry(void);

	virtual ~segment_database_entry(void) { }

  void set_id(gevxl::database::identity_field id) { id_ = id; }
  gevxl::database::identity_field get_id(void) const { return id_; }

  void set_session_id(gevxl::database::identity_field session_id) { session_id_ = session_id; }
  gevxl::database::identity_field get_session_id(void) const { return session_id_; }

  void set_experiment_id(gevxl::database::identity_field experiment_id) { experiment_id_ = experiment_id; }
  gevxl::database::identity_field get_experiment_id(void) const { return experiment_id_; }

  void set_start_time(const gevxl::database::date_time &dt) { start_time_ = dt; }
  const gevxl::database::date_time &get_start_time(void) const { return start_time_; }

  void set_image_label_map_filename(const vcl_string &filename) { image_label_map_filename_ = filename; }
  const vcl_string &get_image_label_map_filename(void) const { return image_label_map_filename_; }

  void set_granulation_percentage(double val) { granulation_percentage_ = val; }
  double get_granulation_percentage(void) const { return granulation_percentage_; }

	void set_slough_percentage(double val) { slough_percentage_ = val; }
  double get_slough_percentage(void) const { return slough_percentage_; }

	void set_eschar_percentage(double val) { eschar_percentage_ = val; }
  double get_eschar_percentage(void) const { return eschar_percentage_; }

  bool write_to_cmd(gevxl::database::command &cmd) const;
  bool read_from_cmd(gevxl::database::command &cmd);

protected:

  gevxl::database::identity_field id_;

	gevxl::database::identity_field session_id_;

	gevxl::database::identity_field experiment_id_;

  gevxl::database::date_time start_time_;

  vcl_string image_label_map_filename_;

	double granulation_percentage_;

	double slough_percentage_;

	double eschar_percentage_;
};

}}}} // end namespaces

#endif
