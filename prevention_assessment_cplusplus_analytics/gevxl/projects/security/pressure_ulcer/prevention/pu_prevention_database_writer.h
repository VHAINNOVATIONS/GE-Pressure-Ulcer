// Copyright (C) 2014 General Electric Company
// 
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 06/15/2014
/// \par Modifications:

#ifndef gevxl_pressure_ulcer_pu_prevention_database_writer_h_
#define gevxl_pressure_ulcer_pu_prevention_database_writer_h_

// the abstract class for the pu_prevention_database_writer, the concrete implementation is delegated to python side.
#include <vcl_string.h>

namespace gevxl {
	namespace pressure_ulcer {
		namespace prevention {
  
			class pu_prevention_database_writer
																												
			{
			public:
        
        // constructor
        pu_prevention_database_writer(void) { }

			  // destructor
        virtual ~pu_prevention_database_writer(void) { } 
    
        // write function
        virtual void write(const bool provider_present_flag, const vcl_string final_position) = 0;

			};  // end of class pu_prevention_database_writer

		} // end namespace prevention
  } //end namespace pressure_ulcer
} //end namespace gevxl

#endif
