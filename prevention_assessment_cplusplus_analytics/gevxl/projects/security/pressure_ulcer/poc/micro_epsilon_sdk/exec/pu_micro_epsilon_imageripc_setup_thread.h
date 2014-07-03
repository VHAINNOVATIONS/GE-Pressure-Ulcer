// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

/// \file
/// \author Ting Yu
/// \date 1/22/2014
/// \brief the inter process communication (ipc) setup for the Micro-Epsilon camera
/// - Original version

#ifndef micro_epsilon_imageripc_setup_thread_h_
#define micro_epsilon_imageripc_setup_thread_h_

#include <threading/thread.h>
#include <threading/mutex.h>

#include <windows.h>
#include "ImagerIPC2.h"

namespace gevxl { 
namespace vid {

class micro_epsilon_imageripc_setup_thread : public gevxl::threading::thread
{
public:

  // constructor
  micro_epsilon_imageripc_setup_thread();
  // deconstructor
  ~micro_epsilon_imageripc_setup_thread();

  // function pointers to be set
  void set_fpOnServerStopped(fpOnServerStopped fp);
  void set_fpOnFrameInit(fpOnFrameInit fp);
  void set_fpOnNewFrame(fpOnNewFrame fp);
  void set_fpOnNewFrameEx(fpOnNewFrameEx fp);
  void set_fpOnInitCompleted(fpOnInitCompleted fp);
  void set_fpOnConfigChanged(fpOnConfigChanged fp);
  void set_fpOnFileCommandReady(fpOnFileCommandReady fp);
  
private:

  // run thread
  void run_thread(void);

  bool init_ipc ();
  bool uninit_ipc ();

  // function pointers to be set
  fpOnServerStopped fpOnServerStopped_;
  fpOnFrameInit fpOnFrameInit_;
  fpOnNewFrame fpOnNewFrame_;
  fpOnNewFrameEx fpOnNewFrameEx_;
  fpOnInitCompleted fpOnInitCompleted_;
  fpOnConfigChanged fpOnConfigChanged_;
  fpOnFileCommandReady fpOnFileCommandReady_;

  // DLL handle
  HINSTANCE dllHandle_;
  bool ipcInitialized_;
};

}} // end namespaces

#endif
