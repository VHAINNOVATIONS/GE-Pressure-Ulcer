// Copyright (C) 2014 General Electric Company
//
// This software is intellectual property of General Electric Co.
// and may not be copied or redistributed without express written consent.

#include <vcl_cstdio.h>
#include <vcl_iostream.h>
#include "pu_micro_epsilon_imageripc_setup_thread.h"
#include <threading/sleep.h>
#include <vcl_string.h>

using namespace gevxl::vid;

micro_epsilon_imageripc_setup_thread::micro_epsilon_imageripc_setup_thread()
{
  fpOnServerStopped_ = NULL;
  fpOnFrameInit_ = NULL;
  fpOnNewFrame_ = NULL;
  fpOnNewFrameEx_ = NULL;
  fpOnInitCompleted_ = NULL;
  fpOnConfigChanged_ = NULL;
  fpOnFileCommandReady_ = NULL;

  dllHandle_ = NULL;
  ipcInitialized_ = false;
}

micro_epsilon_imageripc_setup_thread::~micro_epsilon_imageripc_setup_thread()
{
  set_time_to_stop_flag();
	wait_until_stopped();
  
  HRESULT hr;
  //bool ipcInitialized = false;

  bool ret = uninit_ipc();
}

void micro_epsilon_imageripc_setup_thread::run_thread(void)
{
  int count = 0;
  while (!get_time_to_stop_flag()) {

    if (ipcInitialized_ == false) {
      //Try ipcInitialization
      ipcInitialized_ = init_ipc();
      vcl_printf ("setup_thread count%d ipcInit=%d\n", count, ipcInitialized_);      
    }
    else {
      float fixed_emmisivity = 0.0;
      typedef float (WINAPI *fpGetFixedEmissivity)(WORD index);
      fpGetFixedEmissivity fpGetFixedEmissivity_ = reinterpret_cast<fpGetFixedEmissivity>(::GetProcAddress(dllHandle_, "GetFixedEmissivity"));
      fixed_emmisivity = fpGetFixedEmissivity_(0);
      
      float fixed_emmisivity1 = 0.0;
      typedef float (WINAPI *fpSetFixedEmissivity)(WORD index, float Value);
      fpSetFixedEmissivity fpSetFixedEmissivity_ = reinterpret_cast<fpSetFixedEmissivity>(::GetProcAddress(dllHandle_, "SetFixedEmissivity"));
      fixed_emmisivity1 = fpSetFixedEmissivity_(0, 0.99);
      
      float fixed_emmisivity2 = 0.0;    
      fixed_emmisivity2 = fpGetFixedEmissivity_(0);

      break;

      typedef float (WINAPI *fpGetTempHousing)(WORD index);
      fpGetTempHousing fpGetTempHousing_ = reinterpret_cast<fpGetTempHousing>(::GetProcAddress(dllHandle_, "GetTempHousing"));  
      float temp_housing = fpGetTempHousing_(0);
      
      vcl_cout << "temp_housing = " << temp_housing << vcl_endl;
    }

    count++;
    gevxl::threading::sleep (20);
  }

  return;
}

bool micro_epsilon_imageripc_setup_thread::init_ipc()
{
  HRESULT hr;
  WORD ImagerCount = 1;
  
  //vcl_string dllFilepath = "D:\\yuting\\projects_cpp\\micro_epsilon\\visual_studio_2008_32bit_exp01\\exp01\\lib\\ImagerIPC2.dll";
  //vcl_string dllFilepath = "D:\\GEVXL\\gevxl_build_vc80x32\\bin\\Debug\\ImagerIPC2.dll";
  vcl_string dllFilepath = "ImagerIPC2.dll";
  
  LPCSTR dllFilepathFPCSTR = dllFilepath.c_str();			
  dllHandle_ = ::LoadLibrary(dllFilepathFPCSTR);  
  
  typedef HRESULT (WINAPI *fpSetLogFile)(wchar_t *LogFilename, int LogLevel, bool Append);
  fpSetLogFile fpSetLogFile_ = reinterpret_cast<fpSetLogFile>(::GetProcAddress(dllHandle_, "SetLogFile"));
  hr = fpSetLogFile_(L".\\IPC Sample.log", 1, false);
	if(FAILED(hr)) {
    //ipcInitialized = true;
    ///vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetLogFile_ returns " << hr << vcl_endl;
    //return -1;
  }

  typedef HRESULT (WINAPI *fpSetImagerIPCCount)(WORD count);
  fpSetImagerIPCCount fpSetImagerIPCCount_ = reinterpret_cast<fpSetImagerIPCCount>(::GetProcAddress(dllHandle_, "SetImagerIPCCount"));  
  hr = fpSetImagerIPCCount_(ImagerCount);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    ///vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetImagerIPCCount_ returns " << hr << vcl_endl;
    //return -1;
  }

  typedef HRESULT (WINAPI *fpInitImagerIPC)(WORD index);
  fpInitImagerIPC fpInitImagerIPC_ = reinterpret_cast<fpInitImagerIPC>(::GetProcAddress(dllHandle_, "InitImagerIPC"));  
  hr = fpInitImagerIPC_(0);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "fpInitImagerIPC returns " << hr << vcl_endl;
    return false;
  }
  
  /*typedef HRESULT (WINAPI *fpInitNamedImagerIPC)(WORD index, wchar_t *InstanceName);
  fpInitNamedImagerIPC fpInitNamedImagerIPC_ = reinterpret_cast<fpInitNamedImagerIPC>(::GetProcAddress(dllHandle_, "InitNamedImagerIPC"));  
  wchar_t szInstanceName[128] = L"";
  hr = fpInitNamedImagerIPC_(0, szInstanceName);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpInitNamedImagerIPC_ returns " << hr << vcl_endl;
    //return -1;
  }*/

  typedef HRESULT (WINAPI *fpSetCallback_OnServerStopped)(WORD index, fpOnServerStopped OnServerStopped);
  fpSetCallback_OnServerStopped fpSetCallback_OnServerStopped_ = reinterpret_cast<fpSetCallback_OnServerStopped>(::GetProcAddress(dllHandle_, "SetCallback_OnServerStopped"));  
  hr = fpSetCallback_OnServerStopped_(0, fpOnServerStopped_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnServerStopped_ returns " << hr << vcl_endl;
    return false;
  }

  typedef HRESULT (WINAPI *fpSetCallback_OnFrameInit)(WORD index, fpOnFrameInit OnFrameInit);
  fpSetCallback_OnFrameInit fpSetCallback_OnFrameInit_ = reinterpret_cast<fpSetCallback_OnFrameInit>(::GetProcAddress(dllHandle_, "SetCallback_OnFrameInit"));  
  hr = fpSetCallback_OnFrameInit_(0, fpOnFrameInit_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnFrameInit_ returns " << hr << vcl_endl;
    return false;
  }

  typedef HRESULT (WINAPI *fpSetCallback_OnNewFrame)(WORD index, fpOnNewFrame OnNewFrame);
  fpSetCallback_OnNewFrame fpSetCallback_OnNewFrame_ = reinterpret_cast<fpSetCallback_OnNewFrame>(::GetProcAddress(dllHandle_, "SetCallback_OnNewFrame"));  
  hr = fpSetCallback_OnNewFrame_(0, fpOnNewFrame_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnNewFrame returns " << hr << vcl_endl;
    return false;
  }

  typedef HRESULT (WINAPI *fpSetCallback_OnConfigChanged)(WORD index, fpOnConfigChanged OnConfigChanged);
  fpSetCallback_OnConfigChanged fpSetCallback_OnConfigChanged_ = reinterpret_cast<fpSetCallback_OnConfigChanged>(::GetProcAddress(dllHandle_, "SetCallback_OnConfigChanged"));
  hr = fpSetCallback_OnConfigChanged_(0, fpOnConfigChanged_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnConfigChanged returns " << hr << vcl_endl;
    return false;
  }

  typedef HRESULT (WINAPI *fpSetCallback_OnInitCompleted)(WORD index, fpOnInitCompleted OnInitCompleted);
  fpSetCallback_OnInitCompleted fpSetCallback_OnInitCompleted_ = reinterpret_cast<fpSetCallback_OnInitCompleted>(::GetProcAddress(dllHandle_, "SetCallback_OnInitCompleted"));
  hr = fpSetCallback_OnInitCompleted_(0, fpOnInitCompleted_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnInitCompleted returns " << hr << vcl_endl;
    return false;
  }

  typedef HRESULT (WINAPI *fpSetCallback_OnFileCommandReady)(WORD index, fpOnFileCommandReady OnFileCommandReady);
  fpSetCallback_OnFileCommandReady fpSetCallback_OnFileCommandReady_ = reinterpret_cast<fpSetCallback_OnFileCommandReady>(::GetProcAddress(dllHandle_, "SetCallback_OnFileCommandReady"));
  hr = fpSetCallback_OnFileCommandReady_(0, fpOnFileCommandReady_);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpSetCallback_OnFileCommandReady_ returns " << hr << vcl_endl;
    return false;
  }

  // this needs to be placed into a separate thread
  typedef HRESULT (WINAPI *fpRunImagerIPC)(WORD index);
  fpRunImagerIPC fpRunImagerIPC_ = reinterpret_cast<fpRunImagerIPC>(::GetProcAddress(dllHandle_, "RunImagerIPC"));
  hr = fpRunImagerIPC_(0);
  if(FAILED(hr)) {
    //ipcInitialized = true;
    vcl_cout << "In micro_epsilon_imageripc_setup_thread::run_thread(): fpRunImagerIPC_ returns " << hr << vcl_endl;
    return false;
  }

  ipcInitialized_ = SUCCEEDED(hr);
  return true;
}

bool micro_epsilon_imageripc_setup_thread::uninit_ipc ()
{
  if (ipcInitialized_) {
    typedef HRESULT (WINAPI *fpReleaseImagerIPC)(WORD index);
    fpReleaseImagerIPC fpReleaseImagerIPC_ = reinterpret_cast<fpReleaseImagerIPC>(::GetProcAddress(dllHandle_, "ReleaseImagerIPC"));
    HRESULT hr = fpReleaseImagerIPC_(0);
    if(FAILED(hr)) {
      //ipcInitialized = true;
      vcl_cout << "In ~micro_epsilon_imageripc_setup_thread(): fpReleaseImagerIPC_ returns " << hr << vcl_endl;
      return false;
    }
  }
  return true;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnServerStopped(fpOnServerStopped fp)
{
  fpOnServerStopped_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnFrameInit(fpOnFrameInit fp)
{
  fpOnFrameInit_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnNewFrame(fpOnNewFrame fp)
{
  fpOnNewFrame_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnNewFrameEx(fpOnNewFrameEx fp)
{
  fpOnNewFrameEx_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnInitCompleted(fpOnInitCompleted fp)
{
  fpOnInitCompleted_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnConfigChanged(fpOnConfigChanged fp)
{
  fpOnConfigChanged_ = fp;
}

void micro_epsilon_imageripc_setup_thread::set_fpOnFileCommandReady(fpOnFileCommandReady fp)
{
  fpOnFileCommandReady_ = fp;
}
