// pu_bayspec_64bit_dll_exp02.cpp : Defines the entry point for the console application.
//
#ifndef _SECURE_ATL
#define _SECURE_ATL 1
#endif

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // Exclude rarely-used stuff from Windows headers
#endif

//#include "targetver.h"
#include <SDKDDKVer.h>

#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS      // some CString constructors will be explicit

// turns off MFC's hiding of some common and often safely ignored warning messages
#define _AFX_ALL_WARNINGS

#include <stdio.h>
#include <tchar.h>

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN            // Exclude rarely-used stuff from Windows headers
#endif

#include <afx.h>
#include <afxwin.h>         // MFC core and standard components
#include <afxext.h>         // MFC extensions
#ifndef _AFX_NO_OLE_SUPPORT
#include <afxdtctl.h>           // MFC support for Internet Explorer 4 Common Controls
#endif
#ifndef _AFX_NO_AFXCMN_SUPPORT
#include <afxcmn.h>                     // MFC support for Windows Common Controls
#endif // _AFX_NO_AFXCMN_SUPPORT

#include <iostream>

#include "GigEBaySpec.h"

#include <vil/vil_save.h>
#include <vil/vil_load.h>

#include <vcl_string.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#undef UNICODE

#define WIN32_LEAN_AND_MEAN

int main(int argc, char* argv[])
{
  int nRetCode = 0;

  //////////////////////////////////////////////////////////////////////////////
  // connect to the camera
  CString lstr_;
  int pixel_format_ = 0;
  int width_, height_;
  float exposure_time_;
  float fps_ = 50;
  CString save_folder_ = _T("D:\\yuting\\data\\pressure_ulcer\\hyper_spectral\\poc");
  int scanning_mode_ = 0; // raw images

  if(DLL_ConnectCamera(&lstr_)) {

    DLL_CreatePreview(NULL);

    DLL_SetPixelFormat(pixel_format_);

    DLL_SetAcquisitionMode(0);

    DLL_DisableDisplay(true);

    DLL_GetImageResolution(&width_, &height_);

    DLL_GetPixelFormat(&pixel_format_);

    double fxt;
    if (DLL_GetExposureTime(&fxt)) {
      exposure_time_ = (float)fxt*0.001;
    }

    // start play function
    CString WhiteRef = _T("");
    CString Dark = _T("");

    DLL_GetWhiteRef(&WhiteRef);
    DLL_GetDarkBkg(&Dark);

    if(WhiteRef == _T("") || Dark == _T("")) {
      //AfxMessageBox(_T("Please record white reference and dark background first!"));
      //return nRetCode;
    }

    if(!DLL_IsSensorFileExists()) {
      //AfxMessageBox(_T("Sensor calibration file is missing! Please copy this file to the folder where current exe is running."));
      //return nRetCode;				
    }

    DLL_DisableDisplay(true);

    fxt = (double)(exposure_time_*1000);
    if (fxt>1) {
      DLL_SetExposureTime(fxt);
    }

    DLL_SetImageSavingFPS(fps_);

    DLL_EnableImageSave(true);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // start the data acquisition for white reference
    DLL_StopAcquisition();
    DLL_SetAcquisitionMode(0);

    scanning_mode_ = 1; // white reference
    DLL_CreateFileSystem(save_folder_, exposure_time_, scanning_mode_);
    DLL_StopAcquisition();
    DLL_StartAcquisition();
    Sleep(5000);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // start the data acquisition for black reference
    DLL_StopAcquisition();
    DLL_SetAcquisitionMode(0);

    scanning_mode_ = 2; // black reference
    DLL_CreateFileSystem(save_folder_, exposure_time_, scanning_mode_);
    DLL_StopAcquisition();
    DLL_StartAcquisition();
    Sleep(5000);

    ////////////////////////////////////////////////////////////////////////////////////////////////////
    // start the data acquisition for raw images
    DLL_StopAcquisition();
    DLL_SetAcquisitionMode(0);

    scanning_mode_ = 0; // raw images
    DLL_CreateFileSystem(save_folder_, exposure_time_, scanning_mode_);
    DLL_StopAcquisition();
    DLL_StartAcquisition();
    Sleep(5000);

    vil_image_view<vxl_byte> img;
    vcl_string filename;
    vil_save(img, filename.c_str());

    // stop play
    DLL_StopAcquisition();
    DLL_SetAcquisitionMode(0);
  }

  DLL_DisconnectCamera();

  return nRetCode;
}