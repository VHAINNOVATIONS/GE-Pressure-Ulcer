// exp01.cpp : Defines the entry point for the console application.
//

//#include "stdafx.h"
#include "util_render.h"
#include "util_pipeline.h"

int main (int argc, char **argv)
{

  UtilPipeline pp;
  pp.EnableImage(PXCImage::COLOR_FORMAT_RGB32, 1280, 720);
  pp.EnableImage(PXCImage::COLOR_FORMAT_DEPTH);
  pp.Init();
  UtilRender color_render(L"Color Stream");
  UtilRender depth_render(L"Depth Stream");
  for(;;) {
    if(!pp.AcquireFrame(true)) break;
    PXCImage *color_image=pp.QueryImage(PXCImage::IMAGE_TYPE_COLOR);
    PXCImage *depth_image=pp.QueryImage(PXCImage::IMAGE_TYPE_DEPTH);
    if(!color_render.RenderFrame(color_image)) break;
    if(!depth_render.RenderFrame(depth_image)) break;
    pp.ReleaseFrame();
  }
  pp.Close();
	return 0;
}

