// exp01.cpp : Defines the entry point for the console application.
//

#include <vcl_cstdio.h>
#include <vcl_string.h>
#include "pu_micro_epsilon_imageripc_camera.h"
#include <threading/sleep.h>

#ifdef _WIN32
#include <conio.h> // for _getch()
#define USE_GETCH
#endif _WIN32

int main (int argc, char **argv)
{
 
  gevxl::vid::micro_epsilon_imageripc_camera camera;
  camera.start_thread();
  
  int count = 0;
  while (true) {
    char curr_ch = 0;
    #ifdef USE_GETCH
    if (kbhit())
      curr_ch = _getch();
    #else USE_GETCH
    vcl_cin.get(curr_ch); //gets a char only after you hit the return key
    #endif
    if (curr_ch == 'Q' || curr_ch == 'q') {
      vcl_printf ("You pressed Q: program quit.\n");
      break;
    }

    count++;
    if (count % 10 == 0)
      vcl_cout << ".";
    
    gevxl::threading::sleep (20);

    //for(int i = 0; i < 100; i++) {
    //  gevxl::threading::sleep(1000);
    //}
  }

	return 0;
}

