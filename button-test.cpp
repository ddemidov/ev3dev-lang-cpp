#include <unistd.h>
#include <cstdio>

#include "ev3dev.h"

using namespace std;
using namespace ev3dev;

int
main ()
{
  bool up = false, down = false, left = false, right = false, enter = false,
      escape = false;

  while (escape == 0)
    {
      up = button::up.pressed ();
      down = button::down.pressed ();
      left = button::left.pressed ();
      right = button::right.pressed ();
      enter = button::enter.pressed ();
      escape = button::back.pressed ();

      printf ("up:%d down:%d left:%d right:%d enter:%d esc:%d\n", up, down,
	      left, right, enter, escape);
      usleep (100000);
    }
}
