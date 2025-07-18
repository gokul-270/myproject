#include <odrive_control/debug_print.hpp>

#define DEBUGINFO true

void printINFO( char *txtmsg)
{
 if (DEBUGINFO) {
   printf((txtmsg)) ; 
 } 
}
