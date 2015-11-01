/* glf_scheduler library                 18 May 2015 GLF

   2015/05/18 GLF -- Add functionality to allow event counting of transitions and events
                     consistent with other glf_scheduler elements

   2014/08/29 GLF -- tested to work on 16 Mhz ATmega328P Arduino platform.

   Adds functionality to basic timer0 millis() counter to allow scheduling arbitrary events,
   debounce specified digital inputs, and optionally perform background analog port scans
   to avoid the blocking read endemic to Arduino anlogRead() function.

   This version of the library implements preemptive multitasking -- the schedule maintenance
   code is called in background once per ms without user intervention.

   2014/08/28 GLF -- shrink the variables to 8-bit where possible

*/

#ifndef __GLF_SCHED_INT_H__
#define __GLF_SCHED_INT_H__ 1

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif

#include "wiring_private.h"
#include "pins_arduino.h"


#if(defined(__ATtinyX5__))
/* Assume ATtiny85 -- ignore reset (D5/A5) pin since it ordinarily can't be used */
#define MAX_DIGITAL_PIN 5
#define MAX_ANALOG_PIN  3
#else
/* Assume Arduino */
#define MAX_DIGITAL_PIN 13
#define MAX_ANALOG_PIN   5
#endif

/* For scheduler, reserve pin numbers 0 through MAX_DIGITAL_PIN as potential
   debounced digital inputs.  These will be handled in the background.
   Other positive values indicate manually checked schedule timers.
   Negative values indicate an unused timer block.

   Optionally specify Analog pins from 0 to specified pin number to be
   sampled in a ring at 1 ms intervals.  This will allow apparently instantaneous
   readings on all available channels by event loop while avoiding the blocking read
   characteristic of analogRead() function. Actual readings may be from 1 to 6 ms old when
   observed by event loop, but this is practically instantaneous for most purposes
   not requiring precise synchronization.  Even if all ports are specified, an effective
   sample rate of 166 per second is achieved on all analog ports, entirely in the background
   from the standpoint of the user event loop.

   If debounced pins are scheduled, usually a 1 ms recurring period is specified for that pin.
   However, if a 0 ms recurring period is specified, debouncing will be turned OFF and state of
   the pin will be polled every 1 ms, and changes noted as if debounced, but without delay.
   */

#define MAX_SCHED 10

/* Note that volatile attribute is used because instances of this struct are handled by an interrupt. */
typedef struct
{
  volatile unsigned char id;
  volatile unsigned char laststate;
  volatile char debounce_ct;
  volatile unsigned char debounce_state;
  volatile unsigned char debounce_change;
  volatile unsigned int event_ct_up;
  volatile unsigned int event_ct_down;
  volatile unsigned char active;
  volatile unsigned char recurring;
  volatile unsigned long schedtime;
  volatile unsigned long schedms;
}
sched;


extern "C"    /* begin C-only code */
{

  /* Notes on use of sched_list_init():
     In the setup() function (ARDUINO) at the beginning of the execution of a program,
     call sched_list_init() to enable all internals for the scheduler.  If a positive number
     is specified for num_analogs_toscan, a recurring, rotating sampling will be done on each
     analog port from 0 to (num_analogs_toscan-1).  Each sampling will take 1 ms or less, and
     will be synched to the 1 ms timer to the extent possible with cooperative multitasking.
     This recurring sampling will allow user program event loops to get analog port status
     NEARLY immediately (using sched_analogread()) without waiting for the blocking read used
     by the built-in analogRead() function to complete. The sampling processes themselves are
     done without blocking, in the background between 1 ms ticks, allowing user event loops
     to more efficiently use the time to process other events.

     If 0 is specified, NO analog ports will be scanned by the scheduler.  In that case, the user
     is free to scan those ports by other means.
  */

  void sched_list_init(unsigned char num_analogs_toscan);

  /* Notes on use of sched_event():

     If ident is a defined pin number, it will be treated as a debounce pin -- in that case normally
     specify 1 ms delay, and the debouncing will be handled in background by a call to
     sched_background() once per (quick, < 1 ms) user event loop.  Other values of ident specify user
     timers which must be handled manually by the user event loop.

     It is legitimate to set up a recurring event for 0 ms, IF it is tied to a debouncing pin.
     The 0 ms will be changed to 1 ms each time it is triggered (recurring).  The pin will
     NOT be debounced as usual, but will be read directly each ms, though other debouncing
     functions, such as identification of changed state and event count continue to work.

     If recur is NOT specified, a time of 0 ms will effectively reset the timer and
     turn it off, while leaving in place the ID's entry in the list.  This characteristic is used
     to advantage in sched_cancel() below, which is the preferred cancellation method for the user.
  */

  char sched_event(char ident, char recur, unsigned long ms);

  /* Cancel the timout for an identified scheduled event, while leaving its entry in place in the
     schedule list.
  */

  char sched_cancel(char ident);

  unsigned int sched_analogread(unsigned char pin);   /* manual asynchronous read of analog port from preset
                                   buffer filled in by background process */

  char sched_check(char ident);   /* Manual asynchronous check of ID'd schedule --
                                 return value HIGH means timeout was reached. */

  unsigned int sched_pin_event_count(char ident, char level, char reset);
  /* Manual asynchronous check of ID'd schedule --
  return value is count of defined transition events since last reset. */

  char sched_pin_gohigh(char ident);   /* Manual asynchronous check of ID'd (debounced) pin change LOW to HIGH
                                      returns HIGH on leading edge of change LOW to HIGH on associated pin. */

  char sched_pin_golow(char ident);   /* Manual asynchronous check of ID'd (debounced) pin change HIGH to LOW
                                      returns HIGH on leading edge of change HIGH to LOW on associated pin. */

  char sched_pin_level(char ident, char level);   /* Manual asynchronous check of ID'd debounce pin level
                                                returns HIGH or LOW for current (debounced) level seen. */

  /* void sched_background(void); */  /* This USED TO BE REQUIRED within user event loop, at least once per ms
                                         to process background events, but is now included in interrupt service 
                                         routine (ISR). */

}             /* end C-only code */

#endif   /* ... of __GLF_SCHED_INT_H__ */

