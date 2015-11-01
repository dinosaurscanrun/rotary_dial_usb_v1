/*
   pulsedial_key                                                            19 May 2015 GLF

   Hackerspace project to read pulses from an old telephone pulse dialer and output as
   keyboard characters -- requires a Teensy to do that, or else defaults to serial port
   RS-232 at 9600 baud (N81).  When compiling for Teensy, be sure to select
   "USB Keyboard" from the Tools>USB Type menu in the Arduino IDE.

   Design works with Arduino, Attiny85, or Teensy using custom circuit appropriate to
   defined pins -- one pin for "dialing" switch (normally off), one pin for "pulse" switch
   (normally on).

*/

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include "Wprogram.h"
#endif

#include "wiring_private.h"
#include "pins_arduino.h"

#include "glf_scheduler.h"

#ifdef CORE_TEENSY
/* Assume Teensy 2.0 */
int led_dialing_pin      =  11;   /* LED to glow during lockput periods */
int now_dialing_in_pin   =  2;
int dial_pulse_in_pin    =  6;

#else
/* Assume Arduino */
int led_dialing_pin      =  13;   /* LED to glow during lockput periods */
int now_dialing_in_pin   =  7;    /* From original IR-D15A "LEFT" output */
int dial_pulse_in_pin    =  6;    /* From original IR-D15A "RIGHT" output */
#endif

/* Either Arduino or Teensy */

#include <Serial.h>


unsigned char ok_left       = 1;
unsigned char ok_right      = 1;
unsigned char lockout_ended = 0;
unsigned char left_lockout  = 0;
unsigned char right_lockout = 0;


/* --------- The setup() method runs once, when the sketch starts ------------------- */

void setup()
{
  pinMode(led_dialing_pin, OUTPUT);
  digitalWrite(led_dialing_pin,LOW);   /* LED off */

  pinMode(now_dialing_in_pin,  INPUT_PULLUP);
  pinMode(dial_pulse_in_pin, INPUT_PULLUP);

  sched_list_init(0);             /* prepare an empty schedule list -- ignore analog pins */
  sched_event(dial_pulse_in_pin,1,1);  /* set up a recurring 1 ms timer to debounce dial_pulse_in pin */
  sched_event(now_dialing_in_pin,1,1);   /* set up a recurring 1 ms timer to debounce now_dialing_in pin */

#ifdef CORE_TEENSY
  /* set up to hold off for 10 seconds -- gives keyboard time to be recognized and enumerated */
  sched_event(20,0,1000);        /* set up a nonrecurring 1 second timer -- identity 20 */

  /* Serial.begin(9600); */  /* use keyboard instead -- no setup needed */

  /* Wait until 10 second timer elapses */
  while (!(sched_check(20)))
    {
    }

#else
  /* set up to hold off for 1 second */
  sched_event(20,0,1000);        /* set up a nonrecurring 1 second timer -- identity 20 */

  Serial.begin(9600);
  Serial.println("GLF 2015/05/19 -- pulsedial_serial.ino -- Arduino");

  /* Wait until 1 second timer elapses */
  while (!(sched_check(20)))
    {
    }

#endif

}



/* ---- The loop() method runs over and over again, as long as the Arduino has power ----- */

/* --- User event loop -- must handle events quickly (on average) to keep scheduler from running behind --- */

void loop()
{
  unsigned int i;
  unsigned int numpulses;
  unsigned int numdigit;
  static char state = 0;

  /* Handle any user defined manual timer events. */

  /* Event number 20 was defined in setup() as a 1 second timer.
     We will repeatedly use the same event number for a 5 second timeout below. */

  switch (state)
    {
      case 0:         /* outside of a dialing period, await initiation of dialing... */
        {
          /* now_dialing_in pin is normally OFF (HIGH) by default */
          if (sched_pin_golow(now_dialing_in_pin))   /* If now_dialing_in pin went from HIGH to LOW (debounced)... */
            {
              /* initiate dialing period */
              sched_event(20,0,5000);           /* restart 5 second timer */
              digitalWrite(led_dialing_pin,HIGH);   /* LED on to indicate lockout */

              /* dial_pulse_in pin should currently be LOW (Normally ON) */

              /* get count (throw it away) of upgoing pulses on dial_pulse_in pin and reset */
              numpulses = sched_pin_event_count(dial_pulse_in_pin,1,1);

              /* show we are in a dialing period */
              digitalWrite(led_dialing_pin,HIGH);   /* LED on to indicate dialing period */
              state = 1;
            }

          break;

        }

      case 1:         /* have seen dialing switch go low -- watch for it to gow high again,
                         signaling end of user dial period... */
        {
          if (sched_pin_gohigh(now_dialing_in_pin))   /* If now_dialing_in pin went from LOW to HIGH (debounced)... */
            {
              /* end dialing period */
              digitalWrite(led_dialing_pin,LOW);   /* LED off to indicate NOT in dialing period */

              /* get count of upgoing pulses on dial_pulse_in pin and reset */
              numpulses = sched_pin_event_count(dial_pulse_in_pin,1,1);

              /* dial_pulse_in pin should currently be LOW (Normally ON), but ignore it if it isn't */

              numdigit = numpulses;

              if (numdigit > 9)
                {
                  numdigit = 0;
                }

#ifdef CORE_TEENSY

              if (numpulses > 0)
                {
                  Keyboard.print(numdigit);
                }

#else

              if (numpulses > 0)
                {
                  Serial.print(numdigit);
                }

#endif

              state = 0;
            }
          else   /* Normal case -- dialing period still in effect */
            {
              /* see if it dialing period should be timed out -- allows 5 seconds --  if user spins dial
                 and holds it at least that long (with no pulses), program will output a linefeed */

              if (sched_check(20))   /* if timer says "dialing timeout" */
                {
                  digitalWrite(led_dialing_pin,LOW);   /* LED off */

                  /* get count (throw it away) of upgoing pulses on dial_pulse_in pin and reset */
                  numpulses = sched_pin_event_count(dial_pulse_in_pin,1,1);

                  state = 0;

#ifdef CORE_TEENSY
                  /* output a linefeed */
                  Keyboard.println();
#else
                  /* output a linefeed */
                  Serial.println();
#endif
                }
            }

          break;
        }

      default:
        {
        }
    }

  /* Any other event loop processing, as long as it doesn't take long... */
}

