/* glf_scheduler library                    18 May 2015 GLF

   2015/05/18 GLF -- Add functionality to allow event counting of transitions and events
                     consistent with other glf_scheduler elements

   2014/11/22 GLF -- port to 8 Mhz ATtiny85 platform.

   2014/08/29 GLF -- tested to work on 16 Mhz ATmega328P Arduino platform.

   2014/08/28 GLF -- Developed by addition to sched_coop library -- add support for extension
                     of Timer 0 interrupt for preemptive scheduling -- no longer need to call
                     sched_background() in user event loop to keep track of milliseconds because
                     the core timing function will be handled by a custom ISR.

                     User event loop still needs to query for scheduled events.

   Adds functionality to basic timer0 millis() counter to allow scheduling arbitrary events,
   debounce specified digital inputs, and optionally perform background analog port scans
   to avoid the blocking read endemic to Arduino anlogRead() function.

   This version of the library implements preemptive multitasking -- the schedule maintenance
   code is called in background once per ms without user intervention.

   2014/08/28 GLF -- shrink the variables to 8-bit where possible

*/

#if ARDUINO >= 100
#include <Arduino.h>
#else
#include "WProgram.h"
#endif

#include "wiring_private.h"
#include "pins_arduino.h"

#include "glf_scheduler.h"

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


extern "C"    /* begin C-only code */
{

  /*  constants for debouncing keystrokes -- assume tested every 1 ms
      -- simulate a low-pass filter into a Schmitt trigger  */
#define DEBOUNCE_THRESH_UP     15
#define DEBOUNCE_THRESH_DOWN    5
#define DEBOUNCE_THRESH_MAX    20
#define DEBOUNCE_THRESH_BOTTOM  0

  static sched schedlist[MAX_SCHED+1];
  static unsigned int sched_analoglist[MAX_ANALOG_PIN+1];
  static char sched_count = 0;
  static unsigned long sched_priorms = 0;
  static unsigned char sched_num_analogs = 0;
  static unsigned char sched_current_analog = 0;

  static volatile char sched_initialized = 0;         /* Only nonzero when fully set up (including ISR). */
  static volatile char sched_ISR_installed = 0;       /* Only nonzero when ISR has been initialized. */

  static void (*vecptr)(void) = NULL;    /* Will hold pointer to original TIMER0_OVF_vect code
                                          when initialized.  Cannot statically assign
                                          the correct address -- must calculate at runtime. */


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


     2014/08/28 GLF -- add initialization of ISR to handle background schedule maintenance
                       using extension to Timer 0 interrupt.
  */

  void sched_list_init(unsigned char num_analogs_toscan)
  {
    char i;

    /* Prepare the necessary data to set up the schedule maintenance procees called by a Timer0 ISR
       running parallel to the built-in Arduino functionality affecting delay() and millis(). */

    sched_initialized = 0;      /* disable further schedule or ISR processing in case this call is
                                  resetting the scheduler in the middle of a program run. */

    for (i=0; i<MAX_SCHED; i++)
      {
        schedlist[i].id = 0;
        schedlist[i].laststate = 0;
        schedlist[i].active = 0;
        schedlist[i].recurring = 0;
        schedlist[i].schedtime = 0;
        schedlist[i].schedms = 0;
        schedlist[i].event_ct_up = 0;
        schedlist[i].event_ct_down = 0;
        schedlist[i].debounce_change = 0;
        schedlist[i].debounce_state = LOW;
      }

    if (num_analogs_toscan > (MAX_ANALOG_PIN+1))
      {
        num_analogs_toscan = MAX_ANALOG_PIN + 1;
      }

    sched_num_analogs = num_analogs_toscan;

    for (i=0; i<=MAX_ANALOG_PIN; i++)
      {
        sched_analoglist[i] = 0;
      }

    sched_current_analog = 0;
    sched_count = 0;
    sched_priorms = millis();

    /* NOW enable the ISR to handle background scheduling processes. */
    if (!sched_ISR_installed)  /* only do this once per program run */
      {
        /* First disable the Timer0 overflow interrupt while we're configuring */
#if(defined(__ATtinyX5__))
        /* Assume ATtiny85 */
        TIMSK &= ~(1<<TOIE0);
#else
        /* Assume Arduino */
        TIMSK0 &= ~(1<<TOIE0);
#endif

        /* Set up flags controlling Timer0 COMPB vector so that it always fires
           shortly after the Timer0 Overflow, allowing us to extend its function reliably. */
        TCNT0 = 0;
        TCCR0B = 0;

        /* The following (OCR0B) determines the phase of firing the COMPB interrupt
           within the cycle of the OVF interrupt for Timer0 -- shorter is
           better, since the timing is crisply bound to the millis() count
           and more consistent in tming across loops. It holds its value
           for the life of the program.
           */

        OCR0B = 4;    /* about 4*4 micoseconds delay in firing COMPB interrupt after OVF */

        /* turn on CTC mode */
        TCCR0A |= (1 << WGM01);

        /* Set CS01 and CS00 bits for 64 prescaler -- same as original millis() process */
        TCCR0B |= (1 << CS01) | (1 << CS00);


#if(defined(__ATtinyX5__))
        /* Assume ATtiny85 */
        /* enable timer compare interrupt */
        TIMSK |= (1 << OCIE0B);
        /* enable Timer0 overflow interrupt */
        TIMSK |= (1<<TOIE0);
#else
        /* Assume Arduino */
        /* enable timer compare interrupt */
        TIMSK0 |= (1 << OCIE0B);
        /* enable Timer0 overflow interrupt */
        TIMSK0 |= (1<<TOIE0);
#endif

        sched_ISR_installed = 1;
      }

    sched_initialized = 1;
  }


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


  char sched_event(char ident, char recur, unsigned long ms)
  {
    char i;
    char pos = -1;
    unsigned long timems;

    timems = millis();

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. If there is room, add another id to the list. */
        if (sched_count < MAX_SCHED)
          {
            pos = sched_count;
            sched_count++;
          }
      }

    if (pos >= 0)
      {
        schedlist[pos].id        =    ident;
        schedlist[pos].schedtime =    ms + timems;
        schedlist[pos].schedms   =    ms;
        schedlist[pos].recurring =    recur;
        schedlist[pos].event_ct_up = 0;
        schedlist[pos].event_ct_down = 0;
        schedlist[pos].active = 1;

        if ((!recur) && (ms == 0))  /* this specifies that timer should be turned off */
          {
            schedlist[pos].active = 0;
          }

        if ((schedlist[pos].id >= 0) && (schedlist[pos].id <= MAX_DIGITAL_PIN)) /* if this is a monitored pin... */
          {
            schedlist[pos].laststate = digitalRead(schedlist[pos].id);

            if (schedlist[pos].laststate)
              {
                /* if pin is HIGH... */
                schedlist[pos].debounce_ct = DEBOUNCE_THRESH_MAX;  /* Immediately force Schmitt trigger action */
                schedlist[i].debounce_change = 0;
                schedlist[i].debounce_state = HIGH;
              }
            else
              {
                schedlist[pos].debounce_ct = DEBOUNCE_THRESH_BOTTOM;  /* Immediately force Schmitt trigger action */
                schedlist[i].debounce_change = 0;
                schedlist[i].debounce_state = LOW;
              }
          }

        return 1;
      }

    /* indicate that no match to event was found */
    return 0;
  }


  /* Cancel the timout for an identified scheduled event, while leaving its entry in place in the
     schedule list.
  */

  char sched_cancel(char ident)
  {
    return sched_event(ident,0,0L);
  }


  static char sched_check0(char pos)     /* individual automated check of one schedule in list -- must be called
                                        from within sched_background() more often than once per ms */
  {
    unsigned long timems;
    char debounce = 1;

    timems = millis();

    if ((pos < 0) || (pos >= sched_count))
      {
        return 0;
      }

    if (schedlist[pos].active)
      {
        if (schedlist[pos].schedtime <= timems)
          {
            /* Time is up! */
            if (schedlist[pos].recurring)
              {
                /* remain active and bump to next scheduled time */
                schedlist[pos].schedtime = (schedlist[pos].schedtime + schedlist[pos].schedms);

                if (schedlist[pos].schedms == 0)
                  {
                    debounce = 0;
                    schedlist[pos].schedtime++;     /* force schedule time to next ms */
                  }
              }
            else
              {
                schedlist[pos].active = 0;
              }

            if ((schedlist[pos].id >= 0) && (schedlist[pos].id <= MAX_DIGITAL_PIN)) /* if this is a monitored pin... */
              {
                schedlist[pos].laststate = digitalRead(schedlist[pos].id);

                if (schedlist[pos].laststate)
                  {
                    /* if pin is HIGH... */
                    if (!debounce)
                      {
                        schedlist[pos].debounce_ct = DEBOUNCE_THRESH_MAX;  /* Immediately force Schmitt trigger action */
                      }
                    else
                      {
                        /* if instantaneously HIGH, count up to simulate low-pass filter */
                        schedlist[pos].debounce_ct++;
                      }

                    /* simulate Schmitt trigger (hysteresis) */
                    if (schedlist[pos].debounce_ct > DEBOUNCE_THRESH_UP)
                      {
                        schedlist[pos].debounce_ct = DEBOUNCE_THRESH_MAX;  /* Schmitt trigger action */

                        if (!(schedlist[pos].debounce_state))    /* if it WAS LOW... */
                          {
                            schedlist[pos].debounce_change++;     /* indicate changed state until checked by user */
                            schedlist[pos].event_ct_up++;         /* indicate up count until reset by user */
                          }

                        schedlist[pos].debounce_state = HIGH;   /* force state at this threshold */
                      }
                  }
                else
                  {
                    /* if pin is LOW... */

                    if (!debounce)
                      {
                        schedlist[pos].debounce_ct = DEBOUNCE_THRESH_BOTTOM;  /* Immediately force Schmitt trigger action */
                      }
                    else
                      {
                        /* if instantaneously LOW, count down to simulate low-pass filter */
                        schedlist[pos].debounce_ct--;
                      }

                    /* simulate Schmitt trigger (hysteresis) */
                    if (schedlist[pos].debounce_ct < DEBOUNCE_THRESH_DOWN)
                      {
                        schedlist[pos].debounce_ct = DEBOUNCE_THRESH_BOTTOM;  /* Schmitt trigger action */

                        if (schedlist[pos].debounce_state)     /* if it WAS HIGH... */
                          {
                            schedlist[pos].debounce_change++;     /* indicate changed state until checked by user */
                            schedlist[pos].event_ct_down++;       /* indicate down count until reset by user */
                          }

                        schedlist[pos].debounce_state = LOW;   /* force state at this threshold */
                      }
                  }
              }

            return 1;
          }
      }

    /* indicate that no match to event was found */
    return 0;
  }


  static char sched_pin_test0(char pos, char level, char delta)     /* individual check of one debounced pin change in list */
  {
    char changes;

    if ((pos < 0) || (pos >= sched_count))
      {
        return 0;
      }

    if (schedlist[pos].active)
      {
        changes = schedlist[pos].debounce_change;
        schedlist[pos].debounce_change = 0;

        if (!delta)      /* special indicator to report raw debounced level only, not changes in level */
          {
            return schedlist[pos].debounce_state;
          }

        if (changes)
          {
            /* Something happened on pin -- see if it matches expected level */
            if (level)
              {
                if (schedlist[pos].debounce_state)
                  {
                    return 1;
                  }
              }

            else
              {
                if (!(schedlist[pos].debounce_state))
                  {
                    return 1;
                  }
              }
          }
      }

    /* indicate that no pin change event was found */
    return 0;
  }


  unsigned int sched_analogread(unsigned char pin)   /* manual asynchronous read of analog port from preset buffer
                                   filled in by background process */
  {
    if (pin > sched_num_analogs)
      {
        return 0;
      }

    /* because this is filled in by ISR, it should be double-checked for change across atomic boundaries */

    return sched_analoglist[pin];
  }


  char sched_check(char ident)   /* Manual asynchronous check of ID'd schedule --
                                 return value HIGH means timeout was reached. */
  {
    char i;
    char pos = -1;

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. */
        return 0;
      }

    return sched_check0(pos);
  }


  unsigned int sched_pin_event_count(char ident, char level, char reset)
  /* Manual asynchronous check of ID'd schedule --
  return value is count of defined transition events since last reset. */
  {
    char i;
    char pos = -1;
    unsigned int val;
    unsigned int holddown;
    unsigned int holdup;

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. */
        return 0;
      }


    /* Because counting is driven by an interrupt, it is possible for count to bump up during user retrieval
       or reset. The following are seemingly complicated, but avoid losing a count if that happens.
       Most of the time, the do while loops below execute only once, but if an interrupt happens to bump the
       affected count while we're looking at it, we loop again and get a stabilized count. This extra loop will
       only happen once if at all, since the interrupt in question only happens once per ms and this routine is
       MUCH faster than that. */

    if (level)
      {
        /* get count of upward transitions -- make sure if interrupt happens, it is updated */
        do
          {
            holdup = schedlist[pos].event_ct_up;
            val = holdup;
          }
        while (holdup != schedlist[pos].event_ct_up);  /* falls through when count is stable */
      }
    else
      {
        /* get count of downward transitions -- make sure if interrupt happens, it is updated */
        do
          {
            holddown = schedlist[pos].event_ct_down;
            val = holddown;
          }
        while (holddown != schedlist[pos].event_ct_down);  /* falls through when count is stable */
      }

    if (reset)
      {
        if (schedlist[pos].event_ct_up != holdup) /* Uh-oh, another interrupt just caught a count... */
          {
            /* We already have a count we can use, so pass the extra count to the next lookup */
            schedlist[pos].event_ct_up = 1;
          }
        else
          {
            schedlist[pos].event_ct_up = 0;
          }

        if (schedlist[pos].event_ct_down != holddown) /* Uh-oh, another interrupt just caught a count... */
          {
            /* We already have a count we can use, so pass the extra count to the next lookup */
            schedlist[pos].event_ct_down = 1;
          }
        else
          {
            schedlist[pos].event_ct_down = 0;
          }
      }

    return val;
  }


  char sched_pin_gohigh(char ident)   /* Manual asynchronous check of ID'd (debounced) pin change LOW to HIGH
                                      returns HIGH on leading edge of change LOW to HIGH on associated pin. */
  {
    char i;
    char pos = -1;

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. */
        return 0;
      }

    return sched_pin_test0(pos,1,1);
  }


  char sched_pin_golow(char ident)   /* Manual asynchronous check of ID'd (debounced) pin change HIGH to LOW
                                      returns HIGH on leading edge of change HIGH to LOW on associated pin. */
  {
    char i;
    char pos = -1;

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. */
        return 0;
      }

    return sched_pin_test0(pos,0,1);
  }


  char sched_pin_level(char ident, char level)   /* Manual asynchronous check of ID'd debounce pin level
                                                returns HIGH or LOW for current (debounced) level seen. */
  {
    char i;
    char pos = -1;

    /* see if this event id is already in list */
    for (i=0; i<sched_count; i++)
      {
        if (ident == schedlist[i].id)
          {
            pos = i;
          }
      }

    if (pos < 0)   /* NOT already in list */
      {
        /* No existing event with this id was found. */
        return 0;
      }

    return sched_pin_test0(pos,level,0);
  }

  static volatile unsigned int alog_val = 1023;
  static volatile uint8_t ahigh = 0x03;
  static volatile uint8_t alow = 0xFF;


  /* ------------------------------------------------------------------------------------------------ */

  /* 2014/08/28 GLF -- convert the user-event-loop-called sched_background() to interrupt-based
                       (invisible) operation.
  */


  /* 2014/08/29 GLF -- The sched_background() function is (almost) unchanged from the cooperative multitask
                        version, and could be called from a user event loop in the same way, except that
                        since the function is not thread-safe, it would likely cause trouble as the ISR
                        above might call it reentrantly.  Therefore it has been renamed and declared static
                        to prevent accidental user calls.
  */

  static void sched_background_int(void)
  {
    char i;
    char toss;
    unsigned long timems;

    timems = millis();

    /* NOTE: By design, this function will ONLY be called if millis() has ALREADY been incremented,
             so the check below, needed in "sched_coop", is not needed here.    */

    /* no point in further processing until at least 1 ms has elapsed */
    /*
    if (timems == sched_priorms)
      {
       return;
      }
    */

    /* at this point a 1 ms elapsed event has been triggered, and all processes
       which depend on that trigger should be executed */
    sched_priorms = timems;

    /* if any analog ports are to be scanned, do so now... */

    /* To ensure timing of input/output is crisply synchronized with the timer, don't use the
    AnalogRead() function directly, since it holds until conversion complete, a delay of
    indeterminate time.  Instead, decompose the AnalogRead function here, starting an input
    ADC conversion, then outputting to DAC whil ADC is in progress, then finally completing
    the ADC conversion and acquiring the input data.

    NOTE:  At this time this code is only KNOWN to work on the ATMEGA328 (Duemilonovae
           or Diavolino).
    */

    if (sched_num_analogs)
      {
        /* Built-in analogRead function blocks because it must start an ADC conversion,
           then wait for results.  To avoid the wait, work backwards -- read the result FIRST,
           assuming it was ALREADY set up for conversion at least 1 ms earlier.
        */

        /* Assume conversion is complete, read the result for current analog port, then store it
           in the corresponding position in array. */

        /* finish any ADC conversion started in previous loop -- this takes up to 25
           ADC clock cycles and so completes between 1 ms clock ticks (timer 0 calls)
           which set up millis() used to schedule the start of these analog reads. */

#if defined(ADCSRA) && defined(ADCL)
        /* ADSC is cleared when the conversion finishes -- for now just assume that */
        alow  = ADCL;
        ahigh = ADCH;
#else
        /* we dont have an ADC, return 0 */
        alow  = 0;
        ahigh = 0;
#endif

        /* combine the two bytes into one 10-bit value */
        alog_val = (ahigh << 8) | alow;

        sched_analoglist[sched_current_analog] = alog_val;
        sched_current_analog++;

        if (sched_current_analog >= sched_num_analogs)
          {
            sched_current_analog = 0;
          }

        /* Start a new conversion for the next port -- get the results next time through. */
#if defined(ADMUX)
        /* For some unknown reason, a statement of the form:
                          ADMUX = (0x40) | (sched_current_analog & 0x07);
           does not work -- a constant seems to be necessary instead of sched_current_analog.
           Therefore, work around with switch statement.           */

        /* Note: the (0x40) below sets up 10-bit ADC (from ATMEL manual) */
        switch (sched_current_analog)
          {
            case 1:
              {
                ADMUX = (0x40) | (0x01);
                break;
              }

            case 2:
              {
                ADMUX = (0x40) | (0x02);
                break;
              }

            case 3:
              {
                ADMUX = (0x40) | (0x03);
                break;
              }

            case 4:
              {
                ADMUX = (0x40) | (0x04);
                break;
              }

            case 5:
              {
                ADMUX = (0x40) | (0x05);
                break;
              }

            default:  /* assume port 0 */
              {
                ADMUX = (0x40) | (0x00);
              }
          }

#endif
#if defined(ADCSRA) && defined(ADCL)
        sbi(ADCSRA, ADSC);
#endif
      }

    /* check for any digital pin debounce monitors... */
    for (i=0; i<sched_count; i++)
      {
        if ((schedlist[i].id >= 0) && (schedlist[i].id <= MAX_DIGITAL_PIN)) /* if this is a monitored pin... */
          {
            toss = sched_check0(i);
          }
      }
  }




  /*
     New ISR to handle sched_background() processes without losing original millis() maintenance.

     It is difficult to balance the limited number of AVR timers against desired functionality
     built into the Arduino runtime platform or common libraries.  Frequently both Timer1
     and Timer2 get tied up in highly technical handling of special data transmission protocols
     or high speed signal porocessing, and it would be useful to have the ability to add functionality
     to existing timer processes rather than override and supplant them.  Ordinarily, one is ill advised
     to encroach upon Timer0, due to the threat of loss of the very useful millis() function, which is
     maintained using a Timer0 Overflow ISR (whose interface and data are invisible to user programs).

     The technique embodied in this file avoids the potential conflict by setting up a parallel interrupt
     with a different ISR that stays in sync with the original Timer0 interrupt.  The new interrupt can
     be used to perform any (quickly executed) functionality tied to a 1 ms clock.  The scheduler library
     fits within that specification.  The scheduler granularity is to the millisecond, and without changing
     the standard A/D clock rate, 1 ms is plenty of time to complete analog samples and process them
     in background, so that functionality is also included in the scheduler, to allow avoidance of the
     "blocking read" used by the built-in analogRead() function.  The user would have no reason
     to SLOW the sample acquisition time, and if for some reason, the sample clock is sped up, it need not
     conflict with the scheduler library.  Both will still work as long as precautions are taken
     to not sample two ports simultaneously.  Judicicious use of flags and blocking can be added to this
     library to support high speed asynchronous use of other analog ports (NOTE: GLF will consider adding this
     to this library later).

     The TIMER0_COMPB interrupt is set up in sched_list_init() to run parallel to the TIMER0_OVF interrupt,
     at the same frequency but out of phase.  This allows thae additional ISR to extend the functionality
     provided by the built-in Arduino Timer0 handling, which maintains the millis() and micros() counts.

     It is assumed that all this function's processes will finish in much less than 1 ms
     so the Timer 0 overflow (or COMPB) interrupt won't be spuriously retriggered before this function is done.
  */

#if(defined(__ATtinyX5__))
  /* Assume ATtiny85 */
  ISR(TIM0_COMPB_vect)
#else
  /* Assume Arduino */
  ISR(TIMER0_COMPB_vect)
#endif
  {
    sei();   /* NOTE: leave interrupts enabled as early as possible */

    /* It is assumed that this ISR is called once per ms, and will take much less than 1 ms to complete. */

    if (sched_initialized)  /* Prevent doing anything critical until structures are set up. */
      {
        /* Because this ISR is ALWAYS executed shortly after the millis() count is
           incremented and remains in sync with it, it is not necessary to have
           interrupts disabled during this routine after registers are saved.
           Because of the timing of this interrupt with respect to the OVF interrupt,
           the millis() count won't change for almost another ms and can be assumed to be
           read atomically at the beginning of any function whch is called immediately. */


        /* Call schedule maintenance here -- in the "sched_coop" version of the scheduler, this
           was a required call in the user event loop. */

        sched_background_int();
      }

    /* Do not need to reload the timer count value -- it is auto-incremented and
      controls both the OVF and the COMPB vectors.  The value does not change
      the frequency of the interrupt, only the phase within the overall Timer0
      overflow loop. Therefore, (if I understand correctly), the value of 4
      set up at the beginning will hold indefinitely and cause this ISR to execute
      approximately 4*4 or 16 microseconds after the millis() count is updated.
      Though this timing is not really that tight, it does appear that lower values
      of OCR0B (COMPB count spec) cause the COMPB vector to execute closer to the
      end of the OVF vector which updates the millis() count. */
  }




  /* ------------------------------------------------------------------------------------------------ */



}             /* end C-only code */



