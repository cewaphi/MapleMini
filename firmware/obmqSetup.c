// vim: expandtab tabstop=2 shiftwidth=2 softtabstop=2

#include "hal.h"
#include "chmtx.h"

#include "obmq.h"
#include "obmqSetup.h"

// OBMQ thread setup

static THD_WORKING_AREA(obmqWorkingArea, 64);
static OneBitMessageQueue obmq;
MUTEX_DECL(obmq_mutex);

static void setLed(void * data __attribute__((unused)), char value)
{
  if(value)
    palSetPad(GPIOB, GPIOB_LED);
  else
    palClearPad(GPIOB, GPIOB_LED);
}

static __attribute__((noreturn)) THD_FUNCTION(obmqTriggerThread, arg __attribute__((unused)))
{
  chRegSetThreadName("obmqTrigger");
  while (true) {
    obmq_trigger(&obmq);
    chThdSleepMilliseconds(10);
  }
}

void obmqSendMessage(char message)
{
  chMtxLock(&obmq_mutex);
  obmq_queuemessage(&obmq, message);
  chMtxUnlock(&obmq_mutex);
}

void obmqSetup(void)
{
  obmq_init(&obmq, setLed, NULL, 0, 0, 5, 8, 0);
  obmqSendMessage(MSG_INIT_OBMQ);
  chThdCreateStatic	( obmqWorkingArea		// working area (stack etc)
      , sizeof(obmqWorkingArea)
      , HIGHPRIO			// priority
      , obmqTriggerThread		// main function
      , NULL				// arg
      );
}

