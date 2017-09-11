// vim: expandtab tabstop=2 shiftwidth=2 softtabstop=2

#include "ch.h"
#include "hal.h"

#include "wdtSetup.h"

static THD_WORKING_AREA(wdtWorkingArea, 64);
MUTEX_DECL(wdt_mutex);

static unsigned wdt_nocounting = 1;
#define WDT_MAGIC_BOTTOM 0x8531
static unsigned wdt_count = WDT_MAGIC_BOTTOM + 1;

static __attribute__((noreturn)) THD_FUNCTION(wdtTriggerThread, arg __attribute__((unused)))
{
  chRegSetThreadName("wdtTrigger");

  const WDGConfig conf = {
    .pr  = STM32_IWDG_PR_256,
    .rlr = STM32_IWDG_RL(256)
  };
  wdgStart(&WDGD1, &conf);

  while(1) {
    chThdSleepMilliseconds(10);
    chMtxLock(&wdt_mutex);
    if(wdt_count > WDT_MAGIC_BOTTOM || wdt_nocounting) {
      if(!wdt_nocounting)
        --wdt_count;
      wdgReset(&WDGD1);
    }
    chMtxUnlock(&wdt_mutex);
  }
}

void wdtAdd(unsigned c)
{
  chMtxLock(&wdt_mutex);
  wdt_count += c;
  chMtxUnlock(&wdt_mutex);
}

void wdtEnable(unsigned enable)
{
  chMtxLock(&wdt_mutex);
  wdt_nocounting = !enable;
  if(wdt_nocounting)
    wdt_count = WDT_MAGIC_BOTTOM + 1;
  chMtxUnlock(&wdt_mutex);
}



void wdtSetup(void)
{
  chThdCreateStatic	( wdtWorkingArea		// working area (stack etc)
      , sizeof(wdtWorkingArea)
      , HIGHPRIO			// priority
      , wdtTriggerThread		// main function
      , NULL				// arg
      );
}

