
#include <avr/io.h>
#include <avr/sleep.h>
#include <compat/deprecated.h>

extern void init();
extern void work();

int main()
{
    init();
    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
    //CLKPR = _BV(CLKPCE);
    //CLKPR = 2;
    while (true)
    {
        sleep_cpu();
        work();
    }
}
