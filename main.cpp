#include "NxtMotor.h"

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
#include <stdint.h>

/* Pin usage
 * PORT  Function       Arduino     HW
 * PD2                     2
 * PD3   Timer2 B Output   3
 * PD4   I/O               4   STDBY
 * PD5   Timer0 B Output   5
 * PD6   Timer0 A Output   6
 * PD7   I/O               7   AIN1
 * PB0   I/O               8   AIN2
 * PB1   Timer1 A Output   9   PWMA
 * PB2   Timer1 B Output  10   PWMB
 * PB3   Timer2 A Output  11
 * PB4   I/O              12   BIN1
 * PB5   I/O              13   BIN2
 * PC0   PCINT8           A0   ROT1B
 * PC1   PCINT9           A1   ROT1A
 * PC2   PCINT10          A2   ROT2B
 * PC3   PCINT11          A3   ROT2A
 * PC4                    A4
 * PC5                    A5
 */

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

static const uint16_t PWMCLK = 5000;
static const uint16_t TIMER1_MAX = 8000000 / PWMCLK;

static volatile uint8_t gMs = 0;
static volatile uint8_t gA = 0;
static volatile uint8_t gB = 0;
static volatile uint8_t gAerror = 0;
static volatile uint8_t gBerror = 0;
static volatile uint8_t gDebug = 0;

void setSpeed(bool a, int16_t speed)
{
    bool cw = true;
    if (speed < 0)
    {
        cw = false;
        speed = -speed;
    }
    if (speed > TIMER1_MAX) speed = TIMER1_MAX;
    if (a)
    {
        OCR1A = speed;
        if (speed == 0)
        {
            cbi(PORTD, PORTD7);
            cbi(PORTB, PORTB0);
        }
        else
        {
            if (cw)
            {
                sbi(PORTD, PORTD7);
                cbi(PORTB, PORTB0);
            }
            else
            {
                cbi(PORTD, PORTD7);
                sbi(PORTB, PORTB0);
            }
        }
    }
    else
    {
        OCR1B = speed;
        if (speed == 0)
        {
            cbi(PORTB, PORTB4);
            cbi(PORTB, PORTB5);
        }
        else
        {
            if (cw)
            {
                sbi(PORTB, PORTB4);
                cbi(PORTB, PORTB5);
            }
            else
            {
                cbi(PORTB, PORTB4);
                sbi(PORTB, PORTB5);
            }
        }
    }
}

static const uint16_t kp = 20, kii = 512, kd = 0;
static uint8_t a = gA, b = gB;
static uint32_t posA = 0, posB = 0;
static int16_t i = 0;
static uint16_t eold = 0;

ISR(TIMER1_OVF_vect)
{
    uint8_t an = gA;
    posA += (int8_t)(an - a);
    a = an;
    uint8_t bn = gB;
    posB += (int8_t)(bn - b);
    b = bn;

    int16_t e = posB - posA;
    i += e;
    if (i > 16000) i = 16000;
    else if (i < -16000) i = -16000;
    int16_t y = kp * e + i / kii + kd * (e - eold);
    eold = e;

    setSpeed(true, y);
}

/* PCINT0-7 */
ISR(PCINT0_vect)
{

}

/* PCINT8-14 */
ISR(PCINT1_vect)
{
    static const int8_t TRANSFORM[16] = {
        // old AB -> AB now
        0,  // 00 -> 00 no change
        1,  // 00 -> 01 cw
        -1, // 00 -> 10 ccw
        2,  // 00 -> 11 error
        -1, // 01 -> 00 ccw
        0,  // 01 -> 01 no change
        2,  // 01 -> 10 error
        1,  // 01 -> 11 cw
        1,  // 10 -> 00 cw
        2,  // 10 -> 01 error
        0,  // 10 -> 10 no change
        -1, // 10 -> 11 ccw
        2,  // 11 -> 00 error
        -1, // 11 -> 01 ccw
        1,  // 11 -> 10 cw
        0,  // 11 -> 11 no change
    };
    static uint8_t old = 0;
    uint8_t now = PINC;
    uint8_t delta = TRANSFORM[((old & 0x3) << 2) | (now & 0x3)];
    if (delta == 2) ++gAerror;
    else gA += delta;
    delta = TRANSFORM[(old & 0xc) | ((now & 0xc) >> 2)];
    if (delta == 2) ++gBerror;
    else gB += delta;
    old = now;
}

/* PCINT16-23 */
ISR(PCINT2_vect)
{

}


int main()
{
    PORTB = 0;
    // Pullups for rotary encoder inputs
    //PORTC = (1 << PORTC0) | (1 << PORTC1) | (1 << PORTC2) | (1 << PORTC3);
    PORTC = 0;
    // enable motor controller
    PORTD = (1 << PORTD4);

    DDRB = (1 << DDB0) | (1 << DDB1) | (1 << DDB2) | (1 << DDB4) | (1 << DDB5);
    DDRC = 0;
    DDRD = (1 << DDD1) | (1 << DDD2) | (1 << DDD3) | (1 << DDD4) | (1 << DDD7);

    PCICR = (1 << PCIE1);
    // Enable pin change irq's for rotery encoders
    PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);

    // Clear on compare match
    TCCR1A = (1 << COM1A1) | (1 << COM1B1);
    // Mode 8: PWM, Phase and Frequence Correct, Top = ICR1, clock = no prescaler
    TCCR1B = (1 << WGM13) | (1 << CS10);
    TIMSK1 = (1 << TOIE1);

    // maximum for desired pwm frequency
    ICR1 = TIMER1_MAX;

    UBRR0 = 103;
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << TXEN0);
    //UCSR0B = (1 << RXCIE0) | (1 << UDRIE0) | (1 << TXEN0) | (1 << RXEN0);
    UCSR0C = 6;
    sbi(UCSR0A, TXC0);

    sei();

    while (gAerror == 0 && gBerror == 0)
    {
        if (UCSR0A & (1 << UDRE0)) UDR0 = gB;


//        setSpeed(true, TIMER1_MAX);
//        while (delta < 360 && gAerror == 0)
//        {
//            uint8_t an = gA;
//            delta += (int8_t)(an - a);
//            a = an;
//        }
//        setSpeed(true, -((int16_t)TIMER1_MAX));
//        while (delta > 0 && gAerror == 0)
//        {
//            uint8_t an = gA;
//            delta += (int8_t)(an - a);
//            a = an;
//        }
//        ++count;
    }
    setSpeed(true, 0);


    const char* DONE = "DONE  ";
    int offset = 0;
    while (true)
    {
        if (UCSR0A & (1 << UDRE0))
        {
            if (offset < 6) UDR0 = DONE[offset];
            else if (offset == 6) UDR0 = gAerror / 16 + 'a';
            else if (offset == 7) UDR0 = gAerror % 16 + 'a';
            else if (offset == 8) UDR0 = gBerror / 16 + 'a';
            else if (offset == 9) UDR0 = gBerror % 16 + 'a';
            else offset = 0;
            ++offset;
        }
    }

    return 0;
}
