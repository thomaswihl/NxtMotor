#include "NxtMotor.h"

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
#include <stdint.h>
#include <limits.h>

// /home/thomas/bin/arduino-1.0.5/hardware/tools/avrdude -C/home/thomas/bin/arduino-1.0.5/hardware/tools/avrdude.conf -patmega328p -P/dev/ttyUSB0 -carduino -b57600 -D -Uflash:w:build-cli/NxtMotor.hex

/* Pin usage
 * PORT  Function       Arduino HW
 * PD0                    RXD
 * PD1                    TXD
 * PD2   I/O               2              ROT3A
 * PD3   Timer2 B Output   3              MOT2ENA
 * PD4   I/O               4              ROT3B
 * PD5   Timer0 B Output   5              MOT3A
 * PD6   Timer0 A Output   6              MOT3ENA
 * PD7   I/O               7              MOT3B
 *
 * PB0   I/O               8n
 * PB1   Timer1 A Output   9              MOT1ENA
 * PB2   Timer1 B Output  10              MOT1A
 * PB3   Timer2 A Output  11   DBG MOSI   MOT2A
 * PB4   I/O              12   DBG MISO   MOT1B
 * PB5   I/O              13   DBG SCK    MOT2B
 * PB6                   XTAL1
 * PB7                   XTAL2
 *
 * PC0   PCINT8           A0              ROT1B
 * PC1   PCINT9           A1              ROT1A
 * PC2   PCINT10          A2              ROT2B
 * PC3   PCINT11          A3              ROT2A
 * PC4   PCINT12          A4              I2C SDA
 * PC5   PCINT13          A5              I2C SCL
 * PC6   PCINT13         RESET
 */

#define SPI_PORT PORTB
#define SPI_DDR DDRB
#define SPI_PIN PINB
#define SPI_SS_PIN PORTB2
#define SPI_MOSI_PIN PORTB3
#define SPI_MISO_PIN PORTB4
#define SPI_SCK_PIN PORTB5

#define M1IN1_PORT PORTB
#define M1IN1_DDR DDRB
#define M1IN1_PIN PORTB2
#define M1IN1_PWM OCR1B

#define M1IN2_PORT PORTB
#define M1IN2_DDR DDRB
#define M1IN2_PIN PORTB4

#define M2IN1_PORT PORTB
#define M2IN1_DDR DDRB
#define M2IN1_PIN PORTB3
#define M2IN1_PWM OCR2A

#define M2IN2_PORT PORTB
#define M2IN2_DDR DDRB
#define M2IN2_PIN PORTB5

#define M1ENA_PORT PORTB
#define M1ENA_DDR DDRB
#define M1ENA_PIN PORTB1

#define M2ENA_PORT PORTD
#define M2ENA_DDR DDRD
#define M2ENA_PIN PORTD3

#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif
#define MIN(X,Y) ((X) < (Y) ? (X) : (Y))
#define MAX(X,Y) ((X) > (Y) ? (X) : (Y))
typedef uint32_t encoder_t;
typedef int32_t controller_t;

struct Encoder
{
    uint8_t lastStatus;
    uint8_t encoderError;
    encoder_t encoderPos;
};

struct Motor
{
    uint8_t encoder;
    encoder_t targetPos;
    controller_t integralError;
    controller_t previousError;
    controller_t y;
};


static const uint16_t PWM_MAX = 255;
static const unsigned MOTOR_COUNT = 1;
static const unsigned ENCODER_COUNT = 2;


static volatile uint16_t gMs = 0;

volatile static Motor gMotor[MOTOR_COUNT] = {0};
volatile static Encoder gEncoder[ENCODER_COUNT] = {0};
volatile static uint8_t gSpiData = 0;

void setSpeed(uint8_t motorIndex, int16_t speed)
{
    bool cw = true;
    if (speed < 0)
    {
        cw = false;
        speed = -speed;
    }
    if (speed > PWM_MAX) speed = PWM_MAX;
    if (motorIndex == 0)
    {
        if (cw)
        {
            M1IN1_PWM = speed;
            cbi(M1IN2_PORT, M1IN2_PIN);
        }
        else
        {
            M1IN1_PWM = PWM_MAX - speed;
            sbi(M1IN2_PORT, M1IN2_PIN);
        }
    }
    else
    {
        if (cw)
        {
            M2IN1_PWM = speed;
            cbi(M2IN2_PORT, M2IN2_PIN);
        }
        else
        {
            M2IN1_PWM = PWM_MAX - speed;
            sbi(M2IN2_PORT, M2IN2_PIN);
        }
    }
}


ISR(TIMER0_OVF_vect)
{
}

ISR(TIMER1_OVF_vect)
{
    static const int16_t kp = 32, kiShift = 2, kd = 32;
    static const int16_t I_MAX = PWM_MAX >> 2;
    //sbi(PORTD, PORTD1);
    sei();

    ++gMs;
    if ((gMs & 1023) != 0) return;

    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        controller_t error = gMotor[i].targetPos - gEncoder[gMotor[i].encoder].encoderPos;
        if (error == 0)
        {
            gMotor[i].integralError = 0;
        }
        else
        {
            gMotor[i].integralError += error;
            if (gMotor[i].integralError > I_MAX) gMotor[i].integralError = I_MAX;
            else if (gMotor[i].integralError < -I_MAX) gMotor[i].integralError = -I_MAX;
        }
        controller_t y = kp * error + (gMotor[i].integralError << kiShift) + kd * (error - gMotor[i].previousError);
        gMotor[i].previousError = error;

        gMotor[i].y = y;
        setSpeed(i, y >> 4);
    }
    //cbi(PORTD, PORTD1);
}

ISR(TIMER2_OVF_vect)
{
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
    //sbi(PORTD, PORTD1);
    sei();
    uint8_t now = PINC;
    uint8_t shift = 0;
    for (uint8_t i = 0; i < ENCODER_COUNT; ++i)
    {
        uint8_t status = (now >> shift) & 3;
        int8_t delta = TRANSFORM[(gEncoder[i].lastStatus << 2) | status];
        if (delta == 2) ++gEncoder[i].encoderError;
        else gEncoder[i].encoderPos += delta;
        shift += 2;
        gEncoder[i].lastStatus = status;
    }
    //cbi(PORTD, PORTD1);
}

/* PCINT16-23 */
ISR(PCINT2_vect)
{

}

ISR(SPI_STC_vect)
{
    SPDR = gSpiData++;
    (void)SPDR;
}

void putc(uint8_t c)
{
    while (!(UCSR0A & (1 << UDRE0)))
    { }
    UDR0 = c;
}


static const uint32_t POW[] = { 1, 10, 100, 1000, 10000, 100000, 1000000, 10000000, 100000000, 1000000000 };

void print(uint8_t data)
{
    for (int i = 2; i >= 0; --i) putc(((data / (uint8_t)POW[i]) % 10) + '0');
}

void print(uint16_t data)
{
    for (int i = 4; i >= 0; --i) putc(((data / (uint16_t)POW[i]) % 10) + '0');
}

void print(int16_t data)
{
    if (data < 0)
    {
        putc('-');
        data = -data;
    }
    for (int i = 4; i >= 0; --i) putc(((data / (uint16_t)POW[i]) % 10) + '0');
}

void print(uint32_t data)
{
    for (int i = 9; i >= 0; --i) putc(((data / POW[i]) % 10) + '0');
}

ISR(TWI_vect)
{
    switch (TWSR & 0xf8)
    {
    case 0xa8:
    case 0xb8:
        TWDR = gSpiData++;
        break;
    }
    sbi(TWCR, TWINT);
}


int main()
{
    PORTB = 0;
    PORTC = 0;
    PORTD = 0;
    DDRB = 0;
    DDRC = 0;
    DDRD = 0;

    sbi(M1IN1_DDR, M1IN1_PIN);
    sbi(M1IN2_DDR, M1IN2_PIN);
    sbi(M2IN1_DDR, M2IN1_PIN);
    sbi(M2IN2_DDR, M2IN2_PIN);
    sbi(M1ENA_DDR, M1ENA_PIN);
    sbi(M2ENA_DDR, M2ENA_PIN);
    sbi(M1ENA_PORT, M1ENA_PIN);
//    sbi(SPI_DDR, SPI_MISO_PIN);
//    sbi(SPCR, SPIE);
//    sbi(SPCR, SPE);


    PCICR = (1 << PCIE1);
    // Enable pin change irq's for rotery encoders
    PCMSK1 = (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);

    /* Clear on compare match
     * Mode 1: PWM, Phase Correct, Top = 0xff, Update of OCRA at top, TOV at bottom
     * CLK I/O /8 prescaling
     */
    TCCR0A = (1 << COM0A1) | (1 << COM0B1) | (1 << WGM00);
    TCCR0B = (1 << CS01);
    TIMSK0 = (1 << TOIE0);

    /* Clear on compare match
     * Mode 8: PWM, Phase and Frequence Correct, Top = ICR1
     * CLK I/O no prescaler
     */
    TCCR1A = /*(1 << COM1A1) | */(1 << COM1B1);
    TCCR1B = (1 << WGM13) | (1 << CS10);
    TIMSK1 = (1 << TOIE1);
    ICR1 = PWM_MAX;

    UCSR0A = 0;
    UCSR0B = 0;
    UCSR0C = 0;
    // Serial TXO
    sbi(DDRD, DDD1);
    cbi(PORTD, PORTD1);

#if 1
    UBRR0 = 103;
    UCSR0A = (1 << U2X0);
    UCSR0B = (1 << TXEN0);
    UCSR0C = 6;
    sbi(UCSR0A, TXC0);
#endif

    TWAR = 0x18 << 1;
    TWCR = (1 << TWEA) | (1 << TWEN) | (1 << TWIE);


    for (int i = 0; i < ENCODER_COUNT; ++i)
    {
        gEncoder[i].lastStatus = (PINC >> (i * 2)) & 3;
    }
    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        gMotor[i].encoder = i;
    }

//    CLKPR = 0x80;
//    CLKPR = 0x02;
    sei();

    //setSpeed(0, TIMER1_MAX);
    //setSpeed(1, TIMER1_MAX);
    uint8_t misses = 0;
    uint8_t lastMs = gMs;
    uint8_t data = 0;

//    while (true)
//    {
//        while (!(TWCR & 0x80)) { }
//        print(TWSR);
//        putc(10);
//        putc(13);

//        if (TWSR == 0xa8 || TWSR == 0xb8) TWDR = data++;
//        sbi(TWCR, TWINT);
//    }
    int count = 0;
    while (true)
    {
        uint8_t ms = gMs;
        if ((SPI_PIN & (1 << SPI_SS_PIN))) gSpiData = 0;
        if (lastMs != ms)
        {
//            if (ms - lastMs > 1 || gEncoder[0].encoderError != 0 || gEncoder[1].encoderError != 0 || gEncoder[2].encoderError != 0)
//            {
//                misses += ms - lastMs - 1;
//                print(misses);
//                putc(' ');
//                print(gEncoder[0].encoderError);
//                putc(' ');
//                print(gEncoder[1].encoderError);
//                putc(' ');
//                print(gEncoder[2].encoderError);
//                putc('\r');
//                putc('\n');
//                gEncoder[0].encoderError = 0;
//                gEncoder[1].encoderError = 0;
//                gEncoder[2].encoderError = 0;
//                ms = gMs;
//            }
            lastMs = ms;
        }
        //gMotor[0].targetPos = gMotor[0].targetPos + 5;// << 5;
        gMotor[0].targetPos = gEncoder[1].encoderPos << 5;
//        putc(M1IN1_PWM);
//        putc(0);
//        putc(0);
//        putc(0);
//        putc(0);
//        putc(0);
//        putc(0);
//        putc(0);
        putc(gMotor[0].targetPos);
        putc(gMotor[0].targetPos >> 8);
        putc(gEncoder[0].encoderPos);
        putc(gEncoder[0].encoderPos >> 8);
        putc(gMotor[0].previousError);
        putc(gMotor[0].previousError >> 8);
        putc(gMotor[0].y);
        putc(gMotor[0].y >> 8);
        if (count++ >= 1024)
        {
            count = 0;
            putc(0x12);
            putc(0x34);
            putc(0x56);
            putc(0x78);

        }
//        print(gEncoder[0].encoderPos);
//        putc(' ');
//        print(gEncoder[1].encoderPos);
//        putc(' ');
//        print(gMotor[0].previousError);
//        putc('\r');
//        putc('\n');
    }
    setSpeed(0, 0);
    setSpeed(1, 0);

    return 0;
}
