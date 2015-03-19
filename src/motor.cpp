#if defined(LEFT) || defined(RIGHT)

#include <avr/io.h>
#include <avr/interrupt.h>
//#include <util/atomic.h>
#include <stdint.h>
#include <limits.h>

#include "common.h"
//#include "uart.h"

// /home/thomas/bin/arduino-1.0.6/hardware/tools/avrdude -C/home/thomas/bin/arduino-1.0.6/hardware/tools/avrdude.conf -patmega328p -P/dev/ttyUSB0 -carduino -b57600 -D -Uflash:w:build-cli/NxtMotor.hex
// /home/thomas/bin/arduino-1.0.6/hardware/tools/avrdude -v -C /home/thomas/bin/arduino-1.0.6/hardware/tools/avrdude.conf -c avrispmkII -p m328p -P usb

/* Pin usage
 * PORT  Function       Arduino HW
 * ------------------------------------------------------
 * PD0   PCINT16          RXD               E3A
 * PD1   PCINT17          TXD               E3B
 * PD2   I/O               2
 * PD3   Timer2 B Output   3                M3IN1
 * PD4   I/O               4                M3IN2
 * PD5   Timer0 B Output   5                M3ENA
 * PD6   Timer0 A Output   6                M2IN1
 * PD7   I/O               7                M2IN2
 *
 * PB0   I/O               8n               M1IN2
 * PB1   Timer1 A Output   9                M2ENA
 * PB2   Timer1 B Output  10                M1IN1
 * PB3   Timer2 A Output  11   DBG MOSI     M1ENA
 * PB4   I/O              12   DBG MISO
 * PB5   I/O              13   DBG SCK
 * PB6                   XTAL1
 * PB7                   XTAL2
 *
 * PC0   PCINT8           A0                E2A
 * PC1   PCINT9           A1                E2B
 * PC2   PCINT10          A2                E1A
 * PC3   PCINT11          A3                E1B
 * PC4   PCINT12          A4                SDA
 * PC5   PCINT13          A5                SCL
 * PC6   PCINT13         RESET
 *
 * ADC6
 * ADC7                                     VBAT 1k2 -> MEAS -> 1k2 -> GND
 */

#define TW_BUFFER_SIZE 16    // I2C buffer size = 1 (address) + 1 (Command) + 4 (data)

volatile uint8_t gTwBuffer[TW_BUFFER_SIZE];
volatile uint8_t gTwTxLen = 0;
volatile uint8_t gTwRxLen = 0;


enum MotorFunction { In1, In2, Enable, EncoderA, EncoderB, PWM_In1, PWM_Enable };
enum Pin { B0, B1, B2, B3, B4, B5, B6, B7, C0, C1, C2, C3, C4, C5, C6, C7, D0, D1, D2, D3, D4, D5, D6, D7 };

const uint8_t ADDR[3][7] = {
/*   In1 In2 Enable ENCA ENCB PWM_IN1 PWM_ENA   */
    { B2, B0,  B3,   C2,  C3, (uint16_t)&OCR1BL,(uint16_t)&OCR2A },     // M1
    { D6, D7,  B1,   C0,  C1, (uint16_t)&OCR0A, (uint16_t)&OCR1AL },     // M2
    { D3, D4,  D5,   D0,  D1, (uint16_t)&OCR2B, (uint16_t)&OCR0B },     // M3
};

typedef uint32_t encoder_t;
typedef int32_t controller_t;

struct Encoder
{
    uint8_t lastStatus;
    uint8_t encoderError;
    encoder_t encoderPos;
};

enum class TargetType : uint8_t
{
    Pwm,
    Speed,
    Position,
};


struct Motor
{
    uint8_t encoder;
    TargetType targetType;
    uint32_t target;
    controller_t integralError;
    controller_t previousError;
    controller_t y;
    bool pwmOnStandby;
};


static const uint16_t PWM_MAX = 255;
static const unsigned MOTOR_COUNT = 3;
static const unsigned ENCODER_COUNT = 3;


static volatile uint32_t gMs = 0;

volatile static Motor gMotor[MOTOR_COUNT] = {0};
volatile static Encoder gEncoder[ENCODER_COUNT] = {0};

void setPortBit(uint8_t offset, uint8_t port, bool on)
{
    uint8_t bit = port & 7;
    offset += (port >> 3) * 3;
//    Uart::printChar(on ? 's' : 'r');
//    Uart::printValueHex(offset);
//    Uart::printChar(':');
//    Uart::printValue(bit);
//    Uart::printChar(' ');
    if (on) _SFR_IO8(offset) |= _BV(bit);
    else _SFR_IO8(offset) &= ~_BV(bit);
}

inline void setPin(uint8_t port, bool on)
{
    setPortBit(5, port, on);
}

enum PortType { Input, Output, PullUp };

void setPort(uint8_t port, PortType type)
{
    bool setPort = false;
    bool setDirection = false;
    switch (type)
    {
    case Input:  setPort = false; setDirection = false; break;
    case Output: setPort = false; setDirection = true;  break;
    case PullUp: setPort = true;  setDirection = false; break;
    }
    setPortBit(5, port, setPort);
    setPortBit(4, port, setDirection);
}

void setSpeed(uint8_t motorIndex, int16_t speed)
{
    if (motorIndex >= MOTOR_COUNT) return;
    bool cw = true;
    if (speed < 0)
    {
        cw = false;
        speed = -speed;
    }
    if (speed > PWM_MAX) speed = PWM_MAX;
//    Uart::printValueHex(ADDR[motorIndex][PWM_In1]);
//    Uart::printChar(':');
//    Uart::printValueHex(OCR2A);
//    Uart::printChar(' ');
//    Uart::printValueHex(OCR1B);
//    Uart::print("\r\n");
    if (cw)
    {
        if (gMotor[motorIndex].pwmOnStandby)
        {
            _SFR_MEM8(ADDR[motorIndex][PWM_In1]) = PWM_MAX;
            _SFR_MEM8(ADDR[motorIndex][PWM_Enable]) = speed;
        }
        else
        {
            _SFR_MEM8(ADDR[motorIndex][PWM_In1]) = speed;
            _SFR_MEM8(ADDR[motorIndex][PWM_Enable]) = PWM_MAX;
        }
        setPin(ADDR[motorIndex][In2], false);
    }
    else
    {
        if (gMotor[motorIndex].pwmOnStandby)
        {
            _SFR_MEM8(ADDR[motorIndex][PWM_In1]) = PWM_MAX;
            _SFR_MEM8(ADDR[motorIndex][PWM_Enable]) = PWM_MAX - speed;
        }
        else
        {
            _SFR_MEM8(ADDR[motorIndex][PWM_In1]) = PWM_MAX - speed;
            _SFR_MEM8(ADDR[motorIndex][PWM_Enable]) = PWM_MAX;
        }
        setPin(ADDR[motorIndex][In2], true);
    }
//    Uart::printValueHex(OCR2A);
//    Uart::printChar(' ');
//    Uart::printValueHex(OCR1B);
//    Uart::print("\r\n");
}


void controllerPosition(uint8_t i)
{
    static const int16_t kp = 8, kiShift = 4, kd = 0;
    static const int16_t I_MAX = PWM_MAX >> 2;
    cli();
    controller_t error = gMotor[i].target - gEncoder[gMotor[i].encoder].encoderPos;
    sei();
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

void controllerSpeed(uint8_t i)
{
    static uint8_t counter = 0;
    static const int16_t kp = 8, kiShift = 4;
    static const int16_t I_MAX = PWM_MAX >> 2;
    ++counter;
    if (counter != 10) return;
    counter = 0;

    gMotor[i].previousError += (int32_t)gMotor[i].target;
    cli();
    int32_t error = (uint32_t)gMotor[i].previousError - gEncoder[gMotor[i].encoder].encoderPos;
    sei();
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
    controller_t y = kp * error + (gMotor[i].integralError << kiShift);
    gMotor[i].y = y;
    setSpeed(i, y >> 5);
}


ISR(TIMER0_OVF_vect)
{
    sei();

    ++gMs;
    if ((gMs & 63ULL) != 0) return;

    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        switch (gMotor[i].targetType)
        {
        case TargetType::Position:
            controllerPosition(i);
            break;
        case TargetType::Speed:
            controllerSpeed(i);
            break;
        case TargetType::Pwm:
            int16_t speed = *((int16_t*)&gMotor[i].target);
            setSpeed(i, speed);
            break;
        }
    }
}

ISR(TIMER1_OVF_vect)
{
}

ISR(TIMER2_OVF_vect)
{
}

void evalEncoders()
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
    uint8_t pd = PIND & 0x3;
    uint8_t pc = PINC & 0xf;
    //sei();
    uint8_t now = (pc >> 3) | ((pc & 4) >> 1) | ((pc & 3) << 2) | (pd << 4);
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
}

/* PCINT0-7 */
ISR(PCINT0_vect)
{

}

/* PCINT8-14 */
ISR(PCINT1_vect)
{
    evalEncoders();
}

/* PCINT16-23 */
ISR(PCINT2_vect)
{
    evalEncoders();
}

void executeCommand()
{
    gTwTxLen = 0;
    if (gTwRxLen == 0) return;
#if defined(RIGHT)
    uint8_t idx = gTwBuffer[1];
#elif defined(LEFT)
    uint8_t idx = 2 - gTwBuffer[1];
#endif
    switch (static_cast<Command>(gTwBuffer[0]))
    {
    case Command::SetPwm:
        gMotor[idx].target = *((volatile uint32_t*)(gTwBuffer + 2));
        gMotor[idx].targetType = TargetType::Pwm;
        break;
    case Command::SetSpeed:
        gMotor[idx].target = *((volatile uint32_t*)(gTwBuffer + 2));
        gMotor[idx].targetType = TargetType::Speed;
        gMotor[idx].previousError = gEncoder[gMotor[idx].encoder].encoderPos;
        gMotor[idx].y = 0;
        break;
    case Command::SetPosition:
        gMotor[idx].target = *((volatile uint32_t*)(gTwBuffer + 2));
        gMotor[idx].targetType = TargetType::Position;
        break;
    case Command::Stop:
        gMotor[idx].targetType = TargetType::Pwm;
        gMotor[idx].target = 0;
        break;
    case Command::PwmOnStandby:
        gMotor[idx].pwmOnStandby = gTwBuffer[2] != 0;
        break;
    case Command::GetPosition:
        cli();
        *reinterpret_cast<volatile uint32_t*>(gTwBuffer) = gEncoder[idx].encoderPos;
//        if (idx == 0) *reinterpret_cast<volatile uint32_t*>(gTwBuffer) = gEncoder[0].encoderPos;
//        if (idx == 1) *reinterpret_cast<volatile uint32_t*>(gTwBuffer) = gMotor[0].previousError;
//        if (idx == 2) *reinterpret_cast<volatile uint32_t*>(gTwBuffer) = gMotor[0].y;
        sei();
        gTwTxLen = 4;
        break;
    }
}

ISR(TWI_vect)
{
    static uint8_t offset = 0;

//    Uart::printValueHex(TWSR);
//    Uart::print(" ");

    switch (TWSR & 0xf8)
    {
    case 0xa8:  // SLA+R received
    case 0xb0:  // Arbitration lost, SLA+R received
        offset = 0;
        // Fall through
    case 0xb8:  // data transmitted + ACK
        if (offset < TW_BUFFER_SIZE) TWDR = gTwBuffer[offset++];
        else TWDR = 0xff;
        if (gTwTxLen == offset)
        {
            TWCR &= ~_BV(TWEA);
        }
        break;
    case 0x60:  // SLA+W received
    case 0x68:  // Arbitration lost, SLA+W received
    case 0x70:  // General call received
    case 0x78:  // Arbitration lost, General call received
        gTwRxLen = 0;
        break;
    case 0x80:  // Data received + ACK
    case 0x88:  // Data received + NACK
    case 0x90:  // General call: Data received + ACK
    case 0x98:  // General call: Data received + NACK
        if (gTwRxLen < TW_BUFFER_SIZE) gTwBuffer[gTwRxLen++] = TWDR;
        break;
    case 0xa0:  // STOP or REPEATED START
        executeCommand();
        break;
    case 0xc0:  // data transmitted + NACK
    case 0xc8:  // last data transmitted + ACK
        // We are done
        TWCR |= _BV(TWEA);
        break;
    }
    TWCR |= _BV(TWINT);
}


void init()
{
#if defined(LEFT)
    TWAR = TW_LEFT_ADDR;
#elif defined(RIGHT)
    TWAR = TW_RIGHT_ADDR;
#endif
    TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);

    PCICR = _BV(PCIE1) | _BV(PCIE2);
    // Enable pin change irq's for rotery encoders
    PCMSK0 = 0;
    PCMSK1 = _BV(PCINT8) | _BV(PCINT9) | _BV(PCINT10) | _BV(PCINT11);
    PCMSK2 = _BV(PCINT16) | _BV(PCINT17);

    /* TIMER0: Clear on compare match
     * Mode 1: PWM, Phase Correct, Top = 0xff, Update of OCRA at top, TOV at bottom
     * CLK I/O /no prescaling -> 39kHz
     */
    TCCR0A = _BV(COM0A1) | _BV(COM0B1) | _BV(WGM00);
    TCCR0B = _BV(CS00);
    TIMSK0 = _BV(TOIE0);

    /* TIMER1: Clear on compare match
     * Mode 8: PWM, Phase and Frequence Correct, Top = ICR1
     * CLK I/O no prescaler -> 39kHz
     */
    TCCR1A = _BV(COM1A1) | _BV(COM1B1);
    TCCR1B = _BV(WGM13) | _BV(CS10);
    TIMSK1 = 0;//_BV(TOIE1);
    ICR1 = PWM_MAX;

    /* TIMER2: Clear on compare match
     * Mode 1: PWM, Phase Correct, Top = 0xff, Update of OCRA at top, TOV at bottom
     * CLK I/O /no prescaling -> 39kHz
     */
    TCCR2A = _BV(COM2A1) | _BV(COM2B1) | _BV(WGM20);
    TCCR2B = _BV(CS20);
    TIMSK2 = 0;//_BV(TOIE2);

    evalEncoders();

    //Uart::init();
    //Uart::print("\r\n");

    for (int i = 0; i < MOTOR_COUNT; ++i)
    {
        setPort(ADDR[i][In1], Output);
        setPort(ADDR[i][In2], Output);
        setPin(ADDR[i][In2], false);
        setPort(ADDR[i][Enable], Output);
        setPort(ADDR[i][EncoderA], PullUp);
        setPort(ADDR[i][EncoderB], PullUp);
        _SFR_MEM8(ADDR[i][PWM_In1]) = 0;
        _SFR_MEM8(ADDR[i][PWM_Enable]) = 0;

        gMotor[i].encoder = i;
        gEncoder[i].encoderError = 0;
        gEncoder[i].encoderPos = 0;
    }

    sei();

}

void work()
{

}

#endif
