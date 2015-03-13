#if defined(CENTER)

#include <avr/io.h>
#include <avr/interrupt.h>
#include <compat/deprecated.h>

#include "common.h"
#include "uart.h"

/* Pin usage
 * PORT  Function       Arduino HW
 * ------------------------------------------------------
 * PD0   PCINT16          RXD               P19.6
 * PD1   PCINT17          TXD               P19.5
 * PD2   I/O               2                P19.8
 * PD3   Timer2 B Output   3                P19.7
 * PD4   I/O               4                PWR_ONOFF
 * PD5   Timer0 B Output   5                BOARD_POWER
 * PD6   Timer0 A Output   6                P20.1
 * PD7   I/O               7                P20.2
 *
 * PB0   I/O               8n               EXTCLK
 * PB1   Timer1 A Output   9                P20.3
 * PB2   Timer1 B Output  10                P20.4
 * PB3   Timer2 A Output  11   DBG MOSI
 * PB4   I/O              12   DBG MISO
 * PB5   I/O              13   DBG SCK
 * PB6                   XTAL1
 * PB7                   XTAL2
 *
 * PC0   PCINT8           A0                P19.2
 * PC1   PCINT9           A1                P19.1
 * PC2   PCINT10          A2                P19.4
 * PC3   PCINT11          A3                P19.3
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
volatile bool gTwReady = false;
volatile uint8_t gRequestedMotorPos = -1;

void printPrompt()
{
    static uint8_t counter = 0;
    Uart::print("\r\n# ");
//    Uart::printValueHex(counter++);
//    if (counter > 80) PORTD = 0;
}


bool twResetBus()
{
#define DATA 4
#define CLOCK 5
    if ((PINC & _BV(DATA)) != 0 && ((PINC & _BV(CLOCK)) != 0)) return true;
    DDRC &= ~_BV(DATA);
    DDRC |= _BV(CLOCK);

    TWCR &= ~(_BV(TWEN) | _BV(TWSTA) | _BV(TWSTO));
    Uart::print("\r\nBUS RESET\r\n");
    PORTC &= ~_BV(CLOCK);
    for (uint8_t i = 10; i > 0 && (PINC & _BV(DATA)) == 0; --i)
    {
        DDRC |= _BV(CLOCK);
        // This are 3 instructions, by 20MHz / 100kHz = 200 / 3 = 66
        for (uint8_t j = 66; j > 0; --j) asm("nop");
        DDRC &= ~_BV(CLOCK);
        for (uint8_t j = 66; j > 0; --j) asm("nop");
    }
    TWCR |= _BV(TWEN);
    return (PINC & _BV(DATA)) != 0 && ((PINC & _BV(CLOCK)) != 0);
}

void twStart()
{
    if (!twResetBus())
    {
        Uart::print("\r\nBUS BLOCKED\r\n");
        return;
    }
    TWCR |= _BV(TWINT) | _BV(TWEA) | _BV(TWSTA);
}

void sendMotorData(uint8_t motor, Command cmd, uint32_t data)
{
    gTwBuffer[0] = (motor < 3) ? TW_RIGHT_ADDR : TW_LEFT_ADDR;
    gTwBuffer[1] = static_cast<uint8_t>(cmd);
    gTwBuffer[2] = (motor < 3) ? motor : motor - 3;
    gTwBuffer[3] = data;
    gTwBuffer[4] = data >> 8;
    gTwBuffer[5] = data >> 16;
    gTwBuffer[6] = data >> 24;
    gTwRxLen = 0;
    gTwTxLen = 7;
    twStart();
}

void receiveMotorData(uint8_t motor, Command cmd)
{
    gTwBuffer[0] = (motor < 3) ? TW_RIGHT_ADDR : TW_LEFT_ADDR;
    gTwBuffer[1] = static_cast<uint8_t>(cmd);
    gTwBuffer[2] = (motor < 3) ? motor : motor - 3;
    gTwRxLen = 4;
    gTwTxLen = 3;
    twStart();
}


ISR(TWI_vect)
{
    static uint8_t offset = 0;
//    Uart::printValueHex(TWSR);
//    Uart::print(" ");
    switch (TWSR & 0xf8)
    {
    case 0x08:  // START transmitted
        offset = 1;
        TWDR = gTwBuffer[0];    // master transmit mode
        TWCR &= ~(_BV(TWSTA) | _BV(TWSTO));
        // TWCR |= _BV(TWINT); // Is done by instruction before
        break;
    case 0x10:  // repeated START transmitted
        TWDR = gTwBuffer[0] | 1;    // master receive mode
        TWCR &= ~(_BV(TWSTA) | _BV(TWSTO));
        // TWCR |= _BV(TWINT); // Is done by instruction before
        break;
    case 0x18:  // SLA+W transmitted + ACK
    case 0x28:  // Data transmitted + ACK
        if (offset == gTwTxLen)
        {
            if (gTwRxLen > 0)
            {
                TWCR |= _BV(TWINT) | _BV(TWSTA);    // Repeated start, as we need to receive data
            }
            else
            {
                TWCR |= _BV(TWINT) | _BV(TWSTO);    // all done, stop
            }
        }
        else
        {
            TWDR = gTwBuffer[offset++]; // send next data byte
            TWCR |= _BV(TWINT);
        }
        break;
    case 0x40:  // SLA+R transmitted + ACK
        offset = 0;
        TWCR |= _BV(TWINT);
        break;
    case 0x50:  // Data received + ACK
    {
        uint8_t twcr = TWCR | _BV(TWINT);
        if (offset < gTwRxLen) gTwBuffer[offset++] = TWDR;
        else twcr |= _BV(TWSTO);
        if (offset == gTwRxLen - 1) twcr &= ~_BV(TWEA);
        TWCR = twcr;
    }   break;
    case 0x58:  // Data received + NACK
        if (offset < gTwRxLen) gTwBuffer[offset++] = TWDR;
        gTwRxLen = offset;
        TWCR |= _BV(TWINT) | _BV(TWSTO);
        gTwReady = true;
        break;
//    case 0x20:  // SLA+w transmitted + NACK
//    case 0x30:  // Data transmitted + NACK
//    case 0x38:  // Arbitration lost
//    case 0x48:  // SLA+R transmitted + NACK
    default:    // something went wrong, stop
        Uart::print("I2C error: ");
        Uart::printValueHex(TWSR);
        Uart::print("\r\n");
        twResetBus();
        break;
    }
}

ISR(TIMER2_COMPA_vect)
{
    static uint16_t ms = 0;
    ++ms;
    if ((ms % 1000) == 0)
    {
        gRequestedMotorPos = 0;
        receiveMotorData(gRequestedMotorPos, Command::GetPosition);
    }
}

void init()
{
    // POWER_ONOFF + BOARD_POWER
    PORTD = 0x30;
    DDRD = 0x30;

    Uart::init();

    // I2C: 400kHz
    TWBR = 17; // (20MHz / 400kHz - 16) / 2 = TWBR * prescalar = 17 * 1
    TWCR = _BV(TWEA) | _BV(TWEN) | _BV(TWIE);  // Enable I2C, acknowledge and interrupts
    TWSR = 0; // prescaler = 1

    /* TIMER2: Clear on compare match
     * Mode 2: CTC, TOP = OCRA
     * CLK I/O /8 prescaling
     */
    TCCR2A = _BV(WGM21);
    TCCR2B = _BV(CS22) | _BV(CS21) | _BV(CS20);
    TIMSK2 = _BV(OCIE2A);
    OCR2A = 9;  // fclk / (2 * prescaler * freq) - 1 = 20MHz / (2 * 1024 * 1kHz) - 1 = 9


    sei();
}


void printMotorUsage()
{
    Uart::print("m[1-6][sp%be]{PARAM}");
}

void cmdMotor(const char* line, uint8_t len)
{
    if (len < 2)
    {
        printMotorUsage();
        return;
    }
    uint8_t motor = line[1] - '1';
    Command cmd = Command::GetPosition;
    if (len >= 3)
    {
        uint32_t param = 0;
        for (uint8_t i = 3; i < len; ++i)
        {
            param *= 10;
            param += line[i] - '0';
        }
        switch (line[2])
        {
        case 's':
            cmd = Command::SetSpeed;
            break;
        case 'p':
            cmd = Command::SetPosition;
            break;
        case '%':
            param = param * 255 / 100;
            cmd = Command::SetPwm;
            break;
        case 'b':
            cmd = Command::Stop;
            break;
        case 'e':
            cmd = Command::PwmOnStandby;
            break;
        }
        sendMotorData(motor, cmd, param);
    }
    else
    {
        receiveMotorData(motor, Command::GetPosition);
    }
}

void printPinUsage()
{
    Uart::print("p[bcd][0-7]{[01i]}");
}

void cmdPin(const char* line, uint8_t len)
{
    static volatile uint8_t* PIN[] = { &PINB, &PINC, &PIND };
    if (len != 3 && len != 4)
    {
        Uart::print("Wrong len: ");
        Uart::printValue(len);
        Uart::print("\r\n");
        printPinUsage();
        return;
    }
    uint8_t portIndex = line[1] - 'b';
    uint8_t pinIndex = line[2] - '0';
    uint8_t action = (len == 4) ? line[3] : 0;
    if (portIndex > 2 || pinIndex > 7)
    {
        Uart::print("Wrong port: ");
        Uart::printValue(portIndex);
        Uart::print("\r\n");
        printPinUsage();
        return;
    }
    volatile uint8_t* pin = PIN[portIndex];
    volatile uint8_t* ddr = pin + 1;
    volatile uint8_t* port = ddr + 1;
    switch (action)
    {
    case 0:
        Uart::printValue((*pin & _BV(pinIndex)) != 0);
        break;
    case 'i':
        *ddr &= ~_BV(pinIndex);
        *port &= ~_BV(pinIndex);
        break;
    case '0':
        *ddr |= _BV(pinIndex);
        *port &= ~_BV(pinIndex);
        break;
    case '1':
        *ddr |= _BV(pinIndex);
        *port |= _BV(pinIndex);
        break;
    default:
        Uart::print("Wrong action: ");
        Uart::printValue(action);
        Uart::print("\r\n");
        printPinUsage();
        return;
    }
}

void work()
{
    if (Uart::lineReady())
    {
        uint8_t len;
        const char* line = Uart::getLine(len);
        if (len > 0)
        {
            Uart::print("\r\n");
            switch (line[0])
            {
            case 'm':
                cmdMotor(line, len);
                break;
            case 'p':
                cmdPin(line, len);
                break;
            case 27:
                PORTD = 0;
                break;
            }
        }
        Uart::clearLine();
        printPrompt();
    }
    if (gTwReady)
    {
        if (gRequestedMotorPos >= 0)
        {
            Uart::printValue(*(volatile uint32_t*)gTwBuffer);
            Uart::printChar(' ');
            ++gRequestedMotorPos;
            if (gRequestedMotorPos < 6)
            {
                receiveMotorData(gRequestedMotorPos, Command::GetPosition);
            }
            else
            {
                gRequestedMotorPos = -1;
                Uart::print("\r");
            }
        }
        else
        {
            Uart::print("\r\ni2c:");
            for (uint8_t i = 0; i < gTwRxLen; ++i)
            {
                Uart::printChar(' ');
                Uart::printValueHex(gTwBuffer[i]);
            }
            printPrompt();
        }
        gTwReady = false;
    }
}

#endif

