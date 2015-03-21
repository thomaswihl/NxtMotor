#include <avr/io.h>
#include <avr/interrupt.h>

#include "uart.h"

namespace Uart
{
volatile char gTxBuffer[TX_BUFFER_SIZE];
volatile uint8_t gTxReadOffset = 0;
volatile uint8_t gTxWriteOffset = 0;

volatile char gRxBuffer[RX_BUFFER_SIZE];
volatile uint8_t gRxLen = 0;
volatile bool gLineReady = false;

ISR(USART_RX_vect)
{
    uint8_t data = UDR0;
    if (gLineReady) return;
    if (data == '\r')
    {
        gLineReady = true;
    }
    else if (data == 8 || data == 0x7f)
    {
        if (gRxLen > 0)
        {
            --gRxLen;
            print("\x1b[D\x1b[K");
        }
    }
    else
    {
        gRxBuffer[gRxLen++] = data;
        printChar(data);
        if (data == 27) gLineReady = true;

    }
}

ISR(USART_UDRE_vect)
{
    if (gTxReadOffset == gTxWriteOffset)
    {
        // nothing to write, disable interrupt and return
        UCSR0B &= ~_BV(UDRIE0);
        return;
    }
    uint8_t read = gTxReadOffset;
    UDR0 = gTxBuffer[read++];
    UCSR0A |= _BV(TXC0);
    if (read >= TX_BUFFER_SIZE) read = 0;
    gTxReadOffset = read;
}

void init()
{
    // UART0: 115200, Async normal mode
    UBRR0 = 10; // 20000000 / (16 * 115200) - 1
    //UCSR0A = 32; // Single speed
    UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0); // Enable interrupts, Rx and Tx
    UCSR0C = _BV(UCSZ01) | _BV(UCSZ00); // Async, no parity, 1 stop bit, 8 data bits
}

void printChar(char c)
{
    uint8_t write = gTxWriteOffset;
    gTxBuffer[write++] = c;
    if (write >= TX_BUFFER_SIZE) write = 0;
    while (write == gTxReadOffset)
    {
        sei();
    }
    gTxWriteOffset = write;
    if ((UCSR0B & _BV(UDRIE0)) == 0) UCSR0B |= _BV(UDRIE0);
}

void print(const char* string)
{
    while (*string != 0)
    {
        printChar(*string++);
    }
}

void print(const char* string, uint8_t len)
{
    while (len != 0)
    {
        printChar(*string++);
        --len;
    }
}


bool lineReady()
{
    return gLineReady;
}


const char *getLine(uint8_t &len)
{
    len = gRxLen;
    return const_cast<const char*>(gRxBuffer);
}


void clearLine()
{
    gRxLen = 0;
    gLineReady = false;
}

bool transmitBufferEmpty()
{
    if ((UCSR0A & _BV(TXC0)) == 0) return false;
    return gTxReadOffset == gTxWriteOffset;
}


}   /* namespace Uart */
