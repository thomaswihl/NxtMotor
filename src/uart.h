#ifndef UART_H
#define UART_H

#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 32

namespace Uart
{

void init();
void printChar(char c);
void print(const char* string);
void print(const char* string, uint8_t len);

template<typename T>
void printValue(T value)
{
    char buf[20]; // 20 digits for uint64, 19 + 1 (sign) = 20 for int64
    if (value < 0)
    {
        printChar('-');
        value = -value;
    }
    uint8_t len = 0;
    do
    {
        buf[len++] = '0' + (value % 10);
        value /= 10;
    }   while (value != 0);
    for (uint8_t i = len; i > 0; --i) printChar(buf[i - 1]);
}

template<typename T>
void printValueHex(T value)
{
    static const char HEX_DIGIT[] = { '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f' };
    print("0x");
    for (int8_t i = sizeof(T) * 2 - 1; i >= 0; --i)
    {
        printChar(HEX_DIGIT[(value >> (i * 4)) & 0xf]);
    }
}


bool lineReady();
const char* getLine(uint8_t& len);
void clearLine();

}
#endif // UART_H
