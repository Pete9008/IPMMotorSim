#ifndef TERMINAL_TESTSTUBS_H
#define TERMINAL_TESTSTUBS_H
#include <stdint.h>

class Terminal
{
public:
   Terminal();
   void SendBinary(uint8_t* data, uint32_t length);
   static Terminal* defaultTerminal;
   void BinaryLogging(char *arg);
   bool BinLoggingEnabled();

private:
   bool binLoggingEnabled = false;
};

#endif // CPP_TESTSTUBS_H
