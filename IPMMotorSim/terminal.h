#ifndef TERMINAL_TESTSTUBS_H
#define TERMINAL_TESTSTUBS_H
#include <stdint.h>
#if __has_include("binarylogging.h")
#include "binarylogging.h"
#endif

class Terminal
{
public:
   Terminal();
   #if __has_include("binarylogging.h")
   LogStruct* GetWriteBuffer(uint32_t length);
   #endif
   bool SendCurrentBuffer(uint32_t len, bool wait);
   static Terminal* defaultTerminal;
   void EnableLogging(bool enable);

private:
   bool loggingEnabled = false;
   static const int bufSize = 128;
   uint8_t curBuf;
   uint32_t curIdx;
   bool firstSend;
   char outBuf[2][bufSize]; //double buffering
};

#endif // CPP_TESTSTUBS_H
