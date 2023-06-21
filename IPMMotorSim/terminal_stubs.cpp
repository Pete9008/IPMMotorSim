#include <QFile>
#include <QTextStream>
#include "terminal.h"
#include "params.h"
#include "terminalcommands.h"
#include "my_fp.h"

Terminal* Terminal::defaultTerminal;

Terminal t;
static Terminal* terminal = &t;

Terminal::Terminal()
{
   loggingEnabled = false;
   firstSend = true;
   curBuf = 0;
   curIdx = 0;
   defaultTerminal = this;
}

void Terminal::EnableLogging(bool enable)
{
   loggingEnabled = enable;
}

void Terminal::SetNodeId(uint8_t id)
{
    (void)id;
}

static QFile logFile("logfile.bin");

#if __has_include("binarylogging.h")
bool Terminal::SendCurrentBuffer(uint32_t len, bool wait)
{
    (void)wait;
    if(!logFile.isOpen())
    {
        logFile.open(QFile::WriteOnly);
        TerminalCommands::PrintParamsJson(this,nullptr);
        logFile.write(BINLOG_JSON, sizeof(BINLOG_JSON));
    }

    if(logFile.isOpen())
    {
        logFile.write(outBuf[curBuf], len);
    }
}

LogStruct* Terminal::GetWriteBuffer(uint32_t length)
{
   LogStruct* retVal = nullptr;

   if(loggingEnabled)
   {
      if(length < (bufSize - curIdx)) //will it fit - note may leave buffer with 0 bytes left
      {
         retVal = (LogStruct*)(&outBuf[curBuf][curIdx]);
         curIdx += length;
      }
      else
      { //won't fit so start new DMA if possible and switch to start of other buffer
         if(SendCurrentBuffer(curIdx, false))
         {
            retVal = (LogStruct*)(&outBuf[curBuf][0]);
            curIdx = length;
         }
      }
   }
   //otherwise no buffer is available so return default null
   return retVal;
}

void TerminalCommands::PrintParamsJson(Terminal* term, char *arg)
{
   (void)term;
   (void)arg;
   //arg = my_trim(arg);
    QTextStream out(&logFile);
    QString str;

   const Param::Attributes *pAtr;
   char comma = ' ';
   bool printHidden = false;
   uint32_t spotIdx = 0;

   out << "{";
   for (uint32_t idx = 0; idx < Param::PARAM_LAST; idx++)
   {
      //int canId, canOffset, canLength;
      //bool isRx;
      //s32fp canGain;
      pAtr = Param::GetAttrib((Param::PARAM_NUM)idx);

      if ((Param::GetFlag((Param::PARAM_NUM)idx) & Param::FLAG_HIDDEN) == 0 || printHidden)
      {
         //fprintf(term, "%c\r\n   \"%s\": {\"unit\":\"%s\",\"value\":%f,",comma, pAtr->name, pAtr->unit, Param::Get((Param::PARAM_NUM)idx));
         str.asprintf("%c\r\n   \"%s\": {\"unit\":\"%s\",\"value\":%.2f,",comma, pAtr->name, pAtr->unit, Param::GetFloat((Param::PARAM_NUM)idx));
         out << str;

         if (Param::IsParam((Param::PARAM_NUM)idx))
         {
            //fprintf(term, "\"isparam\":true,\"minimum\":%f,\"maximum\":%f,\"default\":%f,\"category\":\"%s\",\"i\":%d}",
            //       pAtr->min, pAtr->max, pAtr->def, pAtr->category, idx);
            str.asprintf("\"isparam\":true,\"minimum\":%.2f,\"maximum\":%.2f,\"default\":%.2f,\"category\":\"%s\",\"i\":%d}", FP_TOFLOAT(pAtr->min), FP_TOFLOAT(pAtr->max), FP_TOFLOAT(pAtr->def), pAtr->category, idx);
            out << str;
         }
         else
         {
            //fprintf(term, "\"isparam\":false}");
             str.asprintf("\"isparam\":false,\"si\":%d}", spotIdx++);
             out << str;
         }
         comma = ',';
      }
   }
   //fprintf(term, "\r\n}\r\n");
   out << "\r\n}\r\n";
}
#endif
