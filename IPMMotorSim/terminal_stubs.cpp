#include <QFile>
#include <QTextStream>
#include "terminal.h"
#include "params.h"
#include "terminalcommands.h"
#include "my_fp.h"

Terminal* Terminal::defaultTerminal;

//Terminal t;
//static Terminal* terminal = &t;

Terminal::Terminal()
{
   binLoggingEnabled = false;
   defaultTerminal = this;
}

//todo - pull from SOMETHING_LIST definition in pwmgeneration
const char binHeader[] =  "{\"01\":{\"name\":\"count\",\"size\":8,\"scale\":1,\"signed\":0},\
\"02\":{\"name\":\"angle\",\"size\":14,\"scale\":4,\"signed\":0},\
\"03\":{\"name\":\"idc\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"04\":{\"name\":\"i1\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"05\":{\"name\":\"i2\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"06\":{\"name\":\"pwm1\",\"size\":14,\"scale\":1,\"signed\":0},\
\"07\":{\"name\":\"opmode\",\"size\":2,\"scale\":1,\"signed\":0},\
\"08\":{\"name\":\"pwm2\",\"size\":14,\"scale\":1,\"signed\":0},\
\"09\":{\"name\":\"desat\",\"size\":2,\"scale\":1,\"signed\":0},\
\"10\":{\"name\":\"pwm3\",\"size\":14,\"scale\":1,\"signed\":0},\
\"11\":{\"name\":\"iqref\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"12\":{\"name\":\"idref\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"13\":{\"name\":\"ifw\",\"size\":14,\"scale\":0.25,\"signed\":1},\
\"14\":{\"name\":\"uq\",\"size\":16,\"scale\":2,\"signed\":1},\
\"15\":{\"name\":\"ud\",\"size\":16,\"scale\":2,\"signed\":1},\
\"16\":{\"name\":\"csum\",\"size\":8,\"scale\":1,\"signed\":0}}";

void Terminal::BinaryLogging(char *arg)
{
   if(arg[0] == '1')
      binLoggingEnabled = true;
   else
      binLoggingEnabled = false;
}

bool Terminal::BinLoggingEnabled()
{
   return binLoggingEnabled;
}

static QFile logFile("logfile.bin");

void Terminal::SendBinary(uint8_t* data, uint32_t len)
{
    if(!logFile.isOpen())
    {
        logFile.open(QFile::WriteOnly);
        TerminalCommands::PrintParamsJson(this,nullptr);
        logFile.write(binHeader, sizeof(binHeader));
    }

    if(logFile.isOpen())
    {
        logFile.write(reinterpret_cast<char *>(data), len);
    }
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
             out << "\"isparam\":false}";
         }
         comma = ',';
      }
   }
   //fprintf(term, "\r\n}\r\n");
   out << "\r\n}\r\n";
}
