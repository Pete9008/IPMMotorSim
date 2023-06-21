/*
 * This file is part of the IPMMotorSim project
 *
 * Copyright (C) 2022 Pete9008 <openinverter.org>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "inc_encoder.h"
#include "params.h"
#include "my_math.h"
#include "pwmgeneration.h"
#include "anain.h"
#include "digio.h"
#include "foc.h"


#define TWO_PI            65536
#define STABLE_ANGLE      ((10 * TWO_PI) / 360)

//dummy globals
#ifdef STM32F1
uint32_t rcc_apb2_frequency = 72000000;
HWREV hwRev;
#endif

#ifdef STM32F4
uint32_t rcc_apb2_frequency = 84000000;
#endif

//test harness interface variables
volatile uint16_t g_input_angle = 0;
#ifdef STM32F1
volatile double g_il1_input = 0;
volatile double g_il2_input = 0;
#endif

//OpenInverter variable
static volatile uint16_t angle = 0;
static int32_t resolverMin = 0, resolverMax = 0, startupDelay=0;
static uint32_t fullTurns = 0;
static bool seenNorthSignal = true;
static int32_t distance = 0;
static int32_t turnsSinceLastSample = 0;
#ifdef STM32F1
static u32fp lastFrequency = 0;
uint16_t PwmGeneration::slipIncr;
#else
static float lastFrequency = 0.0f;
#endif
static int32_t detectedDirection = 0;

static uint16_t lastAngle = 0;
static int poleCounter = 0;

void testStubsClearEncoder(void)
{
    angle = 0;
    resolverMin = 0;
    resolverMax = 0;
    startupDelay=0;
    fullTurns = 0;
    seenNorthSignal = true;
    distance = 0;
    turnsSinceLastSample = 0;
    lastFrequency = 0;
    detectedDirection = 0;
    lastAngle = 0;
    poleCounter = 0;
}

#ifdef STM32F1
bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}
#endif

#ifdef STM32F1
void Encoder::UpdateRotorAngle(int dir)
{
    (void)dir;

    angle = g_input_angle;
    UpdateTurns(angle, lastAngle);

    if (lastAngle <= (TWO_PI / 2) && angle > (TWO_PI / 2))
    {
       if (poleCounter == 0)
       {
          fullTurns++;
          poleCounter = Param::GetInt(Param::respolepairs);
       }
       else
       {
          poleCounter--;
       }
    }

    lastAngle = angle;
}


void Encoder::UpdateTurns(uint16_t angle, uint16_t lastAngle)
{
   int signedDiff = (int)angle - (int)lastAngle;
   int absDiff = ABS(signedDiff);
   int sign = signedDiff < 0 ? -1 : 1;

   if (absDiff > (TWO_PI / 2)) //wrap detection
   {
      sign = -sign;
      signedDiff += sign * TWO_PI;
      absDiff = ABS(signedDiff);
   }

   turnsSinceLastSample += signedDiff;
}
#else
void Encoder::UpdateRotorAngle(uint32_t updateRate, uint32_t runEveryNth)
{
   static uint32_t freqUpdateCount = 0;
   static uint32_t stableCount = 1;
   angle = g_input_angle;

   int signedDiff = (int)angle - (int)lastAngle;
   int absDiff = ABS(signedDiff);
   int sign = signedDiff < 0 ? -1 : 1;

   if (absDiff > (TWO_PI / 2)) //wrap detection
   {
      sign = -sign;
      signedDiff += sign * TWO_PI;
      absDiff = ABS(signedDiff);
   }

   turnsSinceLastSample += signedDiff;

   startupDelay = startupDelay > 0 ? startupDelay - 1 : 0;
   lastAngle = angle;

   freqUpdateCount++;
   if(freqUpdateCount >= runEveryNth)
   {
     freqUpdateCount = 0;
     distance += turnsSinceLastSample;

     int absTurns = ABS(turnsSinceLastSample);
     if (absTurns > STABLE_ANGLE)
     {
        lastFrequency = ((float)(updateRate * absTurns)) / (float)(TWO_PI*stableCount);
        detectedDirection = turnsSinceLastSample > 0 ? 1 : -1;
        turnsSinceLastSample = 0;
        stableCount = 1;
     }
     else
     {
        if(++stableCount > 20)
        {
           lastFrequency = 0.0f;
           stableCount = 1;
        }
     }
   }
}
#endif

#ifdef STM32F1
void Encoder::UpdateRotorFrequency(int callingFrequency)
{
    distance += turnsSinceLastSample;

    int absTurns = ABS(turnsSinceLastSample);
    if (startupDelay == 0 && absTurns > STABLE_ANGLE)
    {
     lastFrequency = (callingFrequency * absTurns) / FP_TOINT(TWO_PI);

     detectedDirection = turnsSinceLastSample > 0 ? 1 : -1;
    }
    else
    {
     lastFrequency = 0;
    }
    turnsSinceLastSample = 0;

}
#endif

uint16_t Encoder::GetRotorAngle()
{
   return angle;
}

#ifdef STM32F1
u32fp Encoder::GetRotorFrequency()
#else
float Encoder::GetRotorFrequency()
#endif
{
   return lastFrequency;
}



int Encoder::GetRotorDirection()
{
   return detectedDirection;
}

void timer_disable_break_main_output(int i)
{
    (void)i;
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
      case Param::canspeed:
         break;
      case Param::throtmax:
      case Param::throtmin:
      case Param::idcmin:
      case Param::idcmax:
      case Param::offthrotregen:
         break;
      case Param::nodeid:
         break;
      default:
         #ifdef STM32F1
            PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
         #endif
         PwmGeneration::SetPolePairRatio(Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs));

#if CONTROL == CTRL_FOC
#ifdef STM32F1
         PwmGeneration::SetControllerGains(Param::GetInt(Param::curkp), Param::GetInt(Param::curki));
#else
         PwmGeneration::SetControllerGains(Param::GetFloat(Param::curkp), Param::GetFloat(Param::curki));
#endif
         FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld)/1000, Param::GetFloat(Param::fluxlinkage)/1000);
#endif // CONTROL
         break;
   }
}

#ifdef STM32F1
void Encoder::SetPwmFrequency(uint32_t frq) {(void)frq;}
#endif

int printf(const char *format, ...) {(void)format;return 0;}

#undef ANA_IN_ENTRY
#define ANA_IN_ENTRY(name, port, pin) AnaIn AnaIn::name(__COUNTER__);
ANA_IN_LIST
#undef ANA_IN_ENTRY

uint8_t AnaIn::channel_array[ANA_IN_COUNT];
uint16_t AnaIn::values[NUM_SAMPLES*ANA_IN_COUNT];

#ifdef STM32F1
uint16_t AnaIn::Get()
{
    double iVal = 0;
    if(this == &AnaIn::il1)
        iVal = g_il1_input;
    else if(this == &AnaIn::il2)
        iVal = g_il2_input;

    iVal = iVal + 2048;
    if(iVal > 4096)
        return 4096;
    else if (iVal < 0)
        return 0;
    else
        return uint16_t(iVal);
}
#endif

void AnaIn::Configure(uint32_t port, uint8_t pin)
{
    (void)port;
    (void)pin;
}

#undef DIG_IO_ENTRY
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo DigIo::name;
DIG_IO_LIST


