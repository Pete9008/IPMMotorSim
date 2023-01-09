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
uint32_t rcc_apb2_frequency = 72000000;
HWREV hwRev;

//test harness interface variables
volatile uint16_t g_input_angle = 0;
volatile double g_il1_input = 0;
volatile double g_il2_input = 0;

//OpenInverter variable
static volatile uint16_t angle = 0;
static int32_t resolverMin = 0, resolverMax = 0, startupDelay=0;
static uint32_t fullTurns = 0;
static bool seenNorthSignal = true;
static int32_t distance = 0;
static int32_t turnsSinceLastSample = 0;
static u32fp lastFrequency = 0;
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

bool Encoder::SeenNorthSignal()
{
   return seenNorthSignal;
}

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

uint16_t Encoder::GetRotorAngle()
{
   return angle;
}

u32fp Encoder::GetRotorFrequency()
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
         PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
         PwmGeneration::SetPolePairRatio(Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs));

         #if CONTROL == CTRL_FOC
         PwmGeneration::SetControllerGains(Param::GetInt(Param::curkp), Param::GetInt(Param::curki));
         FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld)/1000, Param::GetFloat(Param::fluxlinkage)/1000);
         #endif // CONTROL
         break;
   }
}

void Encoder::SetPwmFrequency(uint32_t frq) {(void)frq;}

int printf(const char *format, ...) {(void)format;return 0;}

#undef ANA_IN_ENTRY
#define ANA_IN_ENTRY(name, port, pin) AnaIn AnaIn::name(__COUNTER__);
ANA_IN_LIST
#undef ANA_IN_ENTRY

uint8_t AnaIn::channel_array[ANA_IN_COUNT];
uint16_t AnaIn::values[NUM_SAMPLES*ANA_IN_COUNT];

uint16_t AnaIn::Get()
{
    double iVal = 0;;
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

void AnaIn::Configure(uint32_t port, uint8_t pin)
{
    (void)port;
    (void)pin;
}

#undef DIG_IO_ENTRY
#define DIG_IO_ENTRY(name, port, pin, mode) DigIo DigIo::name;
DIG_IO_LIST


