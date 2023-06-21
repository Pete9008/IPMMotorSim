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

#include "params.h"
#include "inc_encoder.h"
#include "throttle.h"
#include "foc.h"
#include "pwmgeneration.h"
#include "digio.h"
#include "terminal.h"
//#include "stm32_can.h"


/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
   switch (paramNum)
   {
   #if CONTROL == CTRL_SINE
      case Param::fslipspnt:
         PwmGeneration::SetFslip(Param::Get(Param::fslipspnt));
         break;
      case Param::ampnom:
         PwmGeneration::SetAmpnom(Param::Get(Param::ampnom));
         break;
   #endif
      case Param::canspeed:
//         can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
         break;
      case Param::throtmax:
      case Param::throtmin:
      case Param::idcmin:
      case Param::idcmax:
      case Param::offthrotregen:
         //These are candidates to be frequently set by CAN, so we handle them separately
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         break;
      case Param::nodeid:
//         can->SetNodeId(Param::GetInt(Param::nodeid));
//         terminal->SetNodeId(Param::GetInt(Param::nodeid));
         Terminal::defaultTerminal->SetNodeId(Param::GetInt(Param::nodeid));
         break;
      default:
         PwmGeneration::SetCurrentLimitThreshold(Param::Get(Param::ocurlim));
         PwmGeneration::SetPolePairRatio(Param::GetInt(Param::polepairs) / Param::GetInt(Param::respolepairs));

         #if CONTROL == CTRL_FOC
         //PwmGeneration::SetControllerGains(Param::GetInt(Param::curkp), Param::GetInt(Param::curki));
         PwmGeneration::SetControllerGains(Param::GetInt(Param::iqkp), Param::GetInt(Param::idkp), Param::GetInt(Param::curki));
         Encoder::SwapSinCos((Param::GetInt(Param::pinswap) & SWAP_RESOLVER) > 0);
         FOC::SetMotorParameters(Param::GetFloat(Param::lqminusld) / 1000.0f, Param::GetFloat(Param::fluxlinkage) / 1000.0f);
         #endif // CONTROL

         Encoder::SetMode((enum Encoder::mode)Param::GetInt(Param::encmode));
         Encoder::SetImpulsesPerTurn(Param::GetInt(Param::numimp));
         Encoder::SetSinCosOffset(Param::GetInt(Param::sincosofs));

         Throttle::potmin[0] = Param::GetInt(Param::potmin);
         Throttle::potmax[0] = Param::GetInt(Param::potmax);
         Throttle::potmin[1] = Param::GetInt(Param::pot2min);
         Throttle::potmax[1] = Param::GetInt(Param::pot2max);
         Throttle::brknom = Param::GetFloat(Param::regentravel);
         Throttle::brknompedal = Param::GetFloat(Param::brakeregen);
         Throttle::regenRamp = Param::GetFloat(Param::regenramp);
         Throttle::brkmax = Param::GetFloat(Param::offthrotregen);
         Throttle::brkcruise = Param::GetFloat(Param::cruiseregen);
         Throttle::throtmax = Param::GetFloat(Param::throtmax);
         Throttle::throtmin = Param::GetFloat(Param::throtmin);
         Throttle::idleSpeed = Param::GetInt(Param::idlespeed);
         Throttle::holdkp = Param::GetFloat(Param::holdkp);
         Throttle::speedkp = Param::GetFloat(Param::speedkp);
         Throttle::speedflt = Param::GetInt(Param::speedflt);
         Throttle::idleThrotLim = Param::GetFloat(Param::idlethrotlim);
         Throttle::bmslimlow = Param::GetInt(Param::bmslimlow);
         Throttle::bmslimhigh = Param::GetInt(Param::bmslimhigh);
         Throttle::udcmin = Param::GetFloat(Param::udcmin) * 0.99; //Leave some room for the notification light
         Throttle::udcmax = Param::GetFloat(Param::udcmax) * 1.01;
         Throttle::idcmin = Param::GetFloat(Param::idcmin);
         Throttle::idcmax = Param::GetFloat(Param::idcmax);
         Throttle::idckp = Param::GetFloat(Param::idckp);
         Throttle::fmax = Param::GetFloat(Param::fmax);

         if (hwRev != HW_BLUEPILL)
         {
            if (Param::GetInt(Param::pwmfunc) == PWM_FUNC_SPEEDFRQ)
               gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_PUSHPULL, GPIO9);
            else
               gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO9);
         }
         break;
   }
}
