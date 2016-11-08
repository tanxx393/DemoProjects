/********************************************************************************** 
* $HeadURL: http://usapgeded000077:8080/svn/Emerson/EDP/Active/HP/HP%20QUAD%202400/branches/evans_working/source/app_fancontrol.c $                 // The full URL of this file in the repository
*
**********************************************************************************
* File:        app_fancontrol.c
*
* $Revision: 1708 $                // Revision of last known commit
* $Date: 2012-06-01 11:41:36 -0500 (Fri, 01 Jun 2012) $                    // Date of last known commit
* $Author: EvanLucore@EMRSN $                  // Author who made the last known commit
**********************************************************************************
* Description:
*
* <insert description of the software module/file>
*
**********************************************************************************
*                        Emerson Network Power
*                        Embedded Power
*                        Street Address
*                        City, State or Code, Zipcode
*                        Country
*
*          [ ] Open Source   [ ] Contains Open Source Material  [ ] Proprietary 
*
*  EMERSON EMBEDDED POWER VIENNA CONFIDENTIAL AND PROPRIETARY
*
*  This material is proprietary of EMERSON NETWORK POWER – EMBEDDED POWER 
*  and may only be viewed under the terms of a signed non-disclosure agreement 
*  or a  source code license agreement! Copying, compilation, modification,
*  distribution or any other use whatsoever of this material is strictly prohibited 
*  without written permission of EMERSON NETWORK POWER – EMBEDDED POWER.
*  The information in this document is subject to change without notice and should 
*  not be construed as a commitment by EMERSON NETWORK POWER – EMBEDDED POWER.
*  Neither EMERSON NETWORK POWER – EMBEDDED POWER nor the authors assume any 
*  responsibility for the use or reliability of this document or the described 
*  software.
*
*  Copyright (c) 2011, EMERSON NETWORK POWER – EMBEDDED POWER
*  All rights reserved.
*********************************************************************************/


/****************************************************************************/
/***** INCLUDES *************************************************************/
/****************************************************************************/
#include "app_parameters.h"
#include "drv_init_hw.h"		       // module header file
#include "p33FJ64GS606.h"
#include "app_fancontrol.h"
#include "app_monitor.h"
#include "mid_psos.h"
#include "app_i2c.h"
#include "drv_io_config.h"
#include "app_prim_comm.h"
#include "app_device_state_machine.h"

/****************************************************************************/
/* *** IMPORTS **************************************************************/
/****************************************************************************/
extern uint16  u16FanAlertCounter;
extern T_timer timerFanFault;

/****************************************************************************/
/* *** LOCAL DEFINES ********************************************************/
/****************************************************************************/
#define POC_FRONT_END_PROTECTION 1
#define LIMIT_TO_NFS  1         // set LIMIT_TO_NFS to 2 for expiriment, else 1.
FAN_CONTROL	fcFan1;

#define FANCONTROL_TEMP_LIMIT 100 //degC
#define FANCONTROL_TEMP_HIGH  90 //degC was 80
//#define FANCONTROL_TEMP_HIGH  105 //degC to allow PTAVS 100% load limits
#define FANCONTROL_TEMP_LOW   70 //degC
//#define FANCONTROL_TEMP_LOW   60 //degC
#define FANCONTROL_TEMP_FAN_OFF 35 //degC
#define FANCONTROL_TEMP_HYSTERESIS 2    //DEGc

#if POC_FRONT_END_PROTECTION
#define K16_FANCONTROL_PROTECTION_VIN   150
#define K16_FANCONTROL_PROTECTION_IOUT  58.0
#endif

#define K16_FANCONTROL_MINIMUM_PERCENT      (uint16)(FANCONTROL_MIN_DUTY * 100)
#define K16_FANCONTROL_50_PERCENT           (uint16)(FANCONTROL_MAX_DUTY * 50)

uint16 u16FanOverride;
uint16 u16FanOverdrive;

uint16 u16LoadSetPoint;

#define K16_FAN_LOAD_AVERAGING 1

#if K16_FAN_LOAD_AVERAGING != 0
//uint32 u32LoadSetPointAccumulator;
#endif

uint16 u16OperatingNFSavg;
uint32 u32OperatingNFSaccm;

/****************************************************************************/
/* *** PUBLIC FUNCTIONS *****************************************************/
/****************************************************************************/

bool fancontrol_SystemOverride ( void );


// ------- NFS Calculations....
#if 1 //K16_BUILD_FOR_PROJECT == 2400
#define K16_NFS_100_PERCENT 28600 //25500   // 100% NFS definition (RPM)
#endif
   //  xx% NFS as RPM definitions
#define K16_NFS_80_PERCENT_RPM  (uint16)(K16_NFS_100_PERCENT * 0.8)
#define K16_NFS_60_PERCENT_RPM  (uint16)(K16_NFS_100_PERCENT * 0.6)
#define K16_NFS_50_PERCENT_RPM  (uint16)(K16_NFS_100_PERCENT * 0.5)
#define K16_NFS_20_PERCENT      (uint16)(K16_NFS_100_PERCENT * 0.2)
#define K16_NFS_21_PERCENT      (uint16)(K16_NFS_100_PERCENT * 0.22)   // ~20% NFS supporting Bias OC/OT (RPM)
#define K16_NFS_STBY_OC_RPM     (uint16)(K16_NFS_100_PERCENT * 0.28)   // ~28% NFS supporting Bias OC
#define K16_NFS_17_PERCENT_RPM  (uint16)(K16_NFS_100_PERCENT * 0.17)   // a minimum operating RPM



// -- "Real" values (ie, floating point...)
#define KD_NFS_m    ( (K16_NFS_100_PERCENT - K16_NFS_20_PERCENT)/(100.0-20.0))
#define KD_NFS_b    ( K16_NFS_100_PERCENT-(KD_NFS_m * 100.0))

// -- Equation to convert any "NFS duty cycle %" to real RPM...
#define K16_NFS_to_RPM(x) ((uint16) ( KD_NFS_m * x + KD_NFS_b))

// -- Usable values (ie, not floating...)
#define K16_NFS_m   ((uint32) (KD_NFS_m * 64))
#define K16_NFS_b   ((int16)  (KD_NFS_b))
#define KD_NFS_bFix    (int16) 0       // fine tuning of NFS_b

/*****************************************************************************
 * fancontrol_NFS_to_RPM(uint16 u16duty)
 *
 *  This routine converts a 0-100% value to what the resulting RPM should be.
 *
 *****************************************************************************/
uint16 fancontrol_NFS_to_RPM(uint16 u16duty)
{
    int16 i16Val = (int16)(((MACRO_MPY_U16(K16_NFS_m, u16duty) >> 6) + K16_NFS_b)-KD_NFS_bFix);
    if (i16Val < K16_MIN_FAN_RPM) i16Val = K16_MIN_FAN_RPM;
    if (i16Val > K16_NFS_100_PERCENT) i16Val = K16_NFS_100_PERCENT;
    return (uint16) (i16Val);
}

#define KD_20_PERCENT_LOAD  (MAX_ISHARE_I * .20)



#if LIMIT_TO_NFS == 1
/*****************************************************************************
 * fancontrol_LimitToNFSCurve(uint16 u16RPM)
 *
 *  This routine keeps requested RPM value under the NFS% curve based on load.
 *  Receives a hotspot thermal u16RPM
 *  //Returns u16RPM if in high ambient override and limits to 100%
 *  Returns 100% fan speed if no NFS limit exists
 *  Else if over MAX_ISHARE_I110 Retruns 100% fan speed
 *  Else a NFS Limited working RPM value based on load range
 *  
 *****************************************************************************/

uint16 fancontrol_LimitToNFSCurve( uint16 u16RPM )
{

    uint16 u16OperatingRPM, u16NFS, u16Load, u16Ambient;

    u16Ambient   = GET_PSInletTemperature();
    u16NFS = 0;

    if (MACRO_MAIN_IS_DISABLED())
    {
        // Main is off, return thermal RPM
//        u32OperatingRPMaccm = 0;
//        u16OperatingRPMavg = 0;
        return u16RPM;
    }

#if K16_FAN_LOAD_AVERAGING != 0
    // long average output current
    u16Load = GET_PSAvgADCOutputCurrent();
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug7 = u16Load;
#endif
#else
    u16Load = tADC_IOUT_AMP.u16Avg;
#endif

    if (TRUE == monitor_GetAcRangeLLStatus())
    {
        // low line scale the load current 2X
        u16Load = (u16Load<<1);
    }

    // Apply a large average on the IOUT to avoid responding to transients... was >> 2
#if K16_FAN_LOAD_AVERAGING != 0
//    u32LoadSetPointAccumulator -= (fcFan1.u16IoutAccum >> 2);
//    u32LoadSetPointAccumulator + u16Load;
//    u16Load = u32LoadSetPointAccumulator >> 2;
#else
    fcFan1.u16IoutAccum -= (fcFan1.u16IoutAccum >> 1);
    fcFan1.u16IoutAccum += u16Load;
    u16Load = fcFan1.u16IoutAccum >> 1;
#endif

    // Check for "high ambient" recovery...
    if (fcFan1.tFlags.bBits.bHighAmbient
         && (u16Ambient < 52))
    {
        fcFan1.tFlags.bBits.bHighAmbient = FALSE; // Recovered...
    }
    else if (fcFan1.tFlags.bBits.bHighAmbient)
    {
        // fcFan1.u16NFSLimitRPM was set in load curves
        return u16RPM;  // Don't limit NFS while flagged for "high ambient"
    }

    // ---- NFS Explanation:
    // 1.  Checks load curve (<=20%, <=50%, <=80%, <=100%(2kW)
    // 2.  Determines point on curve with ambient reading
    // 3.  Verifies current RPM is not above this value

    // Check for 20% load curve...
    if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .26))
    {
        u16NFS = 15;    // Assume 20% first...

        // Starts to bend at 49degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        if (u16Ambient > 67)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_60_PERCENT_RPM;
            return u16RPM;
        }
#if 1 //(K16_BUILD_FOR_PROJECT == 2400)
        else if (u16Ambient > 62)
        {
            // Beyond PTAVS curves, agressive ramp
            u16NFS = 40;
            u16NFS += (u16Ambient - 62) * 6;
        }
        else if (u16Ambient > 56)
        {
            // Curve is approximated by 2.4% per degC above 55
            u16NFS = 28;
            u16NFS += (u16Ambient - 56) << 1;
        }
#endif
        else if (u16Ambient > 46)
        {
            // Curve is approximated by 1% per degC above 49
            u16NFS = 19;
            u16NFS += (u16Ambient - 46);
       }
       else
       {
            if (1 == fcFan1.tFlags.bBits.bStbyOCWtherm)
            {
                // Push minimum fan here to support bias module cooling needs
                u16RPM = K16_NFS_STBY_OC_RPM;
                u16NFS = 29;
            }
       }
    }
    // Check for 50% load curve...
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .54))
    {
        // Starts to bend at 35degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        //if (eDebugPinState == FIRMWAREflagF) MACRO_ASSERT_FIRMWARE_FLAG();

        if (u16Ambient > 64)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return K16_NFS_100_PERCENT;
        }
#if 1 //(K16_BUILD_FOR_PROJECT == 2400)
        // High line - normal minimum NFS
        else if (u16Ambient > 56)
        {
            // Curve is approximated by 6% NFS per degC above 50
            u16NFS = 62;
            u16NFS += ((u16Ambient - 56) * 6);

        }
        else if (u16Ambient > 50)
        {
            // Curve is approximated by 2.4% NFS per degC above 50
            u16NFS = 47;
            u16NFS += ((u16Ambient - 50) * 2);

        }
        else if (u16Ambient > 39)
        {
            // Curve is approximated by 1% per degC above 38
            u16NFS = 35;
            u16NFS += (u16Ambient - 39);
        }
        else if (u16Ambient > 29)
        {
            //if (eDebugPinState == FIRMWAREflagF) MACRO_ASSERT_FIRMWARE_FLAG();
            u16NFS = 28;
                // 30%<load<56%  ~25 to 40 degC ~.5% per degC
                u16NFS += ((u16Ambient - 29)>>1);
        }
        else
        {
            // below 25 degC
            u16NFS = 25;
        }

    }
#else
        else if (u16Ambient > 38)
        {
            // Curve is approximated by 1% per degC above 38
            u16NFS = 29;
            u16NFS += (u16Ambient - 38);
        }
        else if (u16Ambient > 27)
        {
            //if (eDebugPinState == FIRMWAREflagF) MACRO_ASSERT_FIRMWARE_FLAG();
            u16NFS = 22;
                // 30%<load<56%  ~25 to 40 degC ~.5% per degC
                u16NFS += ((u16Ambient - 26)>>1);
        }
        else
        {
            // below 25 degC
            u16NFS = 21;
        }
    }
#endif

    // Check for 80% load curve...
    // - Linear approximated with a "corner" at 42degC
    // - Care was taken to ensure this curve is less than the spec curve
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .85))
    {
        #if K16_BUILD_FOR_PROJECT == 495

            u16NFS = 43;

            if (u16Ambient < 27)
            {
                u16NFS = 43;
            }
            else if (u16Ambient <= 42)
            {
                u16NFS += (u16Ambient - 25); // Slope is approx. 1% per degC
            }
            else if (u16Ambient <= 60)
            {
                u16NFS = ((u16Ambient - 41) << 1) + 50;
            }
            else
            {
                fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
                fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
                return u16RPM;  // No NFS curve specified above 55degC...
            }

#else



//            if (eDebugPinState == FIRMWAREflagF) MACRO_DEASSERT_FIRMWARE_FLAG();
            //u16NFS = 32;
            if (u16Ambient > 55)
            {
                fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
                fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
                return K16_NFS_100_PERCENT;    // No NFS curve specified above 55degC...
            }
            else if (u16Ambient > 52)
            {
                u16NFS = 95;
                // slope approx 6%/degC 50-55degC
                u16NFS += ((u16Ambient - 52) *6);
            }
            else if (u16Ambient > 41)
            {
                u16NFS = 72;
                // slope approx 2%/degC 35-45degC
                u16NFS += (u16Ambient - 41)<<1;
            }
            else if (u16Ambient > 27)
            {
                // slope approx 1%/degC 25-35degC
                u16NFS = 58;
                u16NFS += (u16Ambient - 27);
            }
            else
            {
                // ~25 degC
                u16NFS = 57;
            }
#endif

    }
#if 1 //(K16_BUILD_FOR_PROJECT == 2400)
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I105))
    {
//#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
//        if (eDebugPinState == FIRMWAREflagF) MACRO_DEASSERT_FIRMWARE_FLAG();
//#endif
        if (u16Ambient > 48)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows over 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return K16_NFS_100_PERCENT;  // No NFS curve specified above 55degC...
        }

        if (u16Ambient > 26)
        {
            // 27<TAmb<45, 15%nfs/15 degreeC = 1.0
            u16NFS = 79;
            u16NFS += (u16Ambient - 26); // Slope is approx. 1% per degC
        }
        else
        {
            // min NFS at >90% load
            u16NFS = 78;
        }
        if (u16Load < OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I95))
        {
            u16NFS -=5;
        }
    }
    else
    {
        // Over 110% load, no NFS curve specified...
        fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
        return K16_NFS_100_PERCENT;
    }   // end if/else uload

#endif

    u32OperatingNFSaccm -= u16OperatingNFSavg;
    u32OperatingNFSaccm +=u16NFS;
    u16OperatingNFSavg = u32OperatingNFSaccm >> 2;

    // -- Now, convert the desired NFS to RPM
    u16NFS = fancontrol_NFS_to_RPM(u16OperatingNFSavg);

#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug0 = u16OperatingNFSavg;
#endif

    // - Ramp to our NFS value...
    if (u16NFS > 0)
    {
        if (fcFan1.u16NFSLimitRPM <= u16NFS)
        {
            // Allow to "step up" fast...
            fcFan1.u16NFSLimitRPM = u16NFS;
        }
        else
        {
            // Must "step down" slowly...
            if ((fcFan1.u16NFSLimitRPM - u16NFS) < 500)
            {
                fcFan1.u16NFSLimitRPM = u16NFS;
            }
            else
            {
                fcFan1.u16NFSLimitRPM -= 500;
            }
        }
    }
    else
    {
         fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
         return u16RPM;
    }

    // Cap thermal RPM to NFS limit ...
    if (u16RPM >= fcFan1.u16NFSLimitRPM)
    {
        return (fcFan1.u16NFSLimitRPM - K8_RPM_CHANGE_PER_LOAD);
    }
    else
    {
        // set operating at ~94% of NFS limit (60/64) to reduce thermal overshoot.
        u16OperatingRPM = (uint16)((MACRO_MPY_U16(fcFan1.u16NFSLimitRPM, (uint16)60) >> 6));

        if (K16_NFS_17_PERCENT_RPM > u16OperatingRPM)
        {
            u16OperatingRPM = K16_NFS_17_PERCENT_RPM;
        }
            
        return u16OperatingRPM;
    }
}

#elif LIMIT_TO_NFS == 99  // previous code for reference.

uint16 fancontrol_LimitToNFSCurve( uint16 u16RPM )
{


    uint16 u16Ambient   = GET_PSInletTemperature();
    uint16 u16Load      = tADC_12V_IOUT.u16Avg;
    uint16 u16NFS = 0;

    // Apply a large average on the IOUT to avoid responding to transients... was >> 2
    fcFan1.u16IoutAccum -= (fcFan1.u16IoutAccum >> 1);
    fcFan1.u16IoutAccum += u16Load;
    u16Load = fcFan1.u16IoutAccum >> 1;
    
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
//    u16i2cDebug0 = u16Load;
//    u16i2cDebug1 = 1;
//    u16i2cDebug2 = 2;
//    u16i2cDebug3 = 3;
    
#endif

    // Check for "high ambient" recovery...
    if (fcFan1.tFlags.bBits.bHighAmbient
         && (u16Ambient < 56))
    {
        fcFan1.tFlags.bBits.bHighAmbient = FALSE; // Recovered...
    }
    else if (fcFan1.tFlags.bBits.bHighAmbient)
    {
        fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
        return u16RPM;  // Don't limit NFS while flagged for "high ambient"
    }

    // ---- NFS Explanation:
    // 1.  Checks load curve (<=20%, <=50%, <=80%, <=100%(2kW)
    // 2.  Determines point on curve with ambient reading
    // 3.  Verifies current RPM is not above this value

    // Check for 20% load curve...
    if (u16Load <= OUTPUT_I_SCALE(MAX_ISHARE_I * .26))
    {
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
        u16i2cDebug1 = 1;
#endif
        u16NFS = 15;    // Assume 20% first...

        // Starts to bend at 49degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        if (u16Ambient > 66)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return u16RPM;
        }
#if K16_BUILD_FOR_PROJECT == 2000
        else if (u16Ambient > 62)
        {
            // Beyond PTAVS curves, agressive ramp
            u16NFS = 42;
            u16NFS += (u16Ambient - 62) * 6;
        }
        else if (u16Ambient > 56)
        {
            // Curve is approximated by 2.4% per degC above 55
            u16NFS += (u16Ambient - 42) * 2;
        }
#endif
        else if (u16Ambient > 49)
        {
            // Curve is approximated by 1% per degC above 49
            u16NFS += (u16Ambient - 49);
        }
    }
    // Check for 50% load curve...
    else if (u16Load <= OUTPUT_I_SCALE(MAX_ISHARE_I * .54))
    {
        // Starts to bend at 35degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        if (u16Ambient > 61)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return u16RPM;
        }
#if K16_BUILD_FOR_PROJECT == 2000
        else if (u16Ambient > 56)
        {
            // Curve is approximated by 6% NFS per degC above 50
            u16NFS = 55;
            u16NFS += ((u16Ambient - 56) * 6);
            
        }
        else if (u16Ambient > 50)
        {
            // Curve is approximated by 2.4% NFS per degC above 50
            u16NFS = 42;
            u16NFS += ((u16Ambient - 50) * 2);
            
        }
#endif
        else if (u16Ambient > 38)
        {
            // Curve is approximated by 1% per degC above 38
            u16NFS = 26;
            u16NFS += (u16Ambient - 38);
        }
        else
        {
            // below 38
            u16NFS = 22;
            if (u16Ambient > 34) u16NFS += (u16Ambient - 33);
        }
    }

    // Check for 80% load curve...
    // - Linear approximated with a "corner" at 42degC
    // - Care was taken to ensure this curve is less than the spec curve
    else if (u16Load <= OUTPUT_I_SCALE(MAX_ISHARE_I * .85))
    {
        #if K16_BUILD_FOR_PROJECT == 495

            u16NFS = 43;

            if (u16Ambient < 27)
            {
                u16NFS = 43;
            }
            else if (u16Ambient <= 42)
            {
                u16NFS += (u16Ambient - 25); // Slope is approx. 1% per degC
            }
            else if (u16Ambient <= 60)
            {
                u16NFS = ((u16Ambient - 41) << 1) + 50;
            }
            else
            {
                fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
                fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
                return u16RPM;  // No NFS curve specified above 55degC...
            }

        #else

            if (u16Ambient > 57)
            {
                fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
                return u16RPM;  // No NFS curve specified above 55degC...
            }
            else if (u16Ambient > 52)
            {
                u16NFS = 81;
                // slope approx 6%/degC 50-55degC
                u16NFS = ((u16Ambient - 52) *6);
            }
            else if (u16Ambient > 47)
            {
                u16NFS = 65;
                // slope approx 3%/degC 45-50degC
                u16NFS = ((u16Ambient - 47) *3);
            }
            else if (u16Ambient > 37)
            {
                u16NFS = 45;
                // slope approx 2%/degC 35-45degC
                u16NFS = ((u16Ambient - 37) << 1);
            }
            else if (u16Ambient >= 27)
            {
                // slope approx 1%/degC 25-35degC
                u16NFS = 36;
                u16NFS += (u16Ambient - 27);
            }
            else
            {
                // ~25 degC
                u16NFS = 36;
            }

        #endif
    }
    #if K16_BUILD_FOR_PROJECT == 2000
    else if (u16Load <= OUTPUT_I_SCALE(MAX_ISHARE_I * 1.05))
    {
#warning !!!!!!!!!!!!!!!!!!using elevated NFS for 100% load !!!!!!!!!!!!!!!!
        if (u16Ambient < 26)
        {
//            u16NFS = 45;
            u16NFS = 70;

        }
        else if (u16Ambient <= 42)
        {
            // 26<TAmb<42, 20%nfs/15 degreeC = 1.33
//            u16NFS = 48;
            u16NFS = 70;

            u16NFS += (u16Ambient - 26); // Slope is approx. 1% per degC
        }
        else if (u16Ambient <= 52)
        {
            // 42<TAmb<52, 30%nfs/10 degreeC = 3.0
//            u16NFS = 65;
            u16NFS = 85;
            u16NFS = ((u16Ambient - 42) * 3); // Slope is approx. 3% per degC
        }
        else
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            return u16RPM;  // No NFS curve specified above 55degC...
        }
    }
#endif
    if (0 == u16NFS)
    {
        fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
        return u16RPM; //No NFS curve specified...
    }
   
    // 495W NFS is approximately cut in half from 750W (with 20% minimum)
    #if K16_BUILD_FOR_PROJECT == 495

    u16NFS = u16NFS >> 1;
    if (u16NFS < 19) u16NFS = 19;

    #endif



    // -- Now, convert the desired NFS to RPM
    u16NFS = fancontrol_NFS_to_RPM(u16NFS);


    // - Ramp to our NFS value...
    if (u16NFS > 0)
    {
        if (fcFan1.u16NFSLimitRPM < u16NFS)
        {
            // Allow to "step up" fast...
            fcFan1.u16NFSLimitRPM = u16NFS;
        }
        else
        {
            // Must "step down" slowly...
            if ((fcFan1.u16NFSLimitRPM - u16NFS) < 500)
            {
                fcFan1.u16NFSLimitRPM = u16NFS;
            }
            else
            {
                fcFan1.u16NFSLimitRPM -= 500;
            }
        }
    }
    else
    {
         fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
         return u16RPM;
    }


    // Apply NFS limit if necessary...
    if (u16RPM > fcFan1.u16NFSLimitRPM)
    {
        return fcFan1.u16NFSLimitRPM;
    }
    else
    {
        return u16RPM;
    }

}
#endif


// RPM-based fan control...
/*  CONCEPT:
 *
 * This algorithm calculates what it wants to run at based on temperature and
 * load, ensuring the result is under the NFS curve (unless we exceed 80% load
 * or are near OTW conditions).  The resulting value is an RPM value we want
 * to "regulate" our fan speed around.  Finally, we handle the loop that
 * actually does the fan speed regulation.
 *
 */

uint16 u16LowLoadEffCounter;

void fancontrol_DetermineSetpoint ( void )
{

    // === [ Update current manufacturer override setting...
    fcFan1.tFlags.bBits.bManufacturerOverride = app_ManufacturingFanOverrideMet();

    // === [ Get contributing factors ready...
    uint16 u16Temperature = GET_PSInternalTemperature();

//#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
//if (eDebugPinState == FIRMWAREflagE) MACRO_TOGGLE_FIRMWARE_FLAG();
//#endif


    // === [ Thermal Factors...
   
    // - Check for 100% override due to OTW if MAIN is enabed
    if (u16Temperature > MAIN_OTW_FAN_OR_THRESHOLD  && MACRO_MAIN_IS_ENABLED())
    {
        if ((u16Temperature >= MAIN_OTW_REC_THRESHOLD)
            && (tADC_12V_IOUT.u16Avg > OUTPUT_I_SCALE(MAX_ISHARE_I * .85))
            && (GET_PSInletTemperature() > 52))
        {
            // allow max fan in extreme case
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows over 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_MAX_FAN_RPM; 
        }
        // Inhibits LimitToNFScurves
        fcFan1.tFlags.bBits.bThermalOverride = TRUE;
    }
    else if (u16Temperature <= MAIN_OTW_FAN_OR_THRESHOLD_RELEASE)
    {
        fcFan1.tFlags.bBits.bThermalOverride = FALSE;
    }

    // - Now, adjust setpoint based on current temperature...
    if (u16Temperature > FANCONTROL_TEMP_HIGH)
    {
        // Increase our setpoint based on the distance above our threshold...
        fcFan1.u16TemperatureRPM += \
                MACRO_MPY_U16(K16_RPM_CHANGE_PER_DEGC, u16Temperature - FANCONTROL_TEMP_HIGH);

        // Don't let it run up to infinite...keep it reasonable ranged...
        if (fcFan1.u16TemperatureRPM > K16_MAX_FAN_RPM)
        {
            fcFan1.u16TemperatureRPM = K16_MAX_FAN_RPM;
        }

    }
    else if (u16Temperature < FANCONTROL_TEMP_LOW)
    {
        uint16 u16Adjust = \
            MACRO_MPY_U16(FANCONTROL_TEMP_LOW - u16Temperature, K16_RPM_CHANGE_PER_DEGC);

        if (fcFan1.u16TemperatureRPM > u16Adjust)
        {
            fcFan1.u16TemperatureRPM -= u16Adjust;
        }
        else
        {
            fcFan1.u16TemperatureRPM = 0;
        }
    }














    // === [ Check for abnormal conditions...
#if 0
    if ( app_GetPrimaryVoltage(READ_CORRECTED_VOLTS) <= K16_FANCONTROL_PROTECTION_VIN
        && tADC_12V_IOUT.u16Avg > OUTPUT_I_SCALE(K16_FANCONTROL_PROTECTION_IOUT)
        && fcFan1.u16TemperatureRPM < K16_FULL_LOAD_RPM)
    {
        if (++u16LowLoadEffCounter >  10)
        {
            fcFan1.u16TemperatureRPM = K16_FULL_LOAD_RPM;
            u16LowLoadEffCounter = 10;
        }
    }
    else
    {
        u16LowLoadEffCounter = 0;
    }
#endif


    // === [ Determine if we should use system override or internal RPM...
    
    // Apply NFS-limiting curve as necessary...
    if (!fcFan1.tFlags.bBits.bThermalOverride
       && !fcFan1.tFlags.bBits.bManufacturerOverride)
    {
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
//    u16i2cDebug0 = fcFan1.u16NFSLimitRPM;
    u16i2cDebug1 = fcFan1.u16TemperatureRPM;
#endif

        fcFan1.u16TemperatureRPM = fancontrol_LimitToNFSCurve(fcFan1.u16TemperatureRPM);
    }
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug2 = fcFan1.u16TemperatureRPM;
#endif

    if (fancontrol_SystemOverride())
    {
        fcFan1.u16SetPointRPM = fcFan1.u16SystemRPM;
        fcFan1.tFlags.bBits.bSystemOverride = TRUE;
    }
    else
    {
        fcFan1.u16SetPointRPM = fcFan1.u16TemperatureRPM;

        // If no override, ensure minimum of 50% NFS if PSU_MANU bit 0 set...
        if (fcFan1.tFlags.bBits.bManufacturerOverride
            && fcFan1.u16SetPointRPM < K16_NFS_to_RPM(50.0))
        {
            fcFan1.u16SetPointRPM = K16_NFS_to_RPM(50.0);
        }

        fcFan1.tFlags.bBits.bSystemOverride = FALSE;
    }

    #if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug3 = fcFan1.u16SetPointRPM;
#endif


    // === [ Check if fan should be off...
    // === Bias Module OC enable of fan

    // - Check if standby and no system override present...
    // ALSO prevent fan enable if PSU is in start-up
    if (!fcFan1.tFlags.bBits.bManufacturerOverride
        && ((!MACRO_MAIN_IS_ENABLED()
                && (FALSE == fcFan1.tFlags.bBits.bSystemOverride))
            || MACRO_SBY_IS_DISABLED()
            || I2C.IPMMdata.rapidon_cntrl.b_wake_status == 0b1)
       )
    {
        uint16 u16Ambient = GET_PSInletTemperature();

        if ((1 == fcFan1.tFlags.bBits.bStbyOCWtherm)
                && MACRO_SBY_IS_ENABLED())
			//            || (1 == fault_FaultFlags.OTP_FLAG))
        {
            fcFan1.tFlags.bBits.bFanOff = FALSE;
        }
        else if (
                (u16Ambient < (FANCONTROL_TEMP_FAN_OFF - FANCONTROL_TEMP_HYSTERESIS)
                 && 0 != PDC5)
                || ((u16Ambient <= FANCONTROL_TEMP_FAN_OFF)
                    && 0 == PDC5)
                || MACRO_SBY_IS_DISABLED()
                || I2C.IPMMdata.rapidon_cntrl.b_wake_status == 0b1)
        {
            fcFan1.tFlags.bBits.bFanOff = TRUE;
        }
        else
        {
            fcFan1.tFlags.bBits.bFanOff = FALSE;
        }

    }
    else
    {
        fcFan1.tFlags.bBits.bFanOff = FALSE;
    }

#if 1 // K16_BUILD_FOR_PROJECT == 2400
    fcFan1.u16LoadSetPointRPM = 0;
#else
    uint16 u16LoadAvgADC;
    if (TRUE == monitor_GetAcRangeLLStatus())
    {
        u16LoadAvgADC = fcFan1.u16IoutAccum >> 2;
        // -Low line load "feed forward" RPM adder...
        if (u16LoadAvgADC >= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_LL_I90))
        {
            //EMI filters need full fan speed at > 90% load
//            if (u16LoadAvgADC > OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_LL_I105))
//            {
            fcFan1.u16LoadSetPointRPM += K8_RPM_CHANGE_PER_LOAD;
//            }
            if (app_GetPrimaryVoltage(READ_CORRECTED_VOLTS) < 100)
            {
               fcFan1.u16LoadSetPointRPM += K8_RPM_CHANGE_PER_LOAD;
            }
        }
        else if (u16LoadAvgADC < OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_LL_I90-4.0))
        {
            if (fcFan1.u16LoadSetPointRPM > K8_RPM_CHANGE_PER_LOAD)
            {
                fcFan1.u16LoadSetPointRPM -= K8_RPM_CHANGE_PER_LOAD;
            }
            else
            {
                fcFan1.u16LoadSetPointRPM = 0;
            }
        }
    }
    else
    {
        // high line load "feed forward" RPM adder...
        u16LoadAvgADC = fcFan1.u16IoutAccum >> 1;
        if (u16LoadAvgADC >= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I90))
        {
            if (u16LoadAvgADC > OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I105))
            {
                fcFan1.u16LoadSetPointRPM += K8_RPM_CHANGE_PER_LOAD;
            }
            if (app_GetPrimaryVoltage(READ_CORRECTED_VOLTS) < 180)
            {
               fcFan1.u16LoadSetPointRPM += K8_RPM_CHANGE_PER_LOAD;
            }
        }
        else if (u16LoadAvgADC < OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I90-4.0))
        {
            if (fcFan1.u16LoadSetPointRPM > K8_RPM_CHANGE_PER_LOAD)
            {
                fcFan1.u16LoadSetPointRPM -= K8_RPM_CHANGE_PER_LOAD;
            }
            else
            {
                fcFan1.u16LoadSetPointRPM = 0;
            }
        }
    }
#endif
    if (fcFan1.u16LoadSetPointRPM > K16_MAX_LOAD_RPM) fcFan1.u16LoadSetPointRPM = K16_MAX_LOAD_RPM;

    fcFan1.u16SetPointRPM += fcFan1.u16LoadSetPointRPM;

#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug4 = fcFan1.u16SetPointRPM;

#endif

}





/****************************************************************************
 *  fancontrol_PerformFanLoop
 *  DESCRIPTION:
 *      Manages the RPM regulation of the fan speed.  It utilizes a "poor mans"
 *      PID loop.  This was done due to the timeframe to develop--it should be
 *      replaced with a true loop at a later time...
 * 
 ****************************************************************************/

//#define K8_RPM_WINDOW   100 // Must be +/- this value to stop adjusting...
#define K8_RPM_WINDOW   200 // Must be +/- this value to stop adjusting...
#define K8_RPM_WINDOW_HIGH  450 // at RPM > K8_RPM_HIGH_WINDOW, Must be +/- this value to stop adjusting

#define K8_RPM_HIGH_WINDOW  24000   // ~95% NFS
#define K8_STEP_MAX_RANGE   7000
#define K8_STEP_MID_RANGE   3000
#define K8_STEP_LOW_RANGE   500
#define K8_STEP_MIN_RANGE   100

#define K8_STEP_MAX     ((uint16)(FAN_PWM_MAX_PDC*.07))
#define K8_STEP_MID     ((uint16)(FAN_PWM_MAX_PDC*.03))
#define K8_STEP_LOW     ((uint16)(FAN_PWM_MAX_PDC*.005))
#define K8_STEP_MIN     10

void fancontrol_PerformFanLoop ( void )
{
    uint16 u16CurrentSpeed = tFanSpeed.u16Avg;
    uint16 u16DesiredSpeed = fcFan1.u16SetPointRPM;
    int16  i16Duty = (int16) fcFan1.u16CurrentDuty;
    uint16 u16Difference;
    uint16 u16RPMrange;
    
    // Check if fan is enabled...
    if (FALSE == fcFan1.tFlags.bBits.bFanOff
        && FALSE == I2C.IPMMdata.psu_manufacturing.bBits.bFanDisable)
    {
        // Enforce 20% if in standby...(fan is not off if gotten here)
        if (!MACRO_MAIN_IS_ENABLED()
             && (!fcFan1.tFlags.bBits.bManufacturerOverride
                || !fault_IsVSENSEvalidAC_Off())  // Added this to prevent fan from collapsing bias when AC removed during PSU_MANUFACTURE = 1 and override = 100%
            )
        {
            if (fcFan1.tFlags.bBits.bStbyOCWtherm)
            {
                // cap STBY_OC_RPM
                u16DesiredSpeed = K16_NFS_STBY_OC_RPM;
            }
            else
            {
                u16DesiredSpeed = K16_NFS_20_PERCENT;
            }
        }

        // Ensure limits...
        else
        {
            if ((K16_MAX_FAN_RPM <= fcFan1.u16NFSLimitRPM)
                    && (TRUE == fcFan1.tFlags.bBits.bHighAmbient))
            {
                // extreme condition override
                u16DesiredSpeed = K16_MAX_FAN_RPM;
            }
            else if (fcFan1.tFlags.bBits.bThermalOverride)
            {
                u16DesiredSpeed = K16_NFS_100_PERCENT;
            }
            else if (u16DesiredSpeed < K16_MIN_FAN_RPM)
            {
                u16DesiredSpeed = K16_MIN_FAN_RPM;
            }
            else if (u16DesiredSpeed > K16_NFS_100_PERCENT)
            {
                u16DesiredSpeed = K16_NFS_100_PERCENT;
            }
        }
        // Check if we're too fast or too slow...
            // Check if still greater than our window...
#if 0
         u16RPMrange = K8_RPM_WINDOW;
#else
        u16RPMrange = u16DesiredSpeed >
                K8_RPM_HIGH_WINDOW ? K8_RPM_WINDOW_HIGH : K8_RPM_WINDOW;
#endif
#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
    u16i2cDebug5 = u16DesiredSpeed;
    u16i2cDebug6 = u16CurrentSpeed;
//    u16i2cDebug7 = u16RPMrange;
if (eDebugPinState == FIRMWAREflagF) MACRO_TOGGLE_FIRMWARE_FLAG();
#endif

        // === [ Too Fast...
        if (u16CurrentSpeed > u16DesiredSpeed)
        {

            if (u16CurrentSpeed > (u16DesiredSpeed + u16RPMrange))
            {
                u16Difference = u16CurrentSpeed - u16DesiredSpeed;

                if (u16Difference > K8_STEP_MAX_RANGE)
                {
                    i16Duty -= K8_STEP_MAX;
                }
                else if (u16Difference > K8_STEP_MID_RANGE)
                {
                    i16Duty -= K8_STEP_MID;
                }
                else if (u16Difference > K8_STEP_LOW_RANGE)
                {
                    i16Duty -= K8_STEP_LOW;
                }
                else
                {
                    i16Duty -= K8_STEP_MIN;
                }

            }
        }

        // === [ Too Slow...
        else if (u16CurrentSpeed < u16DesiredSpeed)
        {
            // Check if greater than our allowable window...
            if (u16DesiredSpeed > (u16CurrentSpeed + u16RPMrange))
            {
                u16Difference = u16DesiredSpeed - u16CurrentSpeed;

                if (u16Difference > K8_STEP_MAX_RANGE)
                {
                    i16Duty += K8_STEP_MAX;
                }
                else if (u16Difference > K8_STEP_MID_RANGE)
                {
                    i16Duty += K8_STEP_MID;
                }
                else if (u16Difference > K8_STEP_LOW_RANGE)
                {
                    i16Duty += K8_STEP_LOW;
                }
                else
                {
                    i16Duty += K8_STEP_MIN;
                }

            }
        }

#if 1
#warning FAN SPEED FOR UT BUILD ONLY
        // ==== [ UT only!
        if (
               (FALSE == monitor_GetAcRangeLLStatus()
                && tADC_12V_IOUT.u16Avg > OUTPUT_I_SCALE(80.0))
           || (TRUE == monitor_GetAcRangeLLStatus()
                && tADC_12V_IOUT.u16Avg > OUTPUT_I_SCALE(40.0))
          ) {
#ifdef THERMAL_TEST
   //         i16Duty = FAN_PWM_MAX_PDC;
    //i16Duty = tADC_IOUT_AMP.u16Avg * 17;  //duty * (FAN_PWM_MAX_PDC_100);
#endif
            i16Duty = FAN_PWM_MAX_PDC;
        }
#endif
                // Enforce limits on our duty cycle...
        if (i16Duty < FAN_PWM_MIN_PDC)
        {
            i16Duty = FAN_PWM_MIN_PDC;
        }
        else if (i16Duty > FAN_PWM_MAX_PDC)
        {
            i16Duty = FAN_PWM_MAX_PDC;
        }
        
        // ==== [ Apply Duty Cycle...

        // Detect if fan was off, but is now turning on...
        if (PDC5 == 0)
        {
            u16FanOverdrive = K8_FAN_OVERDRIVE_COUNTER;
            PDC5 = (uint16) (FAN_PWM_PERIOD * 0.37);
            u16FanAlertCounter = 0;
            //fcFan1.u16CurrentDuty = K16_MIN_FAN_RPM;
            fcFan1.u16CurrentDuty = (uint16) (FAN_PWM_PERIOD * 0.37);

            // restart fanFault on any turning-on.
            psos_RestartTimer(&timerFanFault);

        }

        // Check if we're still overdriving (to ensure it spins up)...
        if (u16FanOverdrive > 0)
        {
            u16FanOverdrive--;
        }

        // Done, so use the desired duty now...
        else
        {
            fcFan1.u16CurrentDuty = (uint16) i16Duty;
            PDC5 = fcFan1.u16CurrentDuty;
//#if IP2M_DEBUG_CONTROL == DEBUG_DATA_REPORTING
//    u16i2cDebug7 = fcFan1.u16CurrentDuty;
//#endif
        }
    }

    // Fan is disabled...
    else
    {
        PDC5 = K16_FAN_DUTY_CYCLE_OFF;
    }

}

/*****************************************************************************
 * app_Fan_SetCurrentMinDuty
 * Set the minimum duty cycle on valid info,
 * 
 * retrun 1 if updated, else 0 */
uint16 app_Fan_SetCurrentMinDuty(uint16 u16duty, uint16 fan_n)
{
    uint16 Temp1;

    Temp1 = FAN_N_INVALID;
    if ((FAN_1 == fan_n) && (u16duty <= FAN_DUTY_CYCLE_LIMIT ))
    {
        // Determine RPM based on NFS curve...
        if (u16duty != 0)
        {
            fcFan1.u16SystemRPM = fancontrol_NFS_to_RPM(u16duty);
        }
        else
        {
            fcFan1.u16SystemRPM = 0;
        }

        fcFan1.u16SystemMinimum = u16duty;
        fcFan1.u16SystemSetDutyCycle = u16duty;
        Temp1 = FAN_1;
    }
    return Temp1;
}

bool app_FanIsSysMinActive(void)
{
    return (fcFan1.tFlags.bBits.bSystemOverride);
}

bool app_ManufacturingFanOverrideMet( void )
{
    // Per spec, cannot be:
    //  1.  12V Main outside regulation (also checking cathode for redundantly powered)
    //  2.  Latched due to something other than fan fault...
    
    if (I2C.IPMMdata.psu_manufacturing.bBits.bPSUManuFanPowerPinConfig
            && (
                !MACRO_IS_VIN_GOOD()         // VIN_GOOD de-asserted
                || (fault_IsVSENSEvalid()           // In standby and rail good
                    && MACRO_MAIN_IS_DISABLED()
                    && MACRO_SBY_IS_ENABLED())
                )
            && AppI2CIsFan1Faulted() == FALSE)
    {
        return TRUE;
    }

    return FALSE;

}

bool fancontrol_SystemOverride ( void )
{
    if (I2C.IPMMdata.psu_manufacturing.bBits.bPSUManuFanPowerPinConfig
            && MACRO_MAIN_IS_DISABLED())
    {
        if (fcFan1.u16SystemRPM > fcFan1.u16TemperatureRPM
            && (!MACRO_AC_IS_OK() || (
                                        fault_IsVSENSEvalidAC_Off() 
                                        && MACRO_MAIN_IS_DISABLED()
                                        && MACRO_SBY_IS_ENABLED())
                                    )
            && fault_FaultFlags.FAULTED == 0
            && I2C.IPMMdata.rapidon_cntrl.b_wake_status == 0)
        {
          return TRUE;
        }
        
    } else {
        if (fcFan1.u16SystemRPM > fcFan1.u16TemperatureRPM
          && MACRO_AC_IS_OK()
          && fault_FaultFlags.FAULTED == 0
          && I2C.IPMMdata.rapidon_cntrl.b_wake_status == 0)
        {
            return TRUE;
        }
    }

    return FALSE;

}

uint16 app_fancontrolDiagLoadSetPt(void)
{
    return u16LoadSetPoint;
}

uint16 fan_controlGetLastSetDutyCycle(void)
{
    return (fcFan1.u16SystemSetDutyCycle);
}

uint16 fan_controlGetRPMSetPt(void)
{
    return (fcFan1.u16SetPointRPM);
}






/*************************************************************************/

// -- "Real" values (ie, floating point...)
#define KD_NFS_m2k    ( (1.0 - .2)/(K16_NFS_100_PERCENT - K16_NFS_20_PERCENT))

#define KD_NFS_b2k    ( (K16_NFS_100_PERCENT *KD_NFS_m2k)-1)

// -- Equation to convert any "NFS duty cycle %" to real RPM...
#define K16_NFS_to_RPM2k(x) ((uint16) ( KD_NFS_m2k * x + KD_NFS_b2k))

#define K16_RPM2K_to_NFS(x) ((uint16) (x *KD_NFS_m2k + KD_NFS_b2k)
// -- Usable values (ie, not floating...)
#define K16_NFS_to_RPM_m2k   ((uint32) (1/KD_NFS_m2k)/100)
#define K16_NFS_to_RPM_b2k   ((int16)  (KD_NFS_b2k *120.0))

// -- scaled by 32768 (<<15 to scale small real value) * 100 to convert to integer %
#define K16_NFS_SCALE_m  (uint16)32768.0
#define K16_RPM_to_NFS_m2k ((uint32)(KD_NFS_m2k * K16_NFS_SCALE_m * 100.0))


#if LIMIT_TO_NFS == 2
/*****************************************************************************
 * fancontrol_LimitToNFSCurve
 * Developmnet code for NFS limts
 * Receives a thermal RPM
 * Returns a WORKING RPM below NFS limit at current load/ambient settings
 *
 *************************************************************************  */

uint16 fancontrol_LimitToNFSCurve( uint16 u16RPM )
{

    uint16 u16OperatingRPM;
    uint16 u16Ambient   = GET_PSInletTemperature();
    uint16 u16Load      = tADC_IOUT_AMP.u16Avg;
    uint16 u16NFS = 0;

    if (MACRO_MAIN_IS_DISABLED())
    {
        // Main is off, return thermal RPM
        return u16RPM;
    }

    if (TRUE == monitor_GetAcRangeLLStatus())
    {
        // low line scale the load current 2X
        u16Load = (u16Load<<1);
    }

    // Apply a large average on the IOUT to avoid responding to transients... was >> 2
    fcFan1.u16IoutAccum -= (fcFan1.u16IoutAccum >> 2);
    fcFan1.u16IoutAccum += u16Load;
    u16Load = fcFan1.u16IoutAccum >> 2;


    // Check for "high ambient" recovery...
    if (fcFan1.tFlags.bBits.bHighAmbient
         && (u16Ambient < 52))
    {
        fcFan1.tFlags.bBits.bHighAmbient = FALSE; // Recovered...
    }
    else if (fcFan1.tFlags.bBits.bHighAmbient)
    {
        // fcFan1.u16NFSLimitRPM was set in load curves
        return u16RPM;  // Don't limit NFS while flagged for "high ambient"
    }

    // ---- NFS Explanation:
    // 1.  Checks load curve (<=20%, <=50%, <=80%, <=100%(2kW)
    // 2.  Determines point on curve with ambient reading
    // 3.  Verifies current RPM is not above this value
    // 4.  Apply a load factor for % load below load curve
    // Check for 20% load curve...
    if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .26))
    {
        // no load factor for loads
        u16NFS = 18;    // Assume 20% load line first...

        // Starts to bend at 40degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        if (u16Ambient > 65)
        {
            // No calculaiton Beyond PTAVS curves, above 65 ambient
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_60_PERCENT_RPM;
            return u16RPM;
        }
        else if (u16Ambient > 58)
        {
            // Renge 2 Curve is approximated by 2.4% per degC above 55
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 45))-1885)>>11);
        }
        else if (u16Ambient > 44)
        {
            // Range 1 Curve is approximated by 1% per degC above 49
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 5))+7)>>10);
         }
       else
       {
            if (1 == fcFan1.tFlags.bBits.bStbyOCWtherm)
            {
                // Push minimum fan here to support bias module cooling needs
                u16RPM = K16_NFS_STBY_OC_RPM;   //K16_NFS_21_PERCENT;
                u16NFS = 29;
            }
            else
            {
                // min NFS for 25% load line
                u16NFS = 18;
            }
       }
    }
    // Check for 50% load curve...
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .54))
    {
        // Starts to bend at 35degC (fairly linear)
        // - No NFS curve if above 60degC (set to 62 for some margin)...
        //if (eDebugPinState == FIRMWAREflagF) MACRO_ASSERT_FIRMWARE_FLAG();
        if (u16Ambient > 63)
        {
            // No calculaiton Beyond PTAVS curves, above 65 ambient
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return u16RPM;
        }
        else if (u16Ambient > 58)
        {
            // Curve is approximated by 2.4% per degC above 55
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 123))-5481)>>11);
        }
        else if (u16Ambient > 48)
        {
            //
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 167))-4100)>>13);

       }
       else
       {
            if (u16Ambient >26)
            {
                u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 4))+24)>>9);
            }
            else
            {
                // min fan for 50% load line
                u16NFS = 57;
            }
       }
    }
    // Check for 80% load curve...
    // - Linear approximated with a "corner" at 42degC
    // - Care was taken to ensure this curve is less than the spec curve
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I * .85))
    {

//            if (eDebugPinState == FIRMWAREflagF) MACRO_DEASSERT_FIRMWARE_FLAG();
        if (u16Ambient > 51)
        {
            // No calculaiton Beyond PTAVS curves, above 65 ambient
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return u16RPM;
        }
        else if (u16Ambient > 47)
        {
            // Curve is approximated by 2.4% per degC above 55
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 2))-33)>>6);
        }
        else if (u16Ambient > 42)
        {
            // Curve is approximated by 1% per degC above 49
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 5))-12)>>8);

       }
       else
       {
            if (u16Ambient >26)
            {
                u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 5))+178)>>9);
            }
            else
            {
                // min fan for 80% load line
                u16NFS = 80;
            }
       }

    }
    else if (u16Load <= OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I105))
    {
//        if (eDebugPinState == FIRMWAREflagF) MACRO_DEASSERT_FIRMWARE_FLAG();
        if (u16Ambient > 48)
        {
            fcFan1.tFlags.bBits.bHighAmbient = TRUE; // Allows over 100% NFS until drops down again
            fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
            return K16_NFS_100_PERCENT;  // No NFS curve specified above 55degC...
        }

        if (u16Ambient > 26)
        {
            // 105% load curve
            u16NFS = (uint16)(((__builtin_muluu(u16Ambient, 9))+600)>>10);
        }
        else
        {
            // min NFS for 105% load line
            u16NFS = 80;
        }
        if (u16Load < OUTPUT_IOUT_AMP_SCALE(MAX_ISHARE_I95))
        {
            u16NFS -=5;
        }
    }
    else
    {
        // Over 110% load, no NFS curve specified...
        fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
        return K16_NFS_100_PERCENT;
    }   // end if/else uload


    // -- Now, convert the desired NFS to RPM
    u16NFS = fancontrol_NFS_to_RPM(u16NFS);



    // - Ramp to our NFS value...
    if (u16NFS > 0)
    {
        if (fcFan1.u16NFSLimitRPM <= u16NFS)
        {
            // Allow to "step up" fast...
            fcFan1.u16NFSLimitRPM = u16NFS;
        }
        else
        {
            // Must "step down" slowly...
            if ((fcFan1.u16NFSLimitRPM - u16NFS) < 500)
            {
                fcFan1.u16NFSLimitRPM = u16NFS;
            }
            else
            {
                fcFan1.u16NFSLimitRPM -= 500;
            }
        }
    }
    else
    {
         fcFan1.u16NFSLimitRPM = K16_NFS_100_PERCENT;
         return u16RPM;
    }

    // Cap thermal RPM to NFS limit ...
    if (u16RPM > fcFan1.u16NFSLimitRPM)
    {
        return fcFan1.u16NFSLimitRPM;
    }
    else
    {
        // set operating at 94% of NFS limit (60/64) to reduce thermal overshoot.
        u16OperatingRPM = (uint16)((__builtin_muluu(fcFan1.u16NFSLimitRPM, (uint16)60) >> 6));

        if (K16_NFS_17_PERCENT_RPM > u16OperatingRPM) u16OperatingRPM = K16_NFS_17_PERCENT_RPM;

        if (u16RPM >= u16OperatingRPM)
        {
            u16OperatingRPM = u16RPM;
        }

        return u16OperatingRPM;
    }
}

#endif
//EOF
