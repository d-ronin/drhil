// FGMacOSXEventInput.cxx -- handle event driven input devices for Mac OS X
//
// Written by Tatsuhiro Nishioka, started Aug. 2009.
//
// Copyright (C) 2009 Tasuhiro Nishioka, tat <dot> fgmacosx <at> gmail <dot> com
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
//
// $Id$

#include "FGMacOSXEventInput.hxx"


#define GetHIDElementLongValue(element, key) ({ \
      long value = 0; \
      CFNumberRef refType = (CFNumberRef)CFDictionaryGetValue(element, CFSTR(key)); \
      if (refType) { CFNumberGetValue(refType, kCFNumberLongType, &value); } \
      value; })

#define GetHIDElementBooleanValue(element, key) ({ \
      bool value = 0; \
      CFBooleanRef refType = (CFBooleanRef)CFDictionaryGetValue(element, CFSTR(key)); \
      if (refType) { value = CFBooleanGetValue(refType); } \
      value; })

#define GetHIDElementStringValue(element, key) ({ \
      const char *buf = ""; \
      CFStringRef refType = (CFStringRef)CFDictionaryGetValue(element, CFSTR(key)); \
      if (refType) { \
        buf = CFStringGetCStringPtr(refType, CFStringGetSystemEncoding());  \
       } \
      buf; })

// HID Element Types (log / debug use)
struct HIDTypes HID_TYPE_TABLE[] = {
  {kIOHIDElementTypeInput_Misc, kHIDElementType, "Input Misc"},
  {kIOHIDElementTypeInput_Button, kHIDElementType, "Input Button"},
  {kIOHIDElementTypeInput_Axis, kHIDElementType, "Input Axis"},
  {kIOHIDElementTypeInput_ScanCodes, kHIDElementType, "Input ScanCodes"},
  {kIOHIDElementTypeOutput, kHIDElementType, "Output"},
  {kIOHIDElementTypeFeature, kHIDElementType, "Feature"},
  {kIOHIDElementTypeCollection, kHIDElementType, "Collection"},
  {-1, kHIDElementType, ""}
};

// HID Element Pages (log / debug use)
struct HIDTypes HID_PAGE_TABLE[] = {
  // Page
  {kHIDPage_GenericDesktop, kHIDElementPage, "GenericDesktop"},
  {kHIDPage_Simulation, kHIDElementPage, "Simulation Controls"},
  {kHIDPage_VR, kHIDElementPage, "VR Controls"},
  {kHIDPage_Sport, kHIDElementPage, "Sport Controls"},
  {kHIDPage_Game, kHIDElementPage, "Game Controls"},
  {0x06, kHIDElementPage, "Generic Device Controls"},
  {kHIDPage_KeyboardOrKeypad, kHIDElementPage, "KeyboardOrKeypad"},
  {kHIDPage_LEDs, kHIDElementPage, "LEDs"},
  {kHIDPage_Button, kHIDElementPage, "Button"},
  {kHIDPage_Ordinal, kHIDElementPage, "Ordinal"},
  {kHIDPage_Telephony, kHIDElementPage, "Telephony"},
  {kHIDPage_Consumer, kHIDElementPage, "Consumer"},
  {kHIDPage_Digitizer, kHIDElementPage, "Digitizer"},
  {kHIDPage_PID, kHIDElementPage, "PID"},
  {kHIDPage_VendorDefinedStart, kHIDElementPage, "VendorDefinedStart"},
  {-1, kHIDElementPage, ""}
};

#define USAGE_KEY(PAGE,USAGE) (PAGE << 16 | USAGE)
#define GET_PAGE(KEY)  (KEY >> 16)
#define GET_USAGE(KEY) (KEY & 0xFFFF)

#define GD_USAGE(USAGE) USAGE_KEY(kHIDPage_GenericDesktop, USAGE)
#define SIM_USAGE(USAGE) USAGE_KEY(kHIDPage_GenericDesktop, USAGE)
#define VR_USAGE(USAGE) USAGE_KEY(kHIDPage_VR, USAGE)
#define SP_USAGE(USAGE) USAGE_KEY(kHIDPage_Sport, USAGE)
#define GAME_USAGE(USAGE) USAGE_KEY(kHIDPage_Game, USAGE)
#define GDC_USAGE(USAGE) USAGE_KEY(0x06, USAGE) // Generic Device Control is "Reserved" in Apple's definition
#define KEY_USAGE(USAGE) USAGE_KEY(kHIDPage_KeyboardOrKeypad, USAGE)
#define LED_USAGE(USAGE) USAGE_KEY(kHIDPage_LEDs, USAGE)
#define BUTTON_USAGE(USAGE) USAGE_KEY(kHIDPage_Button, USAGE)
#define DIG_USAGE(USAGE) USAGE_KEY(kHIDPage_Digitizer, USAGE)
#define CON_USAGE(USAGE) USAGE_KEY(kHIDPage_Consumer, USAGE)

// HID Element Usage <-> FGEventData convertion data
struct HIDTypes HID_USAGE_TABLE[] = {
  // Generic Desktop Page
  {GD_USAGE(kHIDUsage_GD_X), kHIDUsageAxis, "x-translate"},
  {GD_USAGE(kHIDUsage_GD_Y), kHIDUsageAxis, "y-translate"},
  {GD_USAGE(kHIDUsage_GD_Z), kHIDUsageAxis, "z-translate"},  
  {GD_USAGE(kHIDUsage_GD_Rx), kHIDUsageAxis, "x-rotate"},  
  {GD_USAGE(kHIDUsage_GD_Ry), kHIDUsageAxis, "y-rotate"},
  {GD_USAGE(kHIDUsage_GD_Rz), kHIDUsageAxis, "z-rotate"},
  {GD_USAGE(kHIDUsage_GD_Slider), kHIDUsageAxis, "slider"},
  {GD_USAGE(kHIDUsage_GD_Dial), kHIDUsageAxis, "dial"},
  {GD_USAGE(kHIDUsage_GD_Wheel), kHIDUsageAxis, "wheel"},
  {GD_USAGE(kHIDUsage_GD_Hatswitch), kHIDUsageHat, "hat"},
  
  {GD_USAGE(kHIDUsage_GD_CountedBuffer), kHIDUsageNotSupported, "counted-buffer"},
  {GD_USAGE(kHIDUsage_GD_ByteCount), kHIDUsageNotSupported, "byte-count"},
  {GD_USAGE(kHIDUsage_GD_MotionWakeup), kHIDUsageDF, "motion-wakeup"},
  {GD_USAGE(kHIDUsage_GD_Start), kHIDUsageOOC, "button-start"},
  {GD_USAGE(kHIDUsage_GD_Select), kHIDUsageOOC, "button-select"},
  {GD_USAGE(kHIDUsage_GD_Vx), kHIDUsageAxis, "x-vector"},
  {GD_USAGE(kHIDUsage_GD_Vy), kHIDUsageAxis, "y-vector"},
  {GD_USAGE(kHIDUsage_GD_Vz), kHIDUsageAxis, "z-vector"},
  {GD_USAGE(kHIDUsage_GD_Vbrx), kHIDUsageAxis, "x-rel-vector"},
  {GD_USAGE(kHIDUsage_GD_Vbry), kHIDUsageAxis, "y-rel-vector"},
  {GD_USAGE(kHIDUsage_GD_Vbrz), kHIDUsageAxis, "z-rel-vector"},
  {GD_USAGE(kHIDUsage_GD_Vno),  kHIDUsageAxis, "no-vector"},

  {GD_USAGE(kHIDUsage_GD_SystemPowerDown), kHIDUsageOSC, "button-system-power-down"},
  {GD_USAGE(kHIDUsage_GD_SystemSleep), kHIDUsageOSC, "button-system-sleep"},
  {GD_USAGE(kHIDUsage_GD_SystemWakeUp), kHIDUsageOSC, "button-system-wake-up"},
  {GD_USAGE(kHIDUsage_GD_SystemContextMenu), kHIDUsageOSC, "button-system-context-menu"},
  {GD_USAGE(kHIDUsage_GD_SystemMainMenu), kHIDUsageOSC, "button-system-main-menu"},
  {GD_USAGE(kHIDUsage_GD_SystemAppMenu), kHIDUsageOSC, "button-system-app-menu"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuHelp), kHIDUsageOSC, "button-system-menu-help"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuExit), kHIDUsageOSC, "button-system-menu-exit"},
  {GD_USAGE(kHIDUsage_GD_SystemMenu), kHIDUsageOSC, "button-system-menu"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuRight), kHIDUsageRTC, "button-system-menu-right"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuLeft), kHIDUsageRTC, "button-system-menu-left"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuUp), kHIDUsageRTC, "button-system-menu-up"},
  {GD_USAGE(kHIDUsage_GD_SystemMenuDown), kHIDUsageRTC, "button-system-menu-down"},
  {GD_USAGE(kHIDUsage_GD_DPadUp), kHIDUsageOOC, "dpad-up"},
  {GD_USAGE(kHIDUsage_GD_DPadDown), kHIDUsageOOC, "dpad-down"},
  {GD_USAGE(kHIDUsage_GD_DPadRight), kHIDUsageOOC, "dpad-right"},
  {GD_USAGE(kHIDUsage_GD_DPadLeft), kHIDUsageOOC, "dpad-left"},

  // Game Controls Page
  {GAME_USAGE(kHIDUsage_Game_TurnRightOrLeft), kHIDUsageAxis, "turn"},
  {GAME_USAGE(kHIDUsage_Game_PitchUpOrDown), kHIDUsageAxis, "pitch"},
  {GAME_USAGE(kHIDUsage_Game_MoveRightOrLeft), kHIDUsageAxis, "x-move"},
  {GAME_USAGE(kHIDUsage_Game_MoveForwardOrBackward), kHIDUsageAxis, "y-move"},
  {GAME_USAGE(kHIDUsage_Game_MoveUpOrDown), kHIDUsageAxis, "z-move"},
  {GAME_USAGE(kHIDUsage_Game_LeanRightOrLeft), kHIDUsageAxis, "x-lean"},
  {GAME_USAGE(kHIDUsage_Game_LeanForwardOrBackward), kHIDUsageAxis, "z-lean"},

  // General Control Devices Page
  {GDC_USAGE(0x20), kHIDUsageDV, "battery-strength"},
  {GDC_USAGE(0x21), kHIDUsageDV, "wireless-channel"},
  {GDC_USAGE(0x22), kHIDUsageDV, "wireless-id"},
  {GDC_USAGE(0x23), kHIDUsageDV, "discover-wireless-control"},
  {GDC_USAGE(0x24), kHIDUsageOSC, "security-code-character-entered"},
  {GDC_USAGE(0x25), kHIDUsageOSC, "security-code-character-erased"},
  {GDC_USAGE(0x26), kHIDUsageOSC, "security-code-cleared"},

  // Simulation Controls Page
  {SIM_USAGE(kHIDUsage_Sim_Aileron), kHIDUsageAxis, "aileron"},
  {SIM_USAGE(kHIDUsage_Sim_AileronTrim), kHIDUsageAxis, "aileron-trim"},
  {SIM_USAGE(kHIDUsage_Sim_AntiTorqueControl), kHIDUsageAxis, "anti-torque-control"},
  {SIM_USAGE(kHIDUsage_Sim_AutopilotEnable), kHIDUsageOOC, "button-autopilot-enable"},
  {SIM_USAGE(kHIDUsage_Sim_ChaffRelease), kHIDUsageOSC, "button-chaff-release"},
  {SIM_USAGE(kHIDUsage_Sim_CollectiveControl), kHIDUsageAxis, "collective-control"},
  {SIM_USAGE(kHIDUsage_Sim_DiveBrake), kHIDUsageAxis, "dive-brake"},
  {SIM_USAGE(kHIDUsage_Sim_ElectronicCountermeasures), kHIDUsageOOC, "electronic-countermeasures"}, // OOC
  {SIM_USAGE(kHIDUsage_Sim_Elevator), kHIDUsageAxis, "elevator"},
  {SIM_USAGE(kHIDUsage_Sim_ElevatorTrim), kHIDUsageAxis, "elevator-trim"},
  {SIM_USAGE(kHIDUsage_Sim_Rudder), kHIDUsageAxis, "rudder"},
  {SIM_USAGE(kHIDUsage_Sim_Throttle), kHIDUsageAxis, "throttle"},
  {SIM_USAGE(kHIDUsage_Sim_FlightCommunications), kHIDUsageOOC, "button-flight-communications"}, // OOC
  {SIM_USAGE(kHIDUsage_Sim_FlareRelease), kHIDUsageOSC, "button-flare-release"},
  {SIM_USAGE(kHIDUsage_Sim_LandingGear), kHIDUsageOOC, "button-landing-gear"}, // OOC
  {SIM_USAGE(kHIDUsage_Sim_ToeBrake), kHIDUsageAxis, "toe-brake"},
  {SIM_USAGE(kHIDUsage_Sim_Trigger), kHIDUsageMC, "button-trigger"},
  {SIM_USAGE(kHIDUsage_Sim_WeaponsArm), kHIDUsageOOC, "button-weapons-arm"}, // OOC
  {SIM_USAGE(kHIDUsage_Sim_Weapons), kHIDUsageOSC, "button-weapons"},
  {SIM_USAGE(kHIDUsage_Sim_WingFlaps), kHIDUsageAxis, "wing-flaps"},  // DV
  {SIM_USAGE(kHIDUsage_Sim_Accelerator), kHIDUsageAxis, "accelerator"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_Brake), kHIDUsageAxis, "brake"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_Clutch), kHIDUsageAxis, "clutch"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_Shifter), kHIDUsageAxis, "shifter"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_Steering), kHIDUsageAxis, "steering"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_TurretDirection), kHIDUsageAxis, "turret-direction"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_BarrelElevation), kHIDUsageAxis, "barrel-elevation"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_DivePlane), kHIDUsageAxis, "dive-plane"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_Ballast), kHIDUsageAxis, "ballast"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_BicycleCrank), kHIDUsageAxis, "bicycle-crank"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_HandleBars), kHIDUsageAxis, "handle-bars"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_FrontBrake), kHIDUsageAxis, "front-brake"}, // DV
  {SIM_USAGE(kHIDUsage_Sim_RearBrake), kHIDUsageAxis, "rear-brake"}, // DV

  // Digitizer Controls Page
  {DIG_USAGE(kHIDUsage_Dig_TipPressure), kHIDUsageAxis, "tip-pressure"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_BarrelPressure), kHIDUsageAxis, "barrel-pressure"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_InRange), kHIDUsageMC, "in-range"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_Touch), kHIDUsageMC, "touch"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_Untouch), kHIDUsageOSC, "button-untouch"}, // OSC
  {DIG_USAGE(kHIDUsage_Dig_Tap), kHIDUsageOSC, "button-tap"}, // OSC
  {DIG_USAGE(kHIDUsage_Dig_Quality), kHIDUsageDV, "quality"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_DataValid), kHIDUsageDV, "button-data-valid"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_TransducerIndex), kHIDUsageDV, "transducer-index"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_BatteryStrength), kHIDUsageDV, "battery-strength"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_Invert), kHIDUsageMC, "invert"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_XTilt), kHIDUsageAxis, "x-tilt"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_YTilt), kHIDUsageAxis, "y-tilt"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_Azimuth), kHIDUsageAxis, "azimuth"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_Altitude), kHIDUsageAxis, "altitude"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_Twist), kHIDUsageAxis, "twist"}, // DV
  {DIG_USAGE(kHIDUsage_Dig_TipSwitch), kHIDUsageMC, "button-tipswitch"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_SecondaryTipSwitch), kHIDUsageMC, "button-secondary-tipswitch"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_BarrelSwitch), kHIDUsageMC, "button-barrelswitch"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_Eraser), kHIDUsageMC, "eraser"}, // MC
  {DIG_USAGE(kHIDUsage_Dig_TabletPick), kHIDUsageMC, "table-pick"}, // MC

 // Consumer Page
  {CON_USAGE(kHIDUsage_Csmr_Plus10), kHIDUsageOSC, "plus10"},
  {CON_USAGE(kHIDUsage_Csmr_Plus100), kHIDUsageOSC, "plus100"},
  {CON_USAGE(kHIDUsage_Csmr_AMOrPM), kHIDUsageOSC, "am-pm"},
  {CON_USAGE(kHIDUsage_Csmr_Power), kHIDUsageOOC, "power"},
  {CON_USAGE(kHIDUsage_Csmr_Reset), kHIDUsageOSC, "reset"},
  {CON_USAGE(kHIDUsage_Csmr_Sleep), kHIDUsageOSC, "sleep"},
  {CON_USAGE(kHIDUsage_Csmr_SleepAfter), kHIDUsageOSC, "sleep-after"},
  {CON_USAGE(kHIDUsage_Csmr_SleepMode), kHIDUsageRTC, "sleep-mode"},
  {CON_USAGE(kHIDUsage_Csmr_Illumination), kHIDUsageOOC, "illumination"},
  {CON_USAGE(kHIDUsage_Csmr_Menu), kHIDUsageOOC, "menu"},
  {CON_USAGE(kHIDUsage_Csmr_MenuPick), kHIDUsageOSC, "menu-pick"},
  {CON_USAGE(kHIDUsage_Csmr_MenuUp), kHIDUsageOSC, "menu-up"},
  {CON_USAGE(kHIDUsage_Csmr_MenuDown), kHIDUsageOSC, "menu-down"},
  {CON_USAGE(kHIDUsage_Csmr_MenuLeft), kHIDUsageOSC, "menu-left"},
  {CON_USAGE(kHIDUsage_Csmr_MenuRight), kHIDUsageOSC, "menu-right"},
  {CON_USAGE(kHIDUsage_Csmr_MenuEscape), kHIDUsageOSC, "menu-escape"},
  {CON_USAGE(kHIDUsage_Csmr_MenuValueIncrease), kHIDUsageOSC, "menu-value-increase"},
  {CON_USAGE(kHIDUsage_Csmr_MenuValueDecrease), kHIDUsageOSC, "menu-value-decrease"},
  {CON_USAGE(kHIDUsage_Csmr_DataOnScreen), kHIDUsageOOC, "data-on-screen"},
  {CON_USAGE(kHIDUsage_Csmr_ClosedCaption), kHIDUsageOOC, "closed-caption"},
  {CON_USAGE(kHIDUsage_Csmr_ClosedCaptionSelect), kHIDUsageSel, "closed-caption-select"},
  {CON_USAGE(kHIDUsage_Csmr_VCROrTV), kHIDUsageOOC, "vcr-tv"},
  {CON_USAGE(kHIDUsage_Csmr_BroadcastMode), kHIDUsageOSC, "broadcast-mode"},
  {CON_USAGE(kHIDUsage_Csmr_Snapshot), kHIDUsageOSC, "snapshot"},
  {CON_USAGE(kHIDUsage_Csmr_Still), kHIDUsageOSC, "still"},
  {CON_USAGE(kHIDUsage_Csmr_Assign), kHIDUsageOSC, "assign"},
  {CON_USAGE(kHIDUsage_Csmr_ModeStep), kHIDUsageOSC, "mode-step"},
  {CON_USAGE(kHIDUsage_Csmr_RecallLast), kHIDUsageOSC, "recall-last"},
  {CON_USAGE(kHIDUsage_Csmr_EnterChannel), kHIDUsageOSC, "enter-channel"},
  {CON_USAGE(kHIDUsage_Csmr_OrderMovie), kHIDUsageOSC, "order-movie"},
  {CON_USAGE(kHIDUsage_Csmr_Channel), kHIDUsageDV, "channel"}, // LC
  {CON_USAGE(kHIDUsage_Csmr_MediaSelection), kHIDUsageSel, "media-selection"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectComputer), kHIDUsageSel, "media-select-computer"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectTV), kHIDUsageSel, "media-select-tv"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectWWW), kHIDUsageSel, "media-seleci-www"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectDVD), kHIDUsageSel, "media-select-dvd"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectTelephone), kHIDUsageSel, "media-select-telephone"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectProgramGuide), kHIDUsageSel, "media-select-programguide"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectVideoPhone), kHIDUsageSel, "media-select-videophone"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectGames), kHIDUsageSel, "media-select-games"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectMessages), kHIDUsageSel, "media-select-messages"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectCD), kHIDUsageSel, "media-select-cd"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectVCR), kHIDUsageSel, "media-select-vcr"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectTuner), kHIDUsageOSC, "media-select-tuner"},
  {CON_USAGE(kHIDUsage_Csmr_Quit), kHIDUsageOSC, "quit"},
  {CON_USAGE(kHIDUsage_Csmr_Help), kHIDUsageOOC, "help"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectTape), kHIDUsageSel, "media-select-tape"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectCable), kHIDUsageSel, "media-select-cable"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectSatellite), kHIDUsageSel, "media-select-satellite"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectSecurity), kHIDUsageSel, "media-select-security"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectHome), kHIDUsageSel, "media-select-home"},
  {CON_USAGE(kHIDUsage_Csmr_MediaSelectCall), kHIDUsageSel, "media-select-call"},
  {CON_USAGE(kHIDUsage_Csmr_ChannelIncrement), kHIDUsageOSC, "channel-increment"},
  {CON_USAGE(kHIDUsage_Csmr_ChannelDecrement), kHIDUsageOSC, "channel-decrement"},
  {CON_USAGE(kHIDUsage_Csmr_Media), kHIDUsageSel, "media"},
  {CON_USAGE(kHIDUsage_Csmr_VCRPlus), kHIDUsageOSC, "vcr-plus"},
  {CON_USAGE(kHIDUsage_Csmr_Once), kHIDUsageOSC, "once"},
  {CON_USAGE(kHIDUsage_Csmr_Daily), kHIDUsageOSC, "daily"},
  {CON_USAGE(kHIDUsage_Csmr_Weekly), kHIDUsageOSC, "weekly"},
  {CON_USAGE(kHIDUsage_Csmr_Monthly), kHIDUsageOSC, "monthly"},
  {CON_USAGE(kHIDUsage_Csmr_Play), kHIDUsageOOC, "play"},
  {CON_USAGE(kHIDUsage_Csmr_Pause), kHIDUsageOOC, "pause"},
  {CON_USAGE(kHIDUsage_Csmr_Record), kHIDUsageOOC, "record"},
  {CON_USAGE(kHIDUsage_Csmr_FastForward), kHIDUsageOOC,"fastforward"},
  {CON_USAGE(kHIDUsage_Csmr_Rewind), kHIDUsageOOC, "rewind"},
  {CON_USAGE(kHIDUsage_Csmr_ScanNextTrack), kHIDUsageOSC, "scan-next-track"},
  {CON_USAGE(kHIDUsage_Csmr_ScanPreviousTrack), kHIDUsageOSC, "scan-previous-track"},
  {CON_USAGE(kHIDUsage_Csmr_Stop), kHIDUsageOSC, "stop"},
  {CON_USAGE(kHIDUsage_Csmr_Eject), kHIDUsageOSC, "eject"},
  {CON_USAGE(kHIDUsage_Csmr_RandomPlay), kHIDUsageOOC, "random-play"},
  {CON_USAGE(kHIDUsage_Csmr_SelectDisc), kHIDUsageNotSupported, "select-disc"}, //NamedArray

  {CON_USAGE(kHIDUsage_Csmr_VolumeIncrement), kHIDUsageRTC, "volume-increment"},
  {CON_USAGE(kHIDUsage_Csmr_VolumeDecrement), kHIDUsageRTC, "volume-decrement"},
  {CON_USAGE(kHIDUsage_Csmr_PlayOrPause), kHIDUsageOSC, "play-pause"},
  {CON_USAGE(kHIDUsage_Csmr_Mute), kHIDUsageOOC, "mute"},
  // Too many... and the rest are T.B.D. ;-)
  {-1, kHIDElementPage, ""},
};

static HIDTypeByID hidTypeByID(HID_TYPE_TABLE);
static HIDTypeByID hidPageByID(HID_PAGE_TABLE);
static HIDTypeByID hidUsageByID(HID_USAGE_TABLE);


HIDElement::HIDElement(CFDictionaryRef element, long page, long usage) : 
  page(page), usage(usage), value(0.0), lastValue(0.0) {

  cookie = (IOHIDElementCookie)GetHIDElementLongValue(element, kIOHIDElementCookieKey);
  name = hidUsageByID.getName(USAGE_KEY(page, usage));
}

float HIDElement::readStatus(IOHIDDeviceInterface **interface)
{
  IOHIDEventStruct event;
  lastValue = value;
  IOReturn ret = (*interface)->getElementValue(interface, cookie, &event);
  if (ret == kIOReturnSuccess) {
    value = event.value;
//    SG_LOG(SG_INPUT, SG_BULK, "Element: " << name << "; value = " << value);
    return (float)event.value;
  } else {
    SG_LOG(SG_INPUT, SG_ALERT, "Failed reading value for HID Element: " << name);
    return 0.0;
  }
}

bool HIDElement::isUpdated()
{
  return (value != lastValue);
}

void HIDElement::generateEvent(FGMacOSXInputDevice *device, double dt, int modifiers)
{
  SG_LOG(SG_INPUT, SG_DEBUG, "Generating Input Event: " << name << "=" << value);
  FGMacOSXEventData eventData(name, value, dt, modifiers);
  device->HandleEvent(eventData);
}

AxisElement::AxisElement(CFDictionaryRef element, long page, long usage) : 
  HIDElement(element, page, usage), dead_band(0.00), saturate(1.0)
{
  //  long scaledmin, scaledmax;
  min = GetHIDElementLongValue(element, kIOHIDElementMinKey); 
  max = GetHIDElementLongValue(element, kIOHIDElementMaxKey);
  isRelative = GetHIDElementBooleanValue(element, kIOHIDElementIsRelativeKey);
  isWrapping = GetHIDElementBooleanValue(element, kIOHIDElementIsWrappingKey);
  isNonLinear = GetHIDElementBooleanValue(element, kIOHIDElementIsNonLinearKey);
  cout << "isRelative=" << isRelative << ", isWrapping=" << isWrapping << ", isNonLinear=" << isNonLinear << endl;

  name = ((isRelative == true) ? "rel-" : "abs-") + name;

  center = min + (max - min) / 2;
  SG_LOG(SG_INPUT, SG_DEBUG, "HID Axis Element; " << name << " min: " << min << " max:" << max << " center: " << center);
}

float AxisElement::readStatus(IOHIDDeviceInterface **interface)
{
  lastValue = value;
  value = HIDElement::readStatus(interface);
  return value;
/*
  if (!isRelative) {
    value = (value - (float)center) / (float)(max - center);
    if (fabs(value) < dead_band)
      value = 0.0;
  }
*/
  
}


ButtonElement::ButtonElement(CFDictionaryRef element, long page, long usage) :
  HIDElement(element, page, usage) 
{
  if (name == "") {
    stringstream ss;
    ss << (page == kHIDPage_KeyboardOrKeypad ? "keyboard-" : "button-") << usage;
    ss >> name;
  }
}

HatElement::HatElement(CFDictionaryRef element, long page, long usage, int id) :
  HIDElement(element, page, usage), id(id)
{
  min = GetHIDElementLongValue(element, kIOHIDElementMinKey);
  max = GetHIDElementLongValue(element, kIOHIDElementMaxKey);
  lastValue = 8;
}

void HatElement::generateEvent(FGMacOSXInputDevice *device, double dt, int modifiers)
{
  // Hat Value is from 0 to 8, representing:
  // 0:N, 1:NE, 2:E, 3:SE, 4:S, 5:SW, 6:W, 7:NW, 8:N
  static float xvalues[] = {0, 1, 1,  1,  0, -1, -1, -1, 0};
  static float yvalues[] = {1, 1, 0, -1, -1, -1,  0,  1, 0};
  stringstream ss;
  string eventName;
  ss << "abs-hat" << id << "-x";
  ss >> eventName;
  SG_LOG(SG_INPUT, SG_BULK, "Generating Input Event: " << eventName << "=" << xvalues[(int)value]);
  FGMacOSXEventData eventDataX(eventName, xvalues[(int)value], dt, modifiers);
  ss << "abs-hat" << id << "-y";
  ss >> eventName;
  SG_LOG(SG_INPUT, SG_BULK, "Generating Input Event: " << eventName << "=" << yvalues[(int)value]);
  FGMacOSXEventData eventDataY(eventName, yvalues[(int)value], dt, modifiers);
  device->HandleEvent((FGEventData &)eventDataX);
  device->HandleEvent((FGEventData &)eventDataY);
}


LEDElement::LEDElement(CFDictionaryRef element, long page, long usage) :
  HIDElement(element, page, usage)
{
  stringstream ss;
  if (name == "") {
    ss << "led-" << usage;
    ss >> name;
  }
}

void LEDElement::write(IOHIDDeviceInterface **interface, double value) {
  IOHIDEventStruct event = (IOHIDEventStruct){kIOHIDElementTypeOutput, cookie, 0, {0}, 0, 0};
  event.value = value;
  (*interface)->setElementValue(interface, cookie, &event, 0, NULL, NULL, NULL);
}

//
// This is just for testing....
//
FeatureElement::FeatureElement(CFDictionaryRef element, long page, long usage, int count=1) :
  HIDElement(element, page, usage) 
{
  stringstream ss;
  if (name == "") {
    ss << "feature-" << usage;
    if (count > 1) 
      ss << "-" << count;
    ss >> name;
  }
}

float FeatureElement::readStatus(IOHIDDeviceInterface **interface) {
  IOHIDEventStruct event;
  IOReturn ret = (*interface)->queryElementValue(interface, cookie, &event, 0, NULL, NULL, NULL);
  if (ret != kIOReturnSuccess) {
    ret = (*interface)->getElementValue(interface, cookie, &event);
    if (ret != kIOReturnSuccess) {
      SG_LOG(SG_INPUT, SG_ALERT, "Can't get element value for feature element: " << getName());
      return 0.0;
    }
  }
  cout << getName() << "=" << event.value << endl;
  return event.value;
}

//
// HIDElementFactory
//
void HIDElementFactory::create(CFTypeRef element, FGMacOSXInputDevice *inputDevice)
{
  assert(CFGetTypeID(element) == CFArrayGetTypeID());
  CFRange range = {0, CFArrayGetCount((CFArrayRef)element)};
  CFArrayApplyFunction((CFArrayRef) element, range, 
		       HIDElementFactory::elementEnumerator, (void *)inputDevice);
};


void HIDElementFactory::elementEnumerator( const void *element, void *inputDevice) {
  if (CFGetTypeID((CFTypeRef) element) != CFDictionaryGetTypeID()) {
    SG_LOG(SG_INPUT, SG_WARN, "Element Enumerator passed non-dictionary value.");
  }
  HIDElementFactory::parseElement((CFDictionaryRef)element, (FGMacOSXInputDevice *)inputDevice);
};


void HIDElementFactory::parseElement(CFDictionaryRef element, FGMacOSXInputDevice *inputDevice) {
  long page = GetHIDElementLongValue(element, kIOHIDElementUsagePageKey);
  long usage = GetHIDElementLongValue(element, kIOHIDElementUsageKey);

  static int id=0;
  static map<FGMacOSXInputDevice *, map<long, unsigned> > elementCount;

  long type = GetHIDElementLongValue(element, kIOHIDElementTypeKey);

  if (type == kIOHIDElementTypeCollection) {
    id = 0;
    cout << "Collection: " << hidTypeByID.getName(type) << "(" << type << ")" 
              <<":" << hidPageByID.getName(page) << "(" << page << ")" 
              << ":" << hidUsageByID.getName(USAGE_KEY(page, usage)) << "(" << usage << ")" << endl;
    HIDElementFactory::create(CFDictionaryGetValue(element, CFSTR(kIOHIDElementKey)), inputDevice);
  } else {
    HIDUsageType usageType = hidUsageByID.getType(USAGE_KEY(page, usage));
    // FIXME: Any other elegant way for counting the same usage on a device?
    // This is mainly needed for feature / hat elements to avoid assigning the sane event name.
    elementCount[inputDevice][USAGE_KEY(page, usage)] += 1;
    
    switch (usageType) {
      case kHIDUsageAxis:
	  inputDevice->addElement(new AxisElement(element, page, usage));
          break;
      case kHIDUsageDV:
      case kHIDUsageDF:
	  inputDevice->addElement(new HIDElement(element, page, usage));
          break;
      case kHIDUsageHat:
	  inputDevice->addElement(new HatElement(element, page, usage, elementCount[inputDevice][USAGE_KEY(page, usage)]));
          break;
      case kHIDUsageOOC:
      case kHIDUsageOSC:
      case kHIDUsageMC:
      case kHIDUsageRTC:
          if (usage > 0)
            inputDevice->addElement(new ButtonElement(element, page, usage));
          break;
          
      default:
        if (page == kHIDPage_Button || type == kIOHIDElementTypeInput_Button && usage > 0) {
        // FIXME: most of KeyboardOrKeypad elements should be treated as Selector type, not as Button...
	  inputDevice->addElement(new ButtonElement(element, page, usage));
        } else if (page == kHIDPage_LEDs && usage > 0) {
	  inputDevice->addElement(new LEDElement(element, page, usage));
/*
        } else if (type == kIOHIDElementTypeFeature) {
          // just for testing feature elements
          inputDevice->addElement(new FeatureElement(element, page, usage, elementCount[inputDevice][USAGE_KEY(page, usage)]));
*/
        } else {
          cout << "HID Element Page/Usage is not supported: type=" << hidTypeByID.getName(type) << "(" << type << ")"
                    << ", page=" << hidPageByID.getName(page) << "(" << page << ")" << ", usage=" << usage << endl;
        }
    }
  }
}

//
// FGMacOSXInputDevice implementation
//

FGMacOSXInputDevice::FGMacOSXInputDevice(io_object_t device) : device(device), devInterface(NULL)
{
  CFDictionaryRef properties = getProperties();
  string deviceName = GetHIDElementStringValue(properties, kIOHIDProductKey);
  if (deviceName == "") {
    deviceName = GetHIDElementStringValue(properties, "USB Product Name");
  }

  SetName(deviceName);
  CFRelease(properties);
}

const char *FGMacOSXInputDevice::TranslateEventName(FGEventData &eventData)
{
  FGMacOSXEventData &macEvent = (FGMacOSXEventData &)eventData;
  return macEvent.name.c_str();
}


void FGMacOSXInputDevice::Send(const char *eventName, double value)
{
  HIDElement *element = elements[eventName];
  if (element) {
    element->write(devInterface, value);
  } else {
    cout << "No element to handle event: " << eventName << endl;
  }
}


CFDictionaryRef FGMacOSXInputDevice::getProperties(io_object_t device)
{
  IOReturn ret;
  CFMutableDictionaryRef properties;

  ret = IORegistryEntryCreateCFProperties( device, &properties, kCFAllocatorDefault, kNilOptions);
  if (ret != kIOReturnSuccess || !properties) {
    SG_LOG(SG_INPUT, SG_WARN, "Error getting device properties.");
    return NULL;
  }

  return properties;
}


void FGMacOSXInputDevice::addElement(HIDElement *element)
{
  elements[element->getName()] = element;
  int count = elements.size();
  SG_LOG(SG_INPUT, SG_DEBUG, "adding element " << count << ":" << element->getName());
}

void FGMacOSXInputDevice::Open() {
  // create device interface
  IOReturn ret;
  SInt32 score;
  IOCFPlugInInterface **plugin;
  SG_LOG(SG_INPUT, SG_INFO, "Opening HID : " << GetName());

  ret = IOCreatePlugInInterfaceForService(device,
					  kIOHIDDeviceUserClientTypeID,
					  kIOCFPlugInInterfaceID,
					  &plugin, &score);
	
  if (ret != kIOReturnSuccess) {
    SG_LOG(SG_INPUT, SG_ALERT, "Error creating a plugin for HID : " << GetName());
    throw exception();
    return;
  }

  HRESULT result = (*plugin)->QueryInterface(plugin, 
					     CFUUIDGetUUIDBytes(kIOHIDDeviceInterfaceID), 
					     (LPVOID*)&devInterface );
	
  if (result != S_OK)
    SG_LOG(SG_INPUT, SG_ALERT, "Failed Querying HID plugin interface: " << GetName());

  (*plugin)->Release(plugin); // don't leak a ref
  if (devInterface == NULL) {
    return;
    throw exception();
  }
	
  // store the interface in this instance
  ret = (*devInterface)->open(devInterface, 0);
  if (ret != kIOReturnSuccess) {
    SG_LOG(SG_INPUT, SG_ALERT, "Error opening device interface: " << GetName());
    throw exception();
    return;
  }
  CFDictionaryRef props = getProperties();
		
  // recursively enumerate all the bits (buttons, axes, hats, ...)
  CFTypeRef topLevelElement = 
    CFDictionaryGetValue (props, CFSTR(kIOHIDElementKey));
  HIDElementFactory::create(topLevelElement, this);
  CFRelease(props);
}

void FGMacOSXInputDevice::Close() {
  SG_LOG(SG_INPUT, SG_INFO, "Closing HID: " << GetName());
  if (devInterface) {
    (*devInterface)->close(devInterface);
  }

  map<string, HIDElement *>::iterator it;
  for (it = elements.begin(); it != elements.end(); it++) {
    if ((*it).second)
      delete (*it).second;
  }
  elements.clear();
}

void FGMacOSXInputDevice::update(double dt)
{
  map<string, HIDElement *>::iterator it;
  for (it = elements.begin(); it != elements.end(); it++) {
    (*it).second->readStatus(devInterface);
    if ((*it).second->isUpdated()) {
      int modifiers = fgGetKeyModifiers();
      (*it).second->generateEvent(this, dt, modifiers);
    }
  }
}

//
// FGMacOSXEventInput implementation
//
FGMacOSXEventInput *FGMacOSXEventInput::_instance=NULL;
FGMacOSXEventInput &FGMacOSXEventInput::instance()
{
  if (!FGMacOSXEventInput::_instance) {
    SG_LOG(SG_INPUT, SG_ALERT, "FGMacOSXEventInput is not created but its instance is referred.");
  }
  return *FGMacOSXEventInput::_instance;
}

FGMacOSXEventInput::~FGMacOSXEventInput() {
  /*
  // inputDevice will be deleted in FGEventInput
  map<io_object_t, FGMacOSXInputDevice *>::iterator deviceIterator;
  for (deviceIterator = inputDevices.begin(); deviceIterator != inputDevices.end(); deviceIterator++) {
  FGMacOSXInputDevice *inputDevice = (*deviceIterator).second;
  //    inputDevice->Close();
  //    delete inputDevice;
  }
  */
  deviceIndices.clear();
}

void FGMacOSXEventInput::init()
{
  IOReturn ret;
  SG_LOG(SG_INPUT, SG_INFO, "initializing FGMacOSXEventInput");

  // We want all HID devices for matching
  CFMutableDictionaryRef matchingDictionary = IOServiceMatching(kIOHIDDeviceKey);

  // Needs to retain machingDict since IOServiceAddMatchingNotification consumes one reference.
  matchingDictionary = (CFMutableDictionaryRef) CFRetain(matchingDictionary);
  matchingDictionary = (CFMutableDictionaryRef) CFRetain(matchingDictionary);

  notifyPort = IONotificationPortCreate(kIOMasterPortDefault); 
  runLoopSource = IONotificationPortGetRunLoopSource(notifyPort);
  CFRunLoopAddSource(CFRunLoopGetCurrent(), runLoopSource, kCFRunLoopDefaultMode);
  ret = IOServiceAddMatchingNotification(notifyPort, kIOFirstMatchNotification, 
                                         matchingDictionary, FGMacOSXEventInput::deviceAttached, this, &addedIterator);
  ret = IOServiceAddMatchingNotification(notifyPort, kIOTerminatedNotification, 
                                         matchingDictionary, FGMacOSXEventInput::deviceDetached, this, &removedIterator);

  // prepare for notification by calling these callback funcs to remove existing HID device iterators
  FGMacOSXEventInput::deviceAttached(NULL, addedIterator);
  FGMacOSXEventInput::deviceDetached(NULL, removedIterator);
}


void FGMacOSXEventInput::attachDevice(io_iterator_t iterator)
{
  io_object_t device;

  while ((device = IOIteratorNext(iterator))) {
    FGMacOSXInputDevice *inputDevice = new FGMacOSXInputDevice(device);
    if (inputDevice) {
      SG_LOG(SG_INPUT, SG_INFO, "HID Device Atached: " << inputDevice->GetName());
      unsigned index = AddDevice(inputDevice);
      // Needs to check if AddDevice closed the device due to lack to config file
      if (index != FGEventInput::INVALID_DEVICE_INDEX) {
        deviceIndices[device] = index;
      }
    }
    IOObjectRelease(device);
  }
}


void FGMacOSXEventInput::detachDevice(io_iterator_t iterator)
{
  io_object_t device;

  while ((device = IOIteratorNext(iterator))) {
    unsigned index = deviceIndices[device];
    if (index != FGEventInput::INVALID_DEVICE_INDEX) {
      FGMacOSXInputDevice *inputDevice = (FGMacOSXInputDevice *)input_devices[index];
      SG_LOG(SG_INPUT, SG_INFO, "HID Device Detached: " << inputDevice->GetName());
      RemoveDevice(index);
      deviceIndices.erase(device); 
    } else {
      SG_LOG(SG_INPUT, SG_INFO, "Device ID unmatched: " << (int)device << " No HID deivce is detached since it is not supported by FG.");
    }
    IOObjectRelease(device);
  }
}

void FGMacOSXEventInput::update(double dt)
{
  FGEventInput::update(dt);

  map<int, FGInputDevice*>::const_iterator it;
  for (it = input_devices.begin(); it != input_devices.end(); it++) {
    if ((*it).second) {
      FGMacOSXInputDevice *inputDevice = (FGMacOSXInputDevice *)((*it).second);
      inputDevice->update(dt);
    }
  }
}