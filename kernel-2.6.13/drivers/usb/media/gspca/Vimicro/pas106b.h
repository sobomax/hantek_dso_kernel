
/****************************************************************************
#	PAS106B library                                                     #
# 	Copyright (C) 2005 Thomas Kaiser thomas@kaiser.linux-site.net       #
#                                                                           #
# This program is free software; you can redistribute it and/or modify      #
# it under the terms of the GNU General Public License as published by      #
# the Free Software Foundation; either version 2 of the License, or         #
# (at your option) any later version.                                       #
#                                                                           #
# This program is distributed in the hope that it will be useful,           #
# but WITHOUT ANY WARRANTY; without even the implied warranty of            #
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             #
# GNU General Public License for more details.                              #
#                                                                           #
# You should have received a copy of the GNU General Public License         #
# along with this program; if not, write to the Free Software               #
# Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA #
#                                                                           #
****************************************************************************/

static __u16 pas106b_com_data[][3] = {
/* 352x288 */
/* Sream and Sensor specific */
    {0xA1, 0x01, 0x0010},	//CMOSSensorSelect
/* System */
    {0xA0, 0x01, 0x0000},	//SystemControl
    {0xA0, 0x01, 0x0000},	//SystemControl
/* Picture size */
    {0xA0, 0x00, 0x0002},	//ClockSelect
    {0xA0, 0x03, 0x003a},
    {0xA0, 0x0c, 0x003b},
    {0xa0, 0x04, 0x0038},
        {0, 0, 0}
    };
    
static __u16 pas106b_start_data[][3] = {
/* 176x144 */
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Sream and Sensor specific */
    {0xA0, 0x0F, 0x0010},	//CMOSSensorSelect
/* Picture size */
    {0xA0, 0x00, 0x0003},	//FrameWidthHigh 00
    {0xA0, 0xB0, 0x0004},	//FrameWidthLow B0
    {0xA0, 0x00, 0x0005},	//FrameHeightHigh 00
    {0xA0, 0x90, 0x0006},	//FrameHightLow 90
/* System */
    {0xA0, 0x01, 0x0001},	//SystemOperating
/* Sream and Sensor specific */
    {0xA0, 0x03, 0x0012},	//VideoControlFunction
    {0xA0, 0x01, 0x0012},	//VideoControlFunction
/* Sensor Interface */
    {0xA0, 0x08, 0x008D},	//Compatibily Mode
/* Window inside sensor array */
    {0xA0, 0x03, 0x009A},	//WinXStartLow
    {0xA0, 0x00, 0x011A},	//FirstYLow
    {0xA0, 0x03, 0x011C},	//FirstxLow
    {0xA0, 0x28, 0x009C},	//WinHeightLow
    {0xA0, 0x68, 0x009E},	//WinWidthLow
/* Init the sensor */
    {0xA0, 0x02, 0x0092},	//write register 0x02 to sensor (i2c)
    {0xA0, 0x04, 0x0093},	//Value 0x04
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x08, 0x0092},	//write register 0x08 to sensor (i2c)
    {0xA0, 0x00, 0x0093},	//Value 0x00
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x09, 0x0092},	//write register 0x09 to sensor (i2c)
    {0xA0, 0x05, 0x0093},	//Value 0x05
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0A, 0x0092},	//write register 0x0A to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0B, 0x0092},	//write register 0x0B to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0C, 0x0092},	//write register 0x0C to sensor (i2c)
    {0xA0, 0x05, 0x0093},	//Value 0x05
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0D, 0x0092},	//write register 0x0D to sensor (i2c)
    {0xA0, 0x00, 0x0093},	//Value 0x00
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0E, 0x0092},	//write register 0x0E to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x14, 0x0092},	//write register 0x14 to sensor (i2c)
    {0xA0, 0x81, 0x0093},	//Value 0x81
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    
/* Other registors */
    {0xA0, 0x37, 0x0101},	//SensorCorrection
/* Frame retreiving */
    {0xA0, 0x00, 0x0019},	//AutoAdjustFPS
/* Gains */
    {0xA0, 0xa0, 0x01A8},	//DigitalGain
/* Unknown */
    {0xA0, 0x00, 0x01Ad},
/* Sharpness */
    {0xA0, 0x03, 0x01C5},	//SharpnessMode
    {0xA0, 0x13, 0x01CB},	//Sharpness05
/* Other registors */
    {0xA0, 0x0D, 0x0100},	//OperationMode
/* Auto exposure and white balance */
    {0xA0, 0x06, 0x0189},	// AWBStatus
/*Dead pixels */
    {0xA0, 0x08, 0x0250},	//DeadPixelsMode
/* EEPROM */
    {0xA0, 0x08, 0x0301},	//EEPROMAccess
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Unknown */
    {0xA0, 0x08, 0x01C6},
/* Sharpness */
    {0xA0, 0x0F, 0x01CB},	//Sharpness05
/* Other registers */
    {0xA0, 0x0D, 0x0100},	//OperationMode
/* Auto exposure and white balance */
    {0xA0, 0x06, 0x0189},	// AWBStatus
/*Dead pixels */
    {0xA0, 0x08, 0x0250},	//DeadPixelsMode
/* EEPROM */
    {0xA0, 0x08, 0x0301},	//EEPROMAccess
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Sharpness */
    {0xA0, 0x08, 0x01C6},	//Sharpness00
    {0xA0, 0x0F, 0x01CB},	//Sharpness05

/* Color matrix */
    {0xA0, 0x58, 0x010A},
    {0xA0, 0xF4, 0x010B},
    {0xA0, 0xF4, 0x010C},
    {0xA0, 0xF4, 0x010D},
    {0xA0, 0x58, 0x010E},
    {0xA0, 0xF4, 0x010F},
    {0xA0, 0xF4, 0x0110},
    {0xA0, 0xF4, 0x0111},
    {0xA0, 0x58, 0x0112},
/* Auto correction */
    {0xA0, 0x03, 0x0181},	//WinXstart
    {0xA0, 0x08, 0x0182},	//WinXWidth
    {0xA0, 0x16, 0x0183},	//WinXCenter
    {0xA0, 0x03, 0x0184},	//WinYStart
    {0xA0, 0x05, 0x0185},	//WinYWidth
    {0xA0, 0x14, 0x0186},	//WinYCenter
    {0xA0, 0x00, 0x0180},	//AutoCorrectEnable
    
/* Auto exposure and white balance */
    {0xA0, 0x00, 0x0190},	//ExposureLimitHigh
    {0xA0, 0x03, 0x0191},	//ExposureLimitMid
    {0xA0, 0xB1, 0x0192},	//ExposureLimitLow
    {0xA0, 0x00, 0x0195},	//AntiFlickerHigh
    {0xA0, 0x00, 0x0196},	//AntiFlickerLow
    {0xA0, 0x87, 0x0197},	//AntiFlickerLow
    {0xA0, 0x0C, 0x018C},	//AEBFreeze
    {0xA0, 0x18, 0x018F},	//AEBUnfreeze
/* sensor on */
{0xA0, 0x07, 0x0092},	//write register 0x07 to sensor (i2c)
    {0xA0, 0xB1, 0x0093},	//Value 0xB1
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x05, 0x0092},	//write register 0x05 to sensor (i2c)
    {0xA0, 0x03, 0x0093},	//Value 0x03
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x04, 0x0092},	//write register 0x04 to sensor (i2c)
    {0xA0, 0x01, 0x0093},	//Value 0x01
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x03, 0x0092},	//write register 0x03 to sensor (i2c)
    {0xA0, 0x3B, 0x0093},	//Value 0x3B
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
/* Gains */
    {0xA0, 0x20, 0x01A9},	//DigitalLimitDiff
    {0xA0, 0x26, 0x01AA},	//DigitalGainStep
    {0xA0, 0xA0, 0x011D},	//GlobalGain
    {0xA0, 0x60, 0x011D},	//GlobalGain
/* Auto correction */
    {0xA0, 0x40, 0x0180},	//AutoCorrectEnable
    {0xa1, 0x01, 0x0180},	//AutoCorrectEnable
    {0xA0, 0x42, 0x0180},	//AutoCorrectEnable
/* Gains */
    {0xA0, 0x40, 0x0116},	//RGain
    {0xA0, 0x40, 0x0117},	//GGain
    {0xA0, 0x40, 0x0118},	//BGain
    {0, 0, 0}
};

static __u16 pas106b_scale_data[][3] = {
/* 352x288 */
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Sream and Sensor specific */
    {0xA0, 0x0F, 0x0010},	//CMOSSensorSelect
/* Picture size */
    {0xA0, 0x01, 0x0003},	//FrameWidthHigh
    {0xA0, 0x60, 0x0004},	//FrameWidthLow
    {0xA0, 0x01, 0x0005},	//FrameHeightHigh
    {0xA0, 0x20, 0x0006},	//FrameHightLow
/* System */
    {0xA0, 0x01, 0x0001},	//SystemOperating
/* Sream and Sensor specific */
    {0xA0, 0x03, 0x0012},	//VideoControlFunction
    {0xA0, 0x01, 0x0012},	//VideoControlFunction
/* Sensor Interface */
    {0xA0, 0x08, 0x008D},	//Compatibily Mode
/* Window inside sensor array */
    {0xA0, 0x03, 0x009A},	//WinXStartLow
    {0xA0, 0x00, 0x011A},	//FirstYLow
    {0xA0, 0x03, 0x011C},	//FirstxLow
    {0xA0, 0x28, 0x009C},	//WinHeightLow
    {0xA0, 0x68, 0x009E},	//WinWidthLow
/* Init the sensor */
    {0xA0, 0x02, 0x0092},	//write register 0x02 to sensor (i2c)
    {0xA0, 0x04, 0x0093},	//Value 0x04
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x08, 0x0092},	//write register 0x08 to sensor (i2c)
    {0xA0, 0x00, 0x0093},	//Value 0x00
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x09, 0x0092},	//write register 0x09 to sensor (i2c)
    {0xA0, 0x05, 0x0093},	//Value 0x05
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0A, 0x0092},	//write register 0x0A to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0B, 0x0092},	//write register 0x0B to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0C, 0x0092},	//write register 0x0C to sensor (i2c)
    {0xA0, 0x05, 0x0093},	//Value 0x05
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0D, 0x0092},	//write register 0x0D to sensor (i2c)
    {0xA0, 0x00, 0x0093},	//Value 0x00
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x0E, 0x0092},	//write register 0x0E to sensor (i2c)
    {0xA0, 0x02, 0x0093},	//Value 0x02
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x14, 0x0092},	//write register 0x14 to sensor (i2c)
    {0xA0, 0x81, 0x0093},	//Value 0x81
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
   
/* Other registors */
    {0xA0, 0x37, 0x0101},	//SensorCorrection
/* Frame retreiving */
    {0xA0, 0x00, 0x0019},	//AutoAdjustFPS
/* Gains */
    {0xA0, 0xa0, 0x01A8},	//DigitalGain
/* Unknown */
    {0xA0, 0x00, 0x01Ad},
/* Sharpness */
    {0xA0, 0x03, 0x01C5},	//SharpnessMode
    {0xA0, 0x13, 0x01CB},	//Sharpness05
/* Other registors */
    {0xA0, 0x0D, 0x0100},	//OperationMode
/* Auto exposure and white balance */
    {0xA0, 0x06, 0x0189},	// AWBStatus
    {0xA0, 0x80, 0x018D},	// ?????????
/*Dead pixels */
    {0xA0, 0x08, 0x0250},	//DeadPixelsMode
/* EEPROM */
    {0xA0, 0x08, 0x0301},	//EEPROMAccess
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Unknown */
    {0xA0, 0x08, 0x01C6},
/* Sharpness */
    {0xA0, 0x0F, 0x01CB},	//Sharpness05
/* Other registers */
    {0xA0, 0x0D, 0x0100},	//OperationMode
/* Auto exposure and white balance */
    {0xA0, 0x06, 0x0189},	// AWBStatus
/*Dead pixels */
    {0xA0, 0x08, 0x0250},	//DeadPixelsMode
/* EEPROM */
    {0xA0, 0x08, 0x0301},	//EEPROMAccess
/* JPEG control */
    {0xA0, 0x03, 0x0008},	//ClockSetting
/* Sharpness */
    {0xA0, 0x08, 0x01C6},	//Sharpness00
    {0xA0, 0x0F, 0x01CB},	//Sharpness05

/* Color matrix */
    {0xA0, 0x58, 0x010A},
    {0xA0, 0xF4, 0x010B},
    {0xA0, 0xF4, 0x010C},
    {0xA0, 0xF4, 0x010D},
    {0xA0, 0x58, 0x010E},
    {0xA0, 0xF4, 0x010F},
    {0xA0, 0xF4, 0x0110},
    {0xA0, 0xF4, 0x0111},
    {0xA0, 0x58, 0x0112},
/* Auto correction */
    {0xA0, 0x03, 0x0181},	//WinXstart
    {0xA0, 0x08, 0x0182},	//WinXWidth
    {0xA0, 0x16, 0x0183},	//WinXCenter
    {0xA0, 0x03, 0x0184},	//WinYStart
    {0xA0, 0x05, 0x0185},	//WinYWidth
    {0xA0, 0x14, 0x0186},	//WinYCenter
    {0xA0, 0x00, 0x0180},	//AutoCorrectEnable
    
/* Auto exposure and white balance */
    {0xA0, 0x00, 0x0190},	//ExposureLimitHigh 0
    {0xA0, 0x03, 0x0191},	//ExposureLimitMid
    {0xA0, 0xb1, 0x0192},	//ExposureLimitLow 0xb1
    
    {0xA0, 0x00, 0x0195},	//AntiFlickerHigh 0x00
    {0xA0, 0x00, 0x0196},	//AntiFlickerLow 0x00
    {0xA0, 0x87, 0x0197},	//AntiFlickerLow 0x87
    
    {0xA0, 0x10, 0x018C},	//AEBFreeze 0x10 0x0c
    {0xA0, 0x20, 0x018F},	//AEBUnfreeze 0x30 0x18
    /* sensor on */
    {0xA0, 0x07, 0x0092},	//write register 0x07 to sensor (i2c)
    {0xA0, 0xB1, 0x0093},	//Value 0xB1
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x05, 0x0092},	//write register 0x05 to sensor (i2c)
    {0xA0, 0x03, 0x0093},	//Value 0x03
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x04, 0x0092},	//write register 0x04 to sensor (i2c)
    {0xA0, 0x01, 0x0093},	//Value 0x01
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
    {0xA0, 0x03, 0x0092},	//write register 0x03 to sensor (i2c)
    {0xA0, 0x3b, 0x0093},	//Value 0x3B
    {0xA0, 0x00, 0x0094},
    {0xA0, 0x01, 0x0090},
    {0xA1, 0x01, 0x0091},	//end write i2c
 
/* Gains */
    {0xA0, 0x20, 0x01A9},	//DigitalLimitDiff
    {0xA0, 0x26, 0x01AA},	//DigitalGainStep
    {0xA0, 0xA0, 0x011D},	//GlobalGain
    {0xA0, 0x60, 0x011D},	//GlobalGain
/* Auto correction */
    {0xA0, 0x40, 0x0180},	//AutoCorrectEnable
    {0xa1, 0x01, 0x0180},	//AutoCorrectEnable
    {0xA0, 0x42, 0x0180},	//AutoCorrectEnable
/* Gains */
    {0xA0, 0x40, 0x0116},	//RGain
    {0xA0, 0x40, 0x0117},	//GGain
    {0xA0, 0x40, 0x0118},	//BGain
    
    {0xA0, 0x00, 0x0007},	//AutoCorrectEnable
    {0xA0, 0xFF, 0x0018},	//Frame adjust
    {0, 0, 0}
};
static __u16 pas106b_50HZ[][3] = {
    {0xa0, 0x0000, 0x0190}, //01,90,00,cc
    {0xa0, 0x0006, 0x0191}, //01,91,06,cc
    {0xa0, 0x0054, 0x0192}, //01,92,54,cc
    {0xa0, 0x0000, 0x0195}, //01,95,00,cc
    {0xa0, 0x0000, 0x0196}, //01,96,00,cc
    {0xa0, 0x0087, 0x0197}, //01,97,87,cc
    {0xa0, 0x0010, 0x018c}, //01,8c,10,cc
    {0xa0, 0x0030, 0x018f}, //01,8f,30,cc
    {0xa0, 0x0003, 0x0092}, //00,03,21,aa
    {0xa0, 0x0021, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0004, 0x0092}, //00,04,0c,aa
    {0xa0, 0x000c, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0005, 0x0092}, //00,05,02,aa
    {0xa0, 0x0002, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0007, 0x0092}, //00,07,1c,aa
    {0xa0, 0x001c, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0004, 0x01a9}, //01,a9,04,cc
/*******************/
{0, 0, 0}
};
static __u16 pas106b_60HZ[][3] = {
    {0xa0, 0x0000, 0x0190}, //01,90,00,cc
    {0xa0, 0x0006, 0x0191}, //01,91,06,cc
    {0xa0, 0x002e, 0x0192}, //01,92,2e,cc
    {0xa0, 0x0000, 0x0195}, //01,95,00,cc
    {0xa0, 0x0000, 0x0196}, //01,96,00,cc
    {0xa0, 0x0071, 0x0197}, //01,97,71,cc
    {0xa0, 0x0010, 0x018c}, //01,8c,10,cc
    {0xa0, 0x0030, 0x018f}, //01,8f,30,cc
    {0xa0, 0x0003, 0x0092}, //00,03,1c,aa
    {0xa0, 0x001c, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0004, 0x0092}, //00,04,04,aa
    {0xa0, 0x0004, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0005, 0x0092}, //00,05,01,aa
    {0xa0, 0x0001, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0007, 0x0092}, //00,07,c4,aa
    {0xa0, 0x00c4, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0004, 0x01a9}, //01,a9,04,cc
/*******************/
{0, 0, 0}
};
static __u16 pas106b_NoFliker[][3] = {
    {0xa0, 0x0000, 0x0190}, //01,90,00,cc
    {0xa0, 0x0006, 0x0191}, //01,91,06,cc
    {0xa0, 0x0050, 0x0192}, //01,92,50,cc
    {0xa0, 0x0000, 0x0195}, //01,95,00,cc
    {0xa0, 0x0000, 0x0196}, //01,96,00,cc
    {0xa0, 0x0010, 0x0197}, //01,97,10,cc
    {0xa0, 0x0010, 0x018c}, //01,8c,10,cc
    {0xa0, 0x0020, 0x018f}, //01,8f,20,cc
    {0xa0, 0x0003, 0x0092}, //00,03,13,aa
    {0xa0, 0x0013, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0004, 0x0092}, //00,04,00,aa
    {0xa0, 0x0000, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0005, 0x0092}, //00,05,01,aa
    {0xa0, 0x0001, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0007, 0x0092}, //00,07,30,aa
    {0xa0, 0x0030, 0x0093}, //
    {0xa0, 0x0000, 0x0094}, //
    {0xa0, 0x0001, 0x0090}, //
    {0xa1, 0x0001, 0x0091}, //
/**********************/
    {0xa0, 0x0000, 0x01a9}, //01,a9,00,cc
/*******************/
{0, 0, 0}
};