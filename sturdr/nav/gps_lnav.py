"""**gps_lnav.py**

======  ============================================================================================
file    sturdr/nav/gps_lnav.py
brief   Implementation of channel.py for GPS L1 C/A signals.
date    October 2024
refs    1. "IS-GPS-200N", 2022
        2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
======  ============================================================================================
"""

# TODO: switch to GetDataBit(s)

import numpy as np
from sturdr.nav.ephemeris import KeplerianEphemeris
from sturdr.utils.enums import GnssSystem, GnssSignalTypes
from sturdr.utils.binary_ops import RotL, TwosCompliment, GetDataBit, GetDataBits, SetDataBitTo
from sturdr.utils.constants import PI

# ===== GPS LNAV Data Parser ===================================================================== #

class GpsLnavParser(KeplerianEphemeris):
    __slots__ = 'subframe', 'bit_count', 'word_count', 'TOW', 'week', 'signalID', 'subframe1', \
                'subframe2', 'subframe3'
    subframe    : np.ndarray
    bit_count   : int
    word_count  : int
    TOW         : np.double
    week        : int
    subframe1   : bool
    subframe2   : bool
    subframe3   : bool
    
    def __init__(self):
        KeplerianEphemeris.__init__()
        self.subframe   = np.zeros(10, dtype=np.uint32)
        self.bit_count  = 0
        self.word_count = 0
        self.subframe1  = False
        self.subframe2  = False
        self.subframe3  = False
        return
    
    def NextBit(self, bit: bool):
        self.subframe[self.word_count] = SetDataBitTo(
            self.subframe[self.word_count], self.bit_count, bit)
        self.bit_count += 1
        
        if self.bit_count > 31:
            # check if subframe is now complete
            old_count = self.word_count
            self.word_count +=1
            if self.word_count > 9:
                self.ParseSubframe()
                self.word_count = 0
            
            # Bit -2 of next word is bit 31 or current word
            self.subframe[self.word_count] = SetDataBitTo(
                self.subframe[old_count], 0, GetDataBit(self.subframe[old_count], 30))
            
            # Bit -1 of next word is bit 32 or current word
            self.subframe[self.word_count] = SetDataBitTo(
                self.subframe[old_count], 1, GetDataBit(self.subframe[old_count], 31))
            
            self.bit_count = 2
            
        return
    
    def ParseSubframe(self):
        # data bits ordered [-2 -1 0 ... 30]
        for gpsword in self.subframe:
            # invert data bits according to D30star
            if (gpsword & 0x40000000):
                gpsword ^= 0x3FFFFFC0
            
            # parity check current word
            if not self.ParityCheck(gpsword):
                raise ValueError("Invalid parity check!")
            
        # get subframe id
        subframe_id = np.uint8((self.subframe[1] & 0x00000700) >> 8)  # bits 20-22
        
        # parse subframe
        match subframe_id:
            case 1:
                self.LoadSubframe1()
                self.subframe1 = True
            case 2:
                self.LoadSubframe2()
                self.subframe2 = True
            case 3:
                self.LoadSubframe3()
                self.subframe3 = True
            case 4:
                self.LoadSubframe4()
            case 5:
                self.LoadSubframe5()
            case _:
                raise ValueError("Invalid parity check!")
                
        return
        
    def ParityCheck(self, gpsword: np.uint32):
        # XOR as many bits in parallel as possible, IS-GPS-200N
        d1 = np.uint32(gpsword & 0xFBFFBF00)
        d2 = np.uint32(RotL(gpsword, 1) & 0x07FFBF01)
        d3 = np.uint32(RotL(gpsword, 2) & 0xFC0F8100)
        d4 = np.uint32(RotL(gpsword, 3) & 0xF81FFE02)
        d5 = np.uint32(RotL(gpsword, 4) & 0xFC00000E)
        d6 = np.uint32(RotL(gpsword, 5) & 0x07F00001)
        d7 = np.uint32(RotL(gpsword, 6) & 0x00003000)
        t = d1 ^ d2 ^ d3 ^ d4 ^ d5 ^ d6 ^ d7

        # XOR the 5 6-bit fields together to produce the 6-bit parity
        parity = np.uint32((t ^ RotL(t, 6) ^ RotL(t, 12) ^ RotL(t, 18) ^ RotL(t, 24)) & 0x0000003F)
        if (parity == (gpsword & 0x0000003F)):
            return True
        return False
        
    def LoadPreamble(self):
        # word 1
        # tlm_message = np.uint16((self.subframe[0] & 0x003FFF00) >> 8)     # bits 9-22
        # integrity_status_flag = bool((self.subframe[0] & 0x00000080) >> 7)# bit 23
        
        # word 2
        self.TOW = 6.0 * np.double((self.subframe[1] & 0x3FFFE000) >> 13)   # bits 1-17
        # alert_flag = bool((self.subframe[1] & 0x00001000) >> 12)          # bit 18
        # anti_spoof_flag = bool((self.subframe[1] & 0x00000800) >> 11)     # bit 19
        return
    
    def LoadSubframe1(self):
        # word 1-2
        self.LoadPreamble()
        
        # word 3
        self.week = np.uint16((self.subframe[2] & 0x3FF00000) >> 20)    # bits 1-10
        # l2_flag = np.uint8((self.subframe[2] & 0x000C0000) >> 18)       # bits 11-12
        self.ura = np.uint8((self.subframe[2] & 0x0003C000) >> 14)      # bits 13-16
        self.health = np.uint8((self.subframe[2] & 0x00003F00) >> 8)    # bits 17-22
        
        # word 7
        self.tgd = TwosCompliment((self.subframe[6] & 0x00003FC0) >> 6, 8) * (2**-31)   # bits 9-24
        
        # word 8
        tmp1 = (self.subframe[2] & 0x000000C0) >> 6                         # bits 23-24 (word 3)
        tmp2 = (self.subframe[7] & 0x3FC00000) >> 22                        # bits 1-8
        self.iodc = np.double((tmp1 << 8) | tmp2)                           #
        self.toc = np.double((self.subframe[7] & 0x003FFFC0) >> 6) * (2**4) # bits 9-24
        
        # word 9
        self.af2 = TwosCompliment((self.subframe[8] & 0x000000C0) >> 22, 8) * (2**-55) # bits 1-8
        self.af1 = TwosCompliment((self.subframe[8] & 0x003FFFC0) >> 6, 16) * (2**4)   # bits 9-24
        
        # word 10
        self.af0 = TwosCompliment((self.subframe[9] & 0x3FFFFF00) >> 8, 22) * (2**-31) # bits 1-22
        
        return
    
    def LoadSubframe2(self):
        # word 1-2
        self.LoadPreamble()
        
        # word 3
        self.iode = np.double((self.subframe[2] & 0x000000C0) >> 22)                    # bits 1-8
        self.crs = TwosCompliment((self.subframe[2] & 0x003FFFC0) >> 6, 16) * (2**-5)   # bits 9-24
        
        # word 4 and 5
        self.deltan = TwosCompliment((self.subframe[3] & 0x3FFFC000) >> 14, 16) * (PI * 2**-43) # bits 1-16
        tmp1 = (self.subframe[3] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[3] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.m0 = TwosCompliment((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 6 and 7
        self.cuc = TwosCompliment((self.subframe[5] & 0x3FFFC000) >> 14, 16) * (2**-29) # bits 1-16
        tmp1 = (self.subframe[5] & 0x00003FC0) >> 6                                     # bits 17-24
        tmp2 = (self.subframe[6] & 0x3FFFFFC0) >> 6                                     # bits 1-24
        self.e = np.double((tmp1 << 24) | tmp2) * (2**-33)
        
        # word 8 and 9
        self.cus = TwosCompliment((self.subframe[7] & 0x3FFFC000) >> 14, 16) * (2**-29) # bits 1-16
        tmp1 = (self.subframe[7] & 0x00003FC0) >> 6                                     # bits 17-24
        tmp2 = (self.subframe[8] & 0x3FFFFFC0) >> 6                                     # bits 1-24
        self.sqrtA = np.double((tmp1 << 24) | tmp2) * (2**-19)
        
        # word 10
        self.toe = np.double((self.subframe[9] & 0x3FFFC000) >> 14) * (2**4)    # bits 1-16
        # fit_interval_alert_flag = bool((self.subframe[9] & 0x00002000) >> 13)   # bit 17
        
        return
    
    def LoadSubframe3(self):
        # word 1-2
        self.LoadPreamble()
        
        # word 3 and 4
        self.cic = TwosCompliment((self.subframe[2] & 0x3FFFC000) >> 14, 16) * (2**-29) # bits 1-16
        tmp1 = (self.subframe[2] & 0x00003FC0) >> 6                                     # bits 17-24
        tmp2 = (self.subframe[3] & 0x3FFFFFC0) >> 6                                     # bits 1-24
        self.omega0 = TwosCompliment((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 5 and 6
        self.cis = TwosCompliment((self.subframe[4] & 0x3FFFC000) >> 14, 16) * (2**-29) # bits 1-16
        tmp1 = (self.subframe[4] & 0x00003FC0) >> 6                                     # bits 17-24
        tmp2 = (self.subframe[5] & 0x3FFFFFC0) >> 6                                     # bits 1-24
        self.i0 = TwosCompliment((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 7 and 8
        self.crc = TwosCompliment((self.subframe[6] & 0x3FFFC000) >> 14, 16) * (2**-5)  # bits 1-16
        tmp1 = (self.subframe[6] & 0x00003FC0) >> 6                                     # bits 17-24
        tmp2 = (self.subframe[7] & 0x3FFFFFC0) >> 6                                     # bits 1-24
        self.omega = TwosCompliment((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 9
        self.omegaDot = TwosCompliment((self.subframe[8] & 0x3FFFFFC0) >> 6, 24) * (PI * 2**-43)# bits 1-24
        
        # word 10
        self.iode = TwosCompliment((self.subframe[9] & 0x000000C0) >> 22, 8)                # bits 1-8
        self.iDot = TwosCompliment((self.subframe[8] & 0x003FFF00) >> 8, 14) * (PI * 2**-43)# bits 9-22
        
        return
    
    def LoadSubframe4(self):
        # word 1-2
        self.LoadPreamble()
        return
    
    def LoadSubframe5(self):
        # word 1-2
        self.LoadPreamble()
        return