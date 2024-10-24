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

# TODO: clean up preamble synchronization process
# TODO: track multiple preamble detections at ones to not skip 6 seconds before next attempt
#       -> one detected begin placing bits into word array (subframe)
#       -> if detection is invalid, shift bits forward in the word array until bits 2-9 match the preamble again
#           -> be mindful of uint32 vs 30 bit words
#       -> repeat until preamble is detected twice, 300 bits apart

import numpy as np
from sturdr.nav.ephemeris import Ephemerides
from sturdr.utils.enums import GnssSystem, GnssSignalTypes
from sturdr.utils.binary_ops import *
from sturdr.utils.constants import PI, LNAV_PREAMBLE_BITS, LNAV_INV_PREAMBLE_BITS

# ===== GPS LNAV Data Parser ===================================================================== #

GPS_D25 = np.asarray([2,3,4,6,7,11,12,13,14,15,18,19,21,24], dtype = np.uint8)    # [1,2,3,5,6,10,11,12,13,14,17,18,20,23]
GPS_D26 = np.asarray([3,4,5,7,8,12,13,14,15,16,19,20,22,25], dtype = np.uint8)    # [2,3,4,6,7,11,12,13,14,15,18,19,21,24]
GPS_D27 = np.asarray([2,4,5,6,8,9,13,14,15,16,17,20,21,23], dtype = np.uint8)     # [1,3,4,5,7,8,12,13,14,15,16,19,20,22]
GPS_D28 = np.asarray([3,5,6,7,9,10,14,15,16,17,18,21,22,24], dtype = np.uint8)    # [2,4,5,6,8,9,13,14,15,16,17,20,21,23]
GPS_D29 = np.asarray([2,4,6,7,8,10,11,15,16,17,18,19,22,23,25], dtype = np.uint8) # [1,3,5,6,7,9,10,14,15,16,17,18,21,22,24]
GPS_D30 = np.asarray([4,6,7,9,10,11,12,14,16,20,23,24,25], dtype = np.uint8)      # [3,5,6,8,9,10,11,13,15,19,22,23,24]


class GpsLnavParser:
    __slots__ = 'subframe', 'bit_count', 'word_count', 'TOW', 'week', 'signalID', 'subframe1', \
                'subframe2', 'subframe3', 'preamble_found', 'bits_since_preamble', 'prev_32_bits', \
                'preamble_detections', 'ephemerides'
    subframe            : np.ndarray[np.uint32]
    bit_count           : np.uint16
    word_count          : np.uint8
    TOW                 : np.double
    week                : np.uint16
    subframe1           : bool
    subframe2           : bool
    subframe3           : bool
    preamble_found      : bool
    bits_since_preamble : np.int16
    prev_32_bits        : np.uint32
    preamble_detections : list[np.int16]
    ephemerides         : Ephemerides
    
    def __init__(self):
        self.subframe            = np.zeros(10, dtype=np.uint32)
        self.bit_count           = 0
        self.word_count          = 0
        self.subframe1           = False
        self.subframe2           = False
        self.subframe3           = False
        self.preamble_found      = False
        self.bits_since_preamble = -1
        self.prev_32_bits        = 0
        self.preamble_detections = []
        self.TOW                 = np.nan
        self.week                = np.nan
        self.ephemerides         = Ephemerides()
        return
    
    def SetID(self, id: str):
        self.ephemerides.id = id
        return
    
    def NextBit(self, bit: bool):
        # data bits ordered [-2 -1 0 ... 29]
        subframe_parsed = False
        
        # shift saved bits left 1 and save next bit (enforce uint32 size)
        self.prev_32_bits = np.uint32((self.prev_32_bits << 1) & 0xFFFFFFFF)
        self.prev_32_bits = ModifyBit(self.prev_32_bits, 31, bit)
        self.bit_count += 1
        
        # ---------------------------------------------------------------------------------------- #
        # if we have not acquired the preamble, we must do that
        if not self.preamble_found:
            # check last 8 bits for the preamble
            test = np.uint8(self.prev_32_bits & 0x000000FF)
            if (test == LNAV_PREAMBLE_BITS) or (test == LNAV_INV_PREAMBLE_BITS):
                # initial correct preamble detection
                if self.bits_since_preamble == -1:
                    # print("preamble detected")
                    self.bit_count = 8
                    self.word_count = 0
                    self.bits_since_preamble = 0
                # second correct preamble detection
                elif self.bits_since_preamble == 300:
                    # print("correct preamble detected!")
                    self.preamble_found = True
                    self.ParseSubframe()
                    self.bit_count = 8
                    self.word_count = 0
                    self.bits_since_preamble = 0
                    subframe_parsed = True
                else:
                    self.preamble_detections.append(self.bits_since_preamble)
                    
            # if we are here, the preamble did not sucessfully sync, reset to the next detection
            if self.bits_since_preamble == 300:
                # print("incorrect preamble detected :(")
                bits_to_shift = self.preamble_detections[0] % 30 + 1
                words_to_shift = self.preamble_detections[0] // 30
                
                # first, shift by full words (30 bits)
                if words_to_shift > 0:
                    self.subframe[:-words_to_shift] = self.subframe[words_to_shift:]
                    self.subframe[-words_to_shift:] = 0
                
                # second, shift by remaining bits
                if bits_to_shift > 0:
                    for i in range(9 - words_to_shift):
                        curr_bits = np.uint32(((self.subframe[i] & 0xFFFFFFFC) << (bits_to_shift-1)) & 0xFFFFFFFF)
                        next_bits = (self.subframe[i+1] >> (31 - bits_to_shift))
                        self.subframe[i] = curr_bits | next_bits
                     
                # update
                self.bits_since_preamble -= self.preamble_detections[0]
                self.bit_count = 31 - bits_to_shift + 8 # +8 to account for preamble
                self.word_count = 9 - words_to_shift
                
                # reset detections and rid of used detection
                self.preamble_detections = [x - self.preamble_detections[0] for x in self.preamble_detections]
                self.preamble_detections.pop(0)
                # print(f"{self.subframe[0]:032b}")
                # print()
            
            # increment bit count if necessary
            if self.bits_since_preamble >= 0:
                self.bits_since_preamble += 1
        # ---------------------------------------------------------------------------------------- #
        
        # if 30 new bits have been parsed, save into gps word
        if self.bit_count == 30:
            self.subframe[self.word_count] = self.prev_32_bits
            # print(f"word_count = {self.word_count:02d}, {self.subframe[self.word_count]:032b}")
            self.bit_count = 0
            self.word_count += 1
        
        # if 10 words have been parsed, reset to 0 and parse subframe
        if self.word_count == 10:
            self.word_count = 0
            if self.preamble_found:
                self.ParseSubframe()
                subframe_parsed = True
        return subframe_parsed
    
    def ParseSubframe(self):
        """
        Attempts to parse subframe

        Raises
        ------
        ValueError
            Invalid parity check!
        ValueError
            Invalid subframe ID!
        """
        # data bits ordered [-2 -1 0 ... 29]
        for i in range(self.subframe.size):
            D29star = CheckBit(self.subframe[i], 0)
            D30star = CheckBit(self.subframe[i], 1)
            
            # check polarity of current word
            if D30star:
                self.subframe[i] ^= 0x3FFFFFC0
            
            # parity check current word
            if not self.ParityCheck(self.subframe[i], D29star, D30star):
                raise ValueError("Invalid parity check!")
            
        # get subframe id (IS-GPS-200N pg. 92)
        subframe_id = np.uint8((self.subframe[1] & 0x00000700) >> 8)  # bits 20-22
        # print(f"subframe_id = {subframe_id}")
        
        # parse subframe
        match subframe_id:
            case 1:
                # print("\nSubframe 1")
                self.LoadSubframe1()
                self.subframe1 = True
            case 2:
                # print("\nSubframe 2")
                self.LoadSubframe2()
                self.subframe2 = True
            case 3:
                # print("\nSubframe 3")
                self.LoadSubframe3()
                self.subframe3 = True
            case 4:
                # print("\nSubframe 4")
                self.LoadSubframe4()
            case 5:
                # print("\nSubframe 5")
                self.LoadSubframe5()
            case _:
                raise ValueError("Invalid subframe ID!")
                
        return
        
    def ParityCheck(self, gpsword: np.uint32, D29star: bool, D30star: bool):
        """
        IS-GPS-200N pg. 139

        Parameters
        ----------
        gpsword : np.uint32
            GPS word to evaluate

        Returns
        -------
        bool
            Parity success or failure
        """
        parity = np.uint32(0)
        parity = ModifyBit(parity, 26, D29star ^ MultiXor(gpsword, GPS_D25)) # D25
        parity = ModifyBit(parity, 27, D30star ^ MultiXor(gpsword, GPS_D26)) # D26
        parity = ModifyBit(parity, 28, D29star ^ MultiXor(gpsword, GPS_D27)) # D27
        parity = ModifyBit(parity, 29, D30star ^ MultiXor(gpsword, GPS_D28)) # D28
        parity = ModifyBit(parity, 30, D30star ^ MultiXor(gpsword, GPS_D29)) # D29
        parity = ModifyBit(parity, 31, D29star ^ MultiXor(gpsword, GPS_D30)) # D30
        
        # check parity
        # print(f"parity = {parity:06b}, gpsword = {(gpsword & 0x0000003F):06b}")
        if (parity == (gpsword & 0x0000003F)):
            return True
        return False
        
    def LoadPreamble(self):
        """
        IS-GPS-200N pg. 92
        """
        # word 1
        # tlm_message = np.uint16((self.subframe[0] & 0x003FFF00) >> 8)     # bits 9-22
        # integrity_status_flag = bool((self.subframe[0] & 0x00000080) >> 7)# bit 23
        
        # print(f"Word 2             = {self.subframe[1]:032b}")
        # print(f"Word 2 TOW         = {(self.subframe[1] & 0x3FFFE000):032b}")
        # print(f"Word 2 TOW shifted = {((self.subframe[1] & 0x3FFFE000) >> 13):032b}")
        
        # word 2
        self.TOW = 6.0 * np.double((self.subframe[1] & 0x3FFFE000) >> 13)   # bits 1-17
        # alert_flag = bool((self.subframe[1] & 0x00001000) >> 12)          # bit 18
        # anti_spoof_flag = bool((self.subframe[1] & 0x00000800) >> 11)     # bit 19
        
        # print(f"TOW = {self.TOW}")
        return
    
    def LoadSubframe1(self):
        """
        IS-GPS-200N pg. 80
        IS-GPS-200N pg. 97
        """
        # word 1-2
        self.LoadPreamble()
        
        # word 3
        self.week = np.uint16((self.subframe[2] & 0x3FF00000) >> 20)                # bits 1-10
        # l2_flag = np.uint8((self.subframe[2] & 0x000C0000) >> 18)                 # bits 11-12
        self.ephemerides.ura = np.uint8((self.subframe[2] & 0x0003C000) >> 14)      # bits 13-16
        self.ephemerides.health = np.uint8((self.subframe[2] & 0x00003F00) >> 8)    # bits 17-22
        
        # word 7
        self.ephemerides.tgd = TwosComp((self.subframe[6] & 0x00003FC0) >> 6, 8) * (2**-31) # bits 17-24
        
        # word 8
        tmp1 = (self.subframe[2] & 0x000000C0) >> 6                                     # bits 23-24 (word 3)
        tmp2 = (self.subframe[7] & 0x3FC00000) >> 22                                    # bits 1-8
        self.ephemerides.iodc = np.double((tmp1 << 8) | tmp2)                           #
        self.ephemerides.toc = np.double((self.subframe[7] & 0x003FFFC0) >> 6) * (2**4) # bits 9-24
        
        # word 9
        self.ephemerides.af2 = TwosComp((self.subframe[8] & 0x000000C0) >> 22, 8) * (2**-55) # bits 1-8
        self.ephemerides.af1 = TwosComp((self.subframe[8] & 0x003FFFC0) >> 6, 16) * (2**-43) # bits 9-24
        
        # word 10
        self.ephemerides.af0 = TwosComp((self.subframe[9] & 0x3FFFFF00) >> 8, 22) * (2**-31) # bits 1-22
        
        # print(f"week = {self.week}")
        # print(f"af0 = {self.af0}")
        # print(f"af1 = {self.af1}")
        # print(f"af2 = {self.af2}")
        # print(f"iodc = {self.iodc}")
        # print(f"toc = {self.toc}")
        # print(f"tgd = {self.tgd}")
        # print(f"ura = {self.ura}")
        # print(f"health = {self.health}")
        
        return
    
    def LoadSubframe2(self):
        """
        IS-GPS-200N pg. 81
        IS-GPS-200N pg. 105
        """
        # word 1-2
        self.LoadPreamble()
        
        # word 3
        self.ephemerides.iode = np.double((self.subframe[2] & 0x3FC00000) >> 22)            # bits 1-8
        self.ephemerides.crs = TwosComp((self.subframe[2] & 0x003FFFC0) >> 6, 16) * (2**-5) # bits 9-24
        
        # word 4 and 5
        self.ephemerides.deltan = TwosComp((self.subframe[3] & 0x3FFFC000) >> 14, 16) * (PI * 2**-43)   # bits 1-16
        tmp1 = (self.subframe[3] & 0x00003FC0) >> 6                                                     # bits 17-24
        tmp2 = (self.subframe[4] & 0x3FFFFFC0) >> 6                                                     # bits 1-24
        self.ephemerides.m0 = TwosComp((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 6 and 7
        self.ephemerides.cuc = TwosComp((self.subframe[5] & 0x3FFFC000) >> 14, 16) * (2**-29)   # bits 1-16
        tmp1 = (self.subframe[5] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[6] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.ephemerides.e = np.double((tmp1 << 24) | tmp2) * (2**-33)
        
        # word 8 and 9
        self.ephemerides.cus = TwosComp((self.subframe[7] & 0x3FFFC000) >> 14, 16) * (2**-29)   # bits 1-16
        tmp1 = (self.subframe[7] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[8] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.ephemerides.sqrtA = np.double((tmp1 << 24) | tmp2) * (2**-19)
        
        # word 10
        self.ephemerides.toe = np.double((self.subframe[9] & 0x3FFFC000) >> 14) * (2**4)    # bits 1-16
        # fit_interval_alert_flag = bool((self.subframe[9] & 0x00002000) >> 13)   # bit 17
        
        # print(f"cus = {self.cus}")
        # print(f"cuc = {self.cuc}")
        # print(f"crs = {self.crs}")
        # print(f"e = {self.e}")
        # print(f"sqrtA = {self.sqrtA}")
        # print(f"m0 = {self.m0}")
        # print(f"deltan = {self.deltan}")
        # print(f"toe = {self.toe}")
        # print(f"iode = {self.iode}")
        
        return
    
    def LoadSubframe3(self):
        """
        IS-GPS-200N pg. 82
        IS-GPS-200N pg. 105
        """
        # word 1-2
        self.LoadPreamble()
        
        # word 3 and 4
        self.ephemerides.cic = TwosComp((self.subframe[2] & 0x3FFFC000) >> 14, 16) * (2**-29)   # bits 1-16
        tmp1 = (self.subframe[2] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[3] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.ephemerides.omega0 = TwosComp((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 5 and 6
        self.ephemerides.cis = TwosComp((self.subframe[4] & 0x3FFFC000) >> 14, 16) * (2**-29)   # bits 1-16
        tmp1 = (self.subframe[4] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[5] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.ephemerides.i0 = TwosComp((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 7 and 8
        self.ephemerides.crc = TwosComp((self.subframe[6] & 0x3FFFC000) >> 14, 16) * (2**-5)    # bits 1-16
        tmp1 = (self.subframe[6] & 0x00003FC0) >> 6                                             # bits 17-24
        tmp2 = (self.subframe[7] & 0x3FFFFFC0) >> 6                                             # bits 1-24
        self.ephemerides.omega = TwosComp((tmp1 << 24) | tmp2, 32) * (PI * 2**-31)
        
        # word 9
        self.ephemerides.omegaDot = TwosComp((self.subframe[8] & 0x3FFFFFC0) >> 6, 24) * (PI * 2**-43)  # bits 1-24
        
        # word 10
        self.ephemerides.iode = np.double((self.subframe[9] & 0x3FC00000) >> 22)                    # bits 1-8
        self.ephemerides.iDot = TwosComp((self.subframe[9] & 0x003FFF00) >> 8, 14) * (PI * 2**-43)  # bits 9-22
        
        # print(f"cic = {self.cic}")
        # print(f"cis = {self.cis}")
        # print(f"crc = {self.crc}")
        # print(f"omega = {self.omega}")
        # print(f"omega0 = {self.omega0}")
        # print(f"omegaDot = {self.omegaDot}")
        # print(f"iode = {self.iode}")
        # print(f"i0 = {self.i0}")
        # print(f"iDot = {self.iDot}")
        
        return
    
    def LoadSubframe4(self):
        # word 1-2
        self.LoadPreamble()
        return
    
    def LoadSubframe5(self):
        # word 1-2
        self.LoadPreamble()
        return
