"""**enums.py**

======  ============================================================================================
file    sturdr/utils/enums.py 
brief   Satellite navigation enumerations.
date    October 2024
======  ============================================================================================
"""

from enum import unique, IntEnum

# ================================================================================================ #

@unique
class GnssSystem(IntEnum):
    """
    Enumeration class for GNSS signal constellations.
    """
    UNKNOWN = 0
    GPS     = 1
    GALILEO = 2
    GLONASS = 3
    BEIDOU  = 4
    QZSS    = 5
    IRNSS   = 6
    SBAS    = 7
    
    def __str__(self):
        return str(self.name)
    
@unique
class GnssSignalTypes(IntEnum):
    """
    Enumeration class for GNSS satellite broadcast signal types.
    """
    UNKNOWN     = 0
    GPS_L1CA    = 1
    GPS_L1C     = 2
    GPS_L2C     = 3
    GPS_L5      = 4
    GALILEO_E1  = 5
    GALILEO_E6  = 6
    GALILEO_E5  = 7
    GALILEO_E5a = 8
    GALILEO_E5b = 9
    
    def __str__(self):
        return str(self.name)
        # return str(self.name).replace("_", " ")
    
@unique
class MeasurementType(IntEnum):
    """
    Enumeration class for measurement types in the navigator.
    """
    UNKNOWN     = 0
    PSEUDORANGE = 1
    DOPPLER     = 2
    PHASE       = 3
    CN0         = 4
    
    def __str__(self):
        return str(self.name)
    
# ================================================================================================ #

@unique
class ChannelState(IntEnum):
    """
    Enumeration class for channel state according to the defined state machine architecture.
    """
    OFF       = 0
    IDLE      = 1
    ACQUIRING = 2
    TRACKING  = 3
    
    def __str__(self):
        return str(self.name)
    
@unique
class TrackingFlags(IntEnum):
    """
    Tracking flags to represent the current stage of tracking. They are to be intepreted in binary 
    format, to allow multiple state represesented in one decimal number. 
    Similar to states in https://developer.android.com/reference/android/location/GnssMeasurement 
    """

    UNKNOWN       = 0    # 0000 0000 No tracking
    CODE_LOCK     = 1    # 0000 0001 Code found (after first tracking?)
    BIT_SYNC      = 2    # 0000 0010 First bit identified 
    SUBFRAME_SYNC = 4    # 0000 0100 First subframe found
    TOW_DECODED   = 8    # 0000 1000 Time Of Week decoded from navigation message
    EPH_DECODED   = 16   # 0001 0000 Ephemeris from navigation message decoded
    TOW_KNOWN     = 32   # 0010 0000 Time Of Week known (retrieved from Assisted Data), to be set if TOW_DECODED set.
    EPH_KNOWN     = 64   # 0100 0000 Ephemeris known (retrieved from Assisted Data), to be set if EPH_DECODED set.
    FINE_LOCK     = 128  # 1000 0000 Fine tracking lock
    
    def __str__(self):
        return str(self.name)
    
@unique
class LoopLockState(IntEnum):
    """
    Enumeration class for signal lock state of internal tracking loops
    """
    UNKNOWN      = 0
    PULL_IN      = 1
    WIDE_TRACK   = 2
    NARROW_TRACK = 3
    
    def __str__(self):
        return str(self.name)