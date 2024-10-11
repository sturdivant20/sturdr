"""**constants.py**

======  ============================================================================================
file    sturdr/utils/constants.py 
brief   Satellite navigation constants.
date    October 2024
refs    1. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        2. "IS-GPS-200N", 2022
======  ============================================================================================
"""

# ===== GENERIC ================================================================================== #
PI          = 3.1415926535898 # GPS defined pi constant
HALF_PI     = PI / 2.0        #
TWO_PI      = PI * 2.0        #
LIGHT_SPEED = 299792458.0     # speed of light in vacuum [m/s]
WEEK        = 604800.0        # seconds in a week
HALF_WEEK   = 302400.0        # seconds in half a week
M_PER_FT    = 0.3048          # meters to feet
MIN_PER_DAY = 1440.0          # minutes per day

# ===== WGS84 EARTH ============================================================================== #
WGS84_R0        = 6378137.0           # Earth's equatorial radius (semi-major axis) [m]
WGS84_RP        = 6356752.314245      # Earth's polar radius (semi-minor axis) [m]
WGS84_E         = 0.0818191908429654  # Eccentricity, e
WGS84_E2        = 0.00669437999019758 # Eccentricity squared, e^2
WGS84_F         = 0.00335281066477569 # Earth's inverse flattening

# ===== ORBITAL ================================================================================== #
J2            = 1.0826269e-03    # Second zonal harmonic coefficient
J3            = -2.5323000e-06   # Third zonal harmonic coefficient
J4            = -1.6204000e-06   # Fourth zonal harmonic coefficient
RELATIVISTC_F = -4.442807633e-10 # Relativistic clock correction [s/m^(1/2)]
OMEGA_DOT     = 7.2921151467e-5  # Earth's rotation rate [rad/s]
GM            = 3.986004415e14   # Earth's gravitational constant [m^3/s^2]

# ===== GNSS CONSTELLATIONS ====================================================================== #
GPS_L1CA_CARRIER_FREQ = 1575.42e6 # [Hz]
GPS_L1CA_CODE_FREQ    = 1.023e6   # [Hz]
GPS_L1CA_CODE_MS      = 1         # number of codes per millisecond of signal
GPS_L1CA_CODE_SIZE    = 1023      # number of chips in the C/A code

# ===== NAVIGATION MESSAGES ====================================================================== #
LNAV_PREAMBLE_BITS = 0b10001011 # GPS L1 C/A preamble bit sequence
LNAV_PREAMBLE_SIZE = 8          # length of GPS L1 C/A preamble
LNAV_MS_PER_BIT    = 20         # number of milliseconds per GPS L1 C/A data bit
LNAV_SUBFRAME_SIZE = 300        # number of data bits in a GPS L1 C/A subframe
LNAV_WORD_SIZE     = 30         # number of data bits in a GPS L1 C/A word