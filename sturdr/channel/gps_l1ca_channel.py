"""**gps_l1ca_channel.py**

======  ============================================================================================
file    sturdr/channel/gps_l1ca_channel.py
brief   Implementation of channel.py for GPS L1 C/A signals.
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        3. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, bertelsen, Rinder, Jensen
======  ============================================================================================
"""

import numpy as np

# ===== GPS L1 C/A Code Generator ================================================================ #
delay = {
    1: (1,5),
    2: (2,6),
    3: (3,7),
    4: (4,8),
    5: (0,8),
    6: (1,9),
    7: (0,7),
    8: (1,8), 
    9: (2,9),
    10: (1,2),
    11: (2,3),
    12: (4,5),
    13: (5,6),
    14: (6,7),
    15: (7,8),
    16: (8,9),
    17: (0,3),
    18: (1,4),
    19: (2,5),
    20: (3,6),
    21: (4,7),
    22: (5,8),
    23: (0,2),
    24: (3,5),
    25: (4,6),
    26: (5,7),
    27: (6,8),
    28: (7,9),
    29: (0,5),
    30: (1,6),
    31: (2,7),
    32: (3,8),
}

def gps_l1ca_code(prn: np.uint8):
    """Generates the GPS L1 C/A code based on the requested PRN

    Parameters
    ----------
    prn : np.uint8
        Satellite ID

    Returns
    -------
    x : np.ndarray
        Requested C/A code
    """
    
    # initialize registers
    g1 = np.ones(10)
    g2 = np.ones(10)
    s1 = delay[prn][0]
    s2 = delay[prn][1]
    
    x = np.zeros(1023, dtype=np.int8)
    for i in range(1023):
        # next code chip
        g2i = (((g2[s1] + g2[s2]) % 2) + g1[9]) % 2
        x[i] = 2 * g2i - 1 # enforce +/- 1
        
        # apply register feedback
        f1 = (g1[9] + g1[2]) % 2
        f2 = (g2[9] + g2[8] + g2[7] + g2[5] + g2[2] + g2[1]) % 2
        for j in range(9,0,-1):
            g1[j] = g1[j-1]
            g2[j] = g2[j-1]
        g1[0] = f1
        g2[0] = f2
    
    return x