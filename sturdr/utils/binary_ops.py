"""**binary_ops.py**

======  ============================================================================================
file    sturdr/utils/binary_ops.py
brief   Binary operations.
date    October 2024
======  ============================================================================================
"""

import numpy as np

def RotL(data: np.uint32, size: np.uint32):
    return (data << size) ^ (data >> (32 - size))

def CheckBit(data: np.uint32, idx: np.uint32):
    # return data & (1 << idx)
    return (data >> idx) & 1

def SetBit(data: np.uint32, idx: np.uint32):
    return data | (1 << idx)

def ClearBit(data: np.uint32, idx: np.uint32):
    return data & ~(1 << idx)

def FlipBit(data: np.uint32, idx: np.uint32):
    return data ^ (1 << idx)

def SetBitTo(data: np.uint32, idx: np.uint32, val: bool):
    return (data & ~(1 << idx)) | (np.uint32(val) << idx)

def GetDataBit(data: np.uint32, idx: np.uint32):
    return (data >> (31 - idx)) & 1

def GetDataBits(data: np.uint32, beg: np.uint32, end: np.uint32):
    return (data >> (31 - end)) & ((1 << (end - beg + 1)) - 1)

def SetDataBitTo(data: np.uint32, idx: np.uint32, val: bool):
    return (data & ~(1 << (31 - idx))) | (np.uint32(val) << (31 - idx))

def TwosCompliment(data: np.uint32, size: np.uint32):
    # mask to only desired bits
    tmp = data & ((1 << size) - 1)
    
    # if bit is set, do compliment
    if tmp >> (size - 1) & 1:
        sb = np.uint32(1 << (size - 1))
        return -np.double(sb - (tmp & ~sb))
    else:
        return np.double(tmp)

    