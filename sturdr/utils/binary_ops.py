"""**binary_ops.py**

======  ============================================================================================
file    sturdr/utils/binary_ops.py
brief   Binary operations.
date    October 2024
======  ============================================================================================
"""

import numpy as np
from numba import njit

# Least significant bit is assumed to be at position 31 (last bit of uint32)
# TODO: check to make sure these functions are correct

@njit(cache=True, fastmath=True)
def SetBit(x: np.uint32, n: np.uint8):
    """
    Set a data bit to 1

    Parameters
    ----------
    x : np.uint32
        Number to modify
    n : np.uint8
        Position of bit to set (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    x : np.uint32
        Modified number
    """
    assert(not (n > 31))
    x |= (0x80000000 >> n)
    return x

@njit(cache=True, fastmath=True)
def ClearBit(x: np.uint32, n: np.uint8):
    """
    Set a data bit to 0

    Parameters
    ----------
    x : np.uint32
        Number to modify
    n : np.uint8
        Position of bit to clear (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    x : np.uint32
        Modified number
    """
    assert(not (n > 31))
    x &= ~(0x80000000 >> n)
    return x

@njit(cache=True, fastmath=True)
def ModifyBit(x: np.uint32, n: np.uint8, b: bool):
    """
    Set a data bit to "b"

    Parameters
    ----------
    x : np.uint32
        Number to modify
    n : np.uint8
        Position of bit to modify (Position 0 is MSB and 31 is LSB)
    b : bool
        New value of bit you want to modify

    Returns
    -------
    x : np.uint32
        Modified number
    """
    assert(not (n > 31))
    x = (x & ~(0x80000000 >> n)) | (b << (31 - n))
    return x

@njit(cache=True, fastmath=True)
def CheckBit(x: np.uint32, n: np.uint8):
    """
    Check the value of a bit

    Parameters
    ----------
    x : np.uint32
        Number to check
    n : np.uint8
        Position of bit to check (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    b : bool
        Modified number
    """
    assert(not (n > 31))
    b = bool(x & (0x80000000 >> n))
    return b

@njit(cache=True, fastmath=True)
def CheckBits(x: np.uint32, beg: np.uint8, end: np.uint8):
    """
    Check the value of multiple bit

    Parameters
    ----------
    x : np.uint32
        Number to check
    beg : np.uint8
        Position of first bit to check (Position 0 is MSB and 31 is LSB)
    end : np.uint8
        Position of last bit to check (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    b : np.uint32
        Modified number
    """
    assert(not (beg > 31))
    assert(not (end > 31))
    return (x >> (31 - end)) & ((1 << (end - beg + 1)) - 1)

@njit(cache=True, fastmath=True)
def InvCheckBit(x: np.uint32, n: np.uint8):
    """
    Check the value of a bit

    Parameters
    ----------
    x : np.uint32
        Number to check
    n : np.uint8
        Position of bit to check (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    b : bool
        Modified number
    """
    assert(not (n > 31))
    b = bool((x >> n) & 1)
    return b

@njit(cache=True, fastmath=True)
def MultiXor(x: np.uint32, n: np.ndarray[np.uint8]):
    """
    XOR multiple bits together

    Parameters
    ----------
    x : np.uint32
        Number to use
    n : np.ndarray[np.uint8]
        Positions of bits to check (Position 0 is MSB and 31 is LSB)

    Returns
    -------
    r : bool
        XOR'ed number
    """
    r = CheckBit(x, n[0])
    for i in range(1, n.size):
        r ^= CheckBit(x, n[i])
    return r

@njit(cache=True, fastmath=True)
def TwosComp(x: np.uint32, n: np.uint8):
    """
    Two's compliment to create a signed integer
    
    Parameters
    ----------
    x : np.uint32
        Number to use
    n : np.ndarray[np.uint8]
        Number of bits in the integer
        
    Returns
    -------
    x : np.int32
        Signed integer
    """
    if (x & (1 << (n - 1))):
        x -= (1 << n)
    return x