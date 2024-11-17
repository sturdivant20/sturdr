"""**iotools.py**

======  ============================================================================================
file    sturdr/utils/iotools.py
brief   Basic file in/out tools.
date    November 2024
======  ============================================================================================
"""

import os

def EnsurePathExists(path: str):
    """
    Make sure directory chosen exists

    Parameters
    ----------
    path : str
        path to check
    """
    os.makedirs(os.path.realpath(path), exist_ok=True)