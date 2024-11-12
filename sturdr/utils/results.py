"""**results.py**

======  ============================================================================================
file    sturdr/utils/results.py 
brief   Functions for parsing results CSVs.
date    November 2024
======  ============================================================================================
"""

import os
import csv
import numpy as np
from pathlib import Path
from dataclasses import asdict

from sturdr.channel.channel import ChannelPacket
from sturdr.nav.ephemeris import Ephemerides
from sturdr.rcvr.navigator import NavResult

def EnsurePathExists(path: str):
    """
    Make sure directory chosen exists

    Parameters
    ----------
    path : str
        path to check
    """
    os.makedirs(os.path.realpath(path), exist_ok=True)
    
def ParseChannelResults(filename: str):
    """Parses the ChannelStatus results CSV

    Parameters
    ----------
    filename : str
        filename (with relevent pathing) of the CSV

    Returns
    -------
    results : dict
        Dictionary containing numbered entrys for each channel inside results. Each channel entry 
        contains the following paramters:
            - Constellation : str
            - Signal        : str
            - ID            : str 
            - State         : str 
            - CodeLock      : list[bool]
            - CarrierLock   : list[bool] 
            - DataLock      : list[bool]
            - Ephemeris     : list[bool]
            - Week          : list[float]
            - ToW           : list[float]
            - CNo           : list[float]
            - Doppler       : list[float]
            - CodePhase     : list[float]
            - CarrierPhase  : list[float]
            - IP            : list[float]
            - QP            : list[float]
            - IE            : list[float]
            - QE            : list[float]
            - IL            : list[float]
            - QL            : list[float]
            - IP_1          : list[float]
            - QP_1          : list[float]
            - IP_2          : list[float]
            - QP_2          : list[float]
    """
    
    # initialize dictionary
    results = {}
    types = dict(zip(
        list(asdict(ChannelPacket()).keys())[4:], 
        [str, bool, bool, bool, bool, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float, float]
        )
    )
    print(types)
    
    # open the file 
    with open(filename, "r") as file:
        reader = csv.DictReader(file) #, fieldnames=asdict(ChannelPacket()).keys())
        
        for row in reader:
            # make sure each channel contains its own dictionary
            i = int(row['ChannelNum'])
            if i not in results:
                results[i]                  = {}
                results[i]['Constellation'] = row['Constellation']
                results[i]['Signal']        = row['Signal']
                results[i]['ID']            = row['ID']
                for k in types.keys():
                    results[i][k] = []
                    
            # append each of the lists
            for k,v in types.items():
                if v == bool:
                    results[i][k].append(row[k] == "True")
                else:
                    results[i][k].append(v(row[k]))
                
    return dict(sorted(results.items()))

def ParseNavigationResults(filename: str):
    """Parses the navigation results CSV

    Parameters
    ----------
    filename : str
        filename (with relevent pathing) of the CSV

    Returns
    -------
    results : dict
        Dictionary containing the following fields:
            - GpsWeek : list[float]
            - GpsToW  : list[float]
            - x       : list[float]
            - y       : list[float]
            - z       : list[float]
            - vx      : list[float]
            - vy      : list[float]
            - vz      : list[float]
    """
    
    # initialize dictionary
    results = {}
    keys = list(asdict(NavResult()).keys())
    for k in keys:
        results[k] = []
    
    # open the file 
    with open(filename, "r") as file:
        reader = csv.DictReader(file) #, fieldnames=asdict(ChannelPacket()).keys())
        
        for row in reader:
            # make sure each channel contains its own dictionary
            for k in keys:
                results[k].append(float(row[k]))
            
                
    return results