"""**main.py**

======  ============================================================================================
file    sturdr/main.py
brief   Main execution of the GNSS receiver.
date    October 2024
refs    1. "Understanding GPS/GNSS Principles and Applications", 3rd Edition, 2017 
            - Kaplan & Hegarty
        2. "Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems", 2nd Edition, 2013
            - Groves
        3. "Are PLLs Dead? A Tutorial on Kalman Filter-Based Techniques for Digital Carrier Syncronization" 
            - Vila-Valls, Closas, Navarro, Fernandez-Prades
        4. "A Software-Defined GPS and Galileo Receiver: A Single-Frequency Approach", 2007
            - Borre, Akos, Bertelsen, Rinder, Jensen
        5. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        6. "IS-GPS-200N", 2022
======  ============================================================================================
"""

import numpy as np
import yaml
from pprint import pprint
from multiprocessing import shared_memory, Queue, Event
import matplotlib.pyplot as plt
import time

from sturdr.utils.rf_data_buffer import RfDataBuffer
from sturdr.channel.gps_l1ca_channel import GpsL1caChannel

def main():
    """
    Main Function
    """
    
    start_t = time.time()
    
    # Load Configuration
    config_file = './config/gps_l1ca_rcvr.yaml'
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    # pprint(config)
    
    # initialize channels
    prn = [30] #[1, 7, 14, 17, 19, 21, 30]
    channel = []
    queue = Queue()
    rfbuffer = RfDataBuffer(config)
    for j in range(len(prn)):
        channel.append(GpsL1caChannel(config, f'Test_GPS{prn[j]}_Channel', rfbuffer, queue, j))
        channel[j].SetSatellite(prn[j])
        channel[j].start()

    # initialize output
    L = 36000
    L2 = 20
    L_size = int(L / L2)
    iq = np.zeros((6, L_size, len(prn)))
    doppler = np.zeros((L_size, len(prn)))
    cn0 = np.zeros((L_size, len(prn)))
    
    loop_t = time.time()
    # run loop
    for i in range(0,L_size):
        # read the next ms of data
        rfbuffer.Push(L2)
        # rfbuffer.NextChunk()
        
        # inform the channels of new data
        for ch in channel:
            ch.event_start.set()
            
        # wait for the channels to process new data
        for ch in channel:
            ch.event_done.wait()
            ch.event_done.clear()
        
        # process results
        while queue.qsize() > 0:
            packet       = queue.get()
            j            = packet.ChannelNum
            iq[0,i,j]    = packet.IP
            iq[1,i,j]    = packet.QP
            doppler[i,j] = packet.Doppler
            cn0[i,j]     = packet.CN0
            
    end_t = time.time()
    print(f"Total Time = {1000 * (end_t - start_t)} ms")
    print(f"Loop Time = {1000 * (end_t - loop_t)} ms")
            
    # plot IP and QP
    plt.figure()
    for j in range(len(prn)):
        plt.plot(iq[0,:,j], '.', label=f'IP GPS{prn[j]}')
        plt.plot(iq[1,:,j], '.', label=f'QP GPS{prn[j]}')
    plt.legend()

    # plot carrier doppler
    plt.figure()
    for j in range(len(prn)):
        plt.plot(doppler[:,j], label=f'Doppler GPS{prn[j]}')
    plt.legend()

    # plot cn0
    plt.figure()
    for j in range(len(prn)):
        plt.plot(cn0[:,j], label=f'C/N0 GPS{prn[j]}')
    plt.legend()

    plt.show()

    return
    
if __name__ == '__main__':
    main()