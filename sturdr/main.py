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

import yaml
import logging
from multiprocessing import Queue
import matplotlib.pyplot as plt

# from sturdr.utils.rf_data_buffer import RfDataBuffer
# from sturdr.channel.gps_l1ca_channel import GpsL1caChannel
# from sturdr.utils.enums import ChannelState
from sturdr.rcvr.channel_controller import ChannelController
from sturdr.rcvr.logger import Logger

def main():
    """
    Main Function
    """
    # Load Configuration
    config_file = './config/gps_l1ca_rcvr.yaml'
    with open(config_file, 'r') as file:
        config = yaml.safe_load(file)
    # pprint(config)
    
    # initialize SturDR logger
    log_queue = Queue()
    log_process = Logger(config, log_queue)
    log_process.start()
    
    # the queue and log level must be added to the logger in the main process/thread
    logger = logging.getLogger('SturDR_Logger')
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.setLevel(logging.DEBUG)
    
    
    # initialize channel controller
    channel_controller = ChannelController(config, log_queue)
    channel_controller.start()
    # logger.info(f"Total Time = {1000 * (end_t - start_t)} ms")
    # log_queue.put(None)
    
    # # initialize channels
    # prn = [1, 7, 14, 17, 19, 21, 30]
    # channel = []
    # rfbuffer = RfDataBuffer(config)
    # queue = Queue()
    # barrier1 = Barrier(len(prn)+1)
    # barrier2 = Barrier(len(prn)+1)
    # for j in range(len(prn)):
    #     channel.append(GpsL1caChannel(config, f'Test_GPS{prn[j]}_Channel', rfbuffer, queue, barrier1, barrier2, j))
    #     channel[j].SetSatellite(prn[j])
    #     channel[j].start()

    # # initialize output
    # L = config['GENERAL']['ms_to_process']
    # L2 = config['GENERAL']['ms_read_size']
    # L_size = int(L / L2)
    # iq = np.zeros((6, L_size, len(prn)))
    # doppler = np.zeros((L_size, len(prn)))
    # cn0 = np.zeros((L_size, len(prn)))
    # TOW = np.zeros(len(prn))
    # ID  = [''] * len(prn)
    
    # loop_t = time.time()
    # # run loop
    # for i in range(0,L_size):
    #     # read the next ms of data
    #     # rfbuffer.Push(L2)
    #     rfbuffer.NextChunk()
        
    #     # # process data
    #     # for j in range(len(prn)):
    #     #     if channel[j].channel_status.State == ChannelState.TRACKING:
    #     #         channel[j].Track()
    #     #     elif channel[j].channel_status.State == ChannelState.ACQUIRING:
    #     #         channel[j].Acquire()
    #     #     TOW[j] = channel[j].channel_status.header.TOW
    #     #     ID[j] = channel[j].channel_status.header.ID
            
    #     #     if channel[j].channel_status.Ephemeris:
    #     #         print(f"SV_POS: {np.array2string(channel[j].nav_packet.SatPos, precision=3, floatmode='fixed')}")
        
    #     # inform the channels of new data
    #     barrier1.wait()
        
    #     # wait for the channels to process new data
    #     barrier2.wait()
        
    #     # process channel results
    #     while queue.qsize() > 0:
    #         packet       = queue.get()
    #         j            = packet.header.ChannelNum
    #         TOW[j]       = packet.header.TOW
    #         ID[j]        = packet.header.ID
    #         iq[0,i,j]    = packet.IP
    #         iq[1,i,j]    = packet.QP
    #         doppler[i,j] = packet.Doppler
    #         cn0[i,j]     = packet.CN0
            
    #     # if not np.any(np.isnan(TOW)):
    #     #     print(f"Delta TOW = {np.array2string(1000*(TOW-np.min(TOW)), precision=6, floatmode='fixed')} ms")
            
    # end_t = time.time()
    # print(f"Total Time = {1000 * (end_t - start_t)} ms")
    # print(f"Loop Time = {1000 * (end_t - loop_t)} ms")
            
    # # plot IP and QP
    # plt.figure()
    # for j in range(len(prn)):
    #     plt.plot(iq[0,:,j], '.', label=f'IP GPS{prn[j]}')
    #     plt.plot(iq[1,:,j], '.', label=f'QP GPS{prn[j]}')
    # plt.legend()

    # # plot carrier doppler
    # plt.figure()
    # for j in range(len(prn)):
    #     plt.plot(doppler[:,j], label=f'Doppler GPS{prn[j]}')
    # plt.legend()

    # # plot cn0
    # plt.figure()
    # for j in range(len(prn)):
    #     plt.plot(cn0[:,j], label=f'C/N0 GPS{prn[j]}')
    # plt.legend()

    # plt.show()

    return
    
if __name__ == '__main__':
    main()