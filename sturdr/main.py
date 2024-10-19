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
from sturdr.rcvr.channel_controller import ChannelController
from sturdr.rcvr.logger import Logger

# TODO: connect a navigator to the channel controller - this needs to be a two-way pipe

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
    logger.setLevel(logging.INFO)
    
    # initialize channel controller
    channel_controller = ChannelController(config, log_queue)
    channel_controller.start()

    return
    
if __name__ == '__main__':
    main()