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
            - Borre, Akos, bertelsen, Rinder, Jensen
        5. "Global Positioning System: Signals, Measurements, and Performance", 2nd Edition, 2006
            - Misra & Enge
        6. "IS-GPS-200N", 2022
======  ============================================================================================
"""

import configparser
import numpy as np
from sturdr.utils.rf_signal_file import RFSignalFile
from sturdr.channel.gps_l1ca_channel import gps_l1ca_code
from sturdr.dsp.acquisition import SerialSearch, PcpsSearch, Peak2NoiseFloorComparison, Peak2PeakComparison
from sturdr.dsp.tracking import NaturalFrequency, FLLassistedPLL_3rdOrder, PLL_2ndOrder, TrackingKF
from sturdr.dsp.discriminators import PllCostas, FllAtan2, DllNneml
from sturdr.dsp.gnss_signal import CorrelateEPL, CodeNCO, CarrierNCO
import matplotlib.pyplot as plt

def main():
    """
    Main Function
    """
    
    # Load Configuration
    config_file = './config/gps_l1ca_rcvr.ini'
    rcvr_config = configparser.ConfigParser()
    rcvr_config.read(config_file)
    
    # open signal file and read 1 ms
    signal_file = RFSignalFile(rcvr_config)
    rfdata = signal_file.tread(1, 20000)
    
    # try acquisition
    code = gps_l1ca_code(7)
    x = np.arange(-5000, 5000+1, 100)
    if rcvr_config['ACQUISITION']['method'] == 'serial':
        correlation_map = SerialSearch(rfdata, code, 5000, 50, 20e6, 1.023e6, 5000445.88565834)
        y = np.linspace(0,1023,2046)
        r = 1023/2046
    else:
        correlation_map = PcpsSearch(rfdata, code, 5000, 100, 20e6, 1.023e6, 5000445.88565834)
        y = np.linspace(0,1023,20000)
        r = 1023/20000
    
    if rcvr_config['ACQUISITION']['test'] == 'peak2noisefloor':
        first_peak_idx, acquisition_metric = Peak2NoiseFloorComparison(correlation_map)
    else:
        first_peak_idx, acquisition_metric = Peak2PeakComparison(correlation_map, 20000, 20)
    
    print(f"Doppler: {x[first_peak_idx[0]]}")
    print(f"Code: {first_peak_idx[1]}")
    print(f"Metric: {acquisition_metric}")
    
    # try tracking
    signal_file.fseek(20000+first_peak_idx[1], 0)
    w0p = NaturalFrequency(15,3)
    w0f = NaturalFrequency(1,2)
    w0d = NaturalFrequency(0.1,2)
    code_freq = 1.023e6 + (1.023e6 / 1575.42e6) * float(x[first_peak_idx[0]]) # [chip/s]
    carrier_freq = 5000445.88565834 + float(x[first_peak_idx[0]]) # [Hz]
    carrier_jitter = 0.0
    rem_code_phase = 0.0
    rem_carrier_phase = 0.0
    jitter_accumulator = 0.0
    doppler_accumulator = float(x[first_peak_idx[0]])
    chip_accumulator = 0.0 #(1.023e6 / 1575.42e6) * float(x[first_peak_idx[0]])
    track_kf = TrackingKF(w0p, w0f, w0d, float(x[first_peak_idx[0]]), intermediate_freq=5000445.88565834, T=0.001)
    dt = 0.001
    
    N = 5000
    IP = np.zeros(N)
    QP = np.zeros(N)
    carrier_doppler = np.zeros(N)
    code_doppler = np.zeros(N)
    for i in range(N):
        new_code, rem_code_phase = CodeNCO(code, 20e6, code_freq, 1023, rem_code_phase)
        L = new_code.size
        new_carrier, rem_carrier_phase = CarrierNCO(20e6, carrier_freq, carrier_jitter, L, rem_carrier_phase)
        rfdata = signal_file.sread(L)
        
        IE, QE, IP[i], QP[i], IL, QL, IP_1, QP_1, IP_2, QP_2 = CorrelateEPL(rfdata, new_carrier, new_code, 19)
        phase_err = PllCostas(IP[i], QP[i]) / (2 * np.pi)                 # [cycle]
        freq_err = FllAtan2(IP_1, IP_2, QP_1, QP_2, dt/2) / (2 * np.pi) # [Hz]
        chip_err = DllNneml(IE, QE, IL, QL)                               # [chip]
        
        carrier_doppler[i], jitter_accumulator, doppler_accumulator = FLLassistedPLL_3rdOrder(
            phase_err, freq_err, jitter_accumulator, doppler_accumulator, dt, w0p, w0f
        )
        code_doppler[i], chip_accumulator = PLL_2ndOrder(chip_err, chip_accumulator, dt, w0d)
        code_doppler[i] += (1.023e6 / 1575.42e6) * carrier_doppler[i]
        
        # track_kf.UpdateIntegrationTime(dt)
        # x = track_kf.Run(phase_err, freq_err, chip_err)
        # rem_carrier_phase = 2.0 * np.pi * np.remainder(x[0], 1.0)
        # carrier_doppler[i] = x[1]
        # carrier_jitter = x[2]
        # rem_code_phase = np.mod(x[3], 1023)
        # code_doppler[i] = x[4] + (1.023e6 / 1575.42e6) * (x[1] + dt * x[2])
        
        carrier_freq = 5000445.88565834 + carrier_doppler[i]
        code_freq = 1.023e6 + code_doppler[i]
        
    
    # # plot map
    # x,y = np.meshgrid(x,y)
    # plt.figure()
    # ax = plt.axes(projection='3d')
    # ax.set_xlabel("Doppler [Hz]")
    # ax.set_ylabel("Code [Chips]")
    # ax.plot_surface(x, y, correlation_map.T, 
    #                 rstride=1, cstride=5, 
    #                 cmap=plt.get_cmap('turbo'))
    # plt.show()
    
    # plot IP and QP
    plt.figure()
    plt.plot(IP, label='IP')
    plt.plot(QP, label='QP')
    plt.legend()
    
    plt.figure()
    plt.plot(carrier_doppler, label='doppler')
    plt.legend()
    
    plt.figure()
    plt.plot(code_doppler, label='code doppler')
    plt.legend()
    
    plt.show()
    
    return
    
if __name__ == '__main__':
    main()