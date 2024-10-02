"""**rf_signal_file.py**

======  ============================================================================================
file    sturdr/utils/rf_signal_file.py 
brief   Helper for reading RF signal data. 
date    October 2024
======  ============================================================================================
"""

import numpy as np
import configparser

class RFSignalFile:
    chunk_size_ms    : int = 100
    chunk            : np.ndarray
    chunk_ms_counter : int
    samples_per_ms   : int
    
    def __init__(self, *args):
        """Constructor for RFSignalFile Class

        Parameters
        ----------
        config : dict
            Condifuration dictionary

        Raises
        ------
        ValueError
            Invalid bit depth (int8 or int16)
        """
        
        # Parse objects from config
        if isinstance(args[0], configparser.ConfigParser) or isinstance(args[0], dict):
            self.filepath      = str(args[0]['DEFAULT']['in_file'])
            self.sampling_freq = float(args[0]['RFSIGNAL']['sampling_freq'])
            self.chunk_size_ms = int(args[0]['DEFAULT']['ms_chunk_size'])
            self.is_complex = (args[0]['RFSIGNAL']['is_complex'] == 'true')
            bit_depth = int(args[0]['RFSIGNAL']['bit_depth'])
        else:
            self.filepath = args[0]
            self.sampling_freq = args[1]
            self.is_complex = args[2]
            bit_depth = args[3]
        
        # Check if IQ or real data
        if self.is_complex:
            self.dtype = np.complex128
        else:
            self.dtype = np.double
        
        # find data type
        if bit_depth == 8:
            self.bit_depth = np.dtype(np.int8)
        elif bit_depth == 16:
            self.bit_depth = np.dtype(np.int16)
        else:
            raise ValueError(f"Data type of {bit_depth} bit(s) is not valid.")
        
        # intialize
        self.file_id = open(self.filepath, 'rb')
        self.samples_per_ms = int(self.sampling_freq * 1e-3)
        self.chunk = np.empty((1, self.chunk_size_ms * self.samples_per_ms))
        self.chunk_ms_counter = self.chunk_size_ms
        
        return
    
    def __del__(self):
        self.file_id.close()
        
    def GetMsChunk(self, num_ms: int):
        """Return the next millisecond(s) of data

        Parameters
        ----------
        num_ms : int
            Number of milliseconds requested

        Returns
        -------
        np.ndarray
            Requested RF data

        Raises
        ------
        ValueError
            The number of milliseconds requested should be a multiple of the chunck size for optimal 
            read
        """
        if self.chunk_size_ms % num_ms:
            raise ValueError(f"The number of millisecond requested should be a multiple of the chunck" + 
                             f"size for optimal read ({num_ms} not multiple of {self.chunk_size_ms}).")
            
        # Check if new data needs to be loaded
        if self.chunk_ms_counter == self.chunk_size_ms:
            self.chunk = self.tread(self.chunk_size_ms)
            self.chunk_ms_counter = 0
            
        start_idx = self.chunk_ms_counter * self.samples_per_ms
        stop_idx = self.chunk_ms_counter * self.samples_per_ms + self.samples_per_ms * num_ms
        self.chunk_ms_counter += num_ms
        
        return self.chunk[start_idx:stop_idx]
    
    def tread(self, time_length: int, skip: int=0):
        """Read specified amount of time from the signal file

        Parameters
        ----------
        time_length : int
            Number of milliseconds to read
        skip : int, optional
            Number of samples to skip, by default 0

        Returns
        -------
        rfdata : np.ndarray
            Data from the file read
        """
        if self.is_complex:
            offset = int(2 * skip * self.bit_depth.itemsize)
            chunk = int(2 * (time_length * 1e-3) * self.sampling_freq)
            rfdata = np.fromfile(self.file_id, self.bit_depth, offset=offset, count=chunk)
            rfdata = (rfdata[::2] + 1j * rfdata[1::2]).astype(self.dtype)
        else:
            offset = int(skip * self.bit_depth.itemsize)
            chunk = int((time_length * 1e-3) * self.sampling_freq)
            rfdata = np.fromfile(self.file_id, self.bit_depth, offset=offset, count=chunk).astype(self.dtype)
            
        return rfdata
    
    def sread(self, sample_length: int, skip: int=0):
        """Read specified amount of samples from the signal file

        Parameters
        ----------
        time_length : int
            Number of milliseconds to read
        skip : int, optional
            Number of samples to skip, by default 0

        Returns
        -------
        rfdata : np.ndarray
            Data from the file read
        """
        
        if self.is_complex:
            offset = int(2 * skip * self.bit_depth.itemsize)
            chunk = int(2 * sample_length)
            rfdata = np.fromfile(self.file_id, self.bit_depth, offset=offset, count=chunk)
            rfdata = (rfdata[::2] + 1j * rfdata[1::2]).astype(self.dtype)
        else:
            offset = int(skip * self.bit_depth.itemsize)
            chunk = int(sample_length)
            rfdata = np.fromfile(self.file_id, self.bit_depth, offset=offset, count=chunk).astype(self.dtype)
            
        return rfdata
    
    def fclose(self):
        """Close open signal file
        """
        if self.file_id is not None:
            self.file_id.close()
            self.file_id = None
            
        return
    
    def ftell(self):
        """Tell the current file location

        Returns
        -------
        int
            Current sample index

        Raises
        ------
        Warning
            No signal file open
        """
        if not self.file_id is None:
            if self.is_complex:
                return int(self.file_id.tell() / 2)
            else:
                return int(self.file_id.tell())
        else:
            raise Warning("Signal file not open, cannot return current cursor position.")
            return -1
        
    def fseek(self, samp, offset):
        if self.is_complex:
            self.file_id.seek(2 * self.bit_depth * (samp + offset), 0)
        else:
            self.file_id.seek(samp + offset, 0)