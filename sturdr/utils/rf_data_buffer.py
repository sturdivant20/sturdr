"""**rf_data_buffer.py**

======  ============================================================================================
file    sturdr/rcvr/rf_data_buffer.py 
brief   Helper for reading RF signal data across multiple processes. 
date    October 2024
======  ============================================================================================
"""

import io
import numpy as np
import configparser
from multiprocessing import shared_memory
# TODO: should i use lock when reading???, enforce no channel reads the same data being writter?

class RfDataBuffer:
    """
    Owner of the RF signal file.
    
    Implemtation of a circular buffer utilizing :obj:`multiprocessing.shared_memory.SharedMemory` 
    to quickly and easily share data between python instances!
    """
    
    __slots__ = 'file_id', 'filepath', 'is_complex', 'bittype', 'chunk_size_ms', 'samples_per_ms', \
                'memory', 'buffer', 'nbytes', 'size', 'dtype', 'write_ptr'
    file_id         : io.BufferedReader             # open/active file
    filepath        : str                           # path to file
    is_complex      : bool                          # is data in complex pairs?
    bittype         : np.dtype                      # data type of incoming RF data
    chunk_size_ms   : int                           # chunk size of stored data in milliseconds
    samples_per_ms  : int                           # samples in 1 millisecond of RF data
    memory          : shared_memory.SharedMemory    # location/pointer to shared memory location
    buffer          : np.ndarray                    # buffer inside of memory
    nbytes          : int                           # length of chunk/buffer in bytes
    size            : int                           # lenght of chunk/buffer in samples
    dtype           : np.dtype                      # datatype of chunk/buffer
    write_ptr       : int                           # current write index of chunk/buffer
    
    def __init__(self, *args):
        """
        Constructor for RFSignalFile Class

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
        self.chunk_size_ms = 100
        if isinstance(args[0], configparser.ConfigParser) or isinstance(args[0], dict):
            self.filepath      = str(args[0]['GENERAL']['in_file'])
            sampling_freq = float(args[0]['RFSIGNAL']['sampling_freq'])
            self.is_complex = (args[0]['RFSIGNAL']['is_complex'] == 'true')
            bittype = int(args[0]['RFSIGNAL']['bit_depth'])
            if 'ms_chunk_size' in args[0]['GENERAL']:
                self.chunk_size_ms = int(args[0]['GENERAL']['ms_chunk_size'])
        else:
            self.filepath = args[0]
            sampling_freq = args[1]
            self.is_complex = args[2]
            bittype = args[3]
            if len(args) > 4:
                self.chunk_size_ms = args[4]
        
        # Check if IQ or real data
        if self.is_complex:
            self.dtype = np.dtype(np.complex128)
        else:
            self.dtype = np.dtype(np.float64)
        
        # find data type
        if bittype == 8:
            self.bittype = np.dtype(np.int8)
        elif bittype == 16:
            self.bittype = np.dtype(np.int16)
        else:
            raise ValueError(f"Data type of {bittype} bit(s) is not valid.")
        
        # intialize
        self.file_id        = open(self.filepath, 'rb')
        self.samples_per_ms = int(sampling_freq * 1e-3)
        self.size           = self.chunk_size_ms * self.samples_per_ms
        self.nbytes         = int(self.size * self.dtype.itemsize)
        self.write_ptr      = 0
        
        # Allocate shared memory
        self.memory = shared_memory.SharedMemory(create=True, size=self.nbytes)
        self.buffer = np.ndarray(self.size, dtype=self.dtype, buffer=self.memory.buf)
        
        return
    
    def __del__(self):
        self.file_id.close()
        self.memory.close()
        self.memory.unlink()
        
    def Push(self, nms: int): #, data: np.ndarray):
        """
        Push new data into the circular buffer. Shift the write index/pointer.
        NOTE: This should only get executed by the ChannelController/Receiver

        Parameters
        ----------
        data : np.ndarray
            New data to be added to the buffer
        nms  : int
            Number of milliseconds to push

        Raises
        ------
        ValueError
            CircularBuffer.maxsize % data.size should be 0!
        """
        data = self.fread(nms * self.samples_per_ms)
        shift = data.size
        if self.size % shift:
            raise ValueError("CircularBuffer.maxsize %% data.size should be 0!")
        
        # update buffer
        old_write_ptr = self.write_ptr
        self.write_ptr += shift
        self.buffer[old_write_ptr:self.write_ptr] = data
        self.write_ptr %= self.size
            
        return
    
    def Pull(self, read_ptr: int, nsamples: int):
        """
        Pull data from the circular buffer.

        Parameters
        ----------
        read_ptr : int
            Starting index of the requested array
        nsamples : int
            Size of the requested array

        Returns
        -------
        np.ndarray
            Requested array
        """
        new_ptr = read_ptr + nsamples
        if new_ptr > self.size:
            new_ptr %= self.size
            return np.concatenate((self.buffer[read_ptr:], self.buffer[:new_ptr]))
        else:
            return self.buffer[read_ptr:new_ptr] #, new_ptr % self.size
    
    def GetNumUnreadSamples(self, read_ptr: int):
        if read_ptr <= self.write_ptr:
            return self.write_ptr - read_ptr
        else:
            return self.size - read_ptr + self.write_ptr
        
    def UpdateWritePtr(self, shift:int):
        self.write_ptr += shift
        self.write_ptr %= self.size
        return
    
# ================================================================================================ #
    
    def fread(self, sample_length: int, skip: int=0):
        """
        Read specified amount of samples from the signal file

        Parameters
        ----------
        time_length : int
            Number of milliseconds to read
        skip : int, optional
            Number of samples to skip, by GENERAL 0

        Returns
        -------
        rfdata : np.ndarray
            Data from the file read
        """
        
        if self.is_complex:
            offset = int(2 * skip * self.bittype.itemsize)
            chunk = int(2 * sample_length)
            rfdata = np.fromfile(self.file_id, self.bittype, offset=offset, count=chunk)
            rfdata = (rfdata[::2] + 1j * rfdata[1::2]).astype(self.dtype)
        else:
            offset = int(skip * self.bittype.itemsize)
            chunk = int(sample_length)
            rfdata = np.fromfile(self.file_id, self.bittype, offset=offset, count=chunk).astype(self.dtype)
            
        return rfdata
    
    def fclose(self):
        """
        Close open signal file
        """
        if self.file_id is not None:
            self.file_id.close()
            self.file_id = None
            
        return
    
    def ftell(self):
        """
        Tell the current file location

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
        
    def fseek(self, samp, offset):
        if self.is_complex:
            self.file_id.seek(2 * self.bittype * (samp + offset), 0)
        else:
            self.file_id.seek(samp + offset, 0)