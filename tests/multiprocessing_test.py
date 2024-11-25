import numpy as np
import time
import logging
import logging.handlers
from pprint import pprint
from multiprocessing import shared_memory, Queue, Process, Pipe
import multiprocessing.connection
from sturdr.rcvr.logger import ColorFormatter

#* === PARALLEL THREAD OPERATIONS === *#

def worker_process(number, log_name, log_queue, shm_name, size, dtype):
    # find logger
    logger = logging.getLogger(log_name)
    logger.addHandler(logging.handlers.QueueHandler(log_queue))
    logger.setLevel(logging.DEBUG)

    # find shared memory
    shm = shared_memory.SharedMemory(name=shm_name, create=False)
    shared_array = np.ndarray(size, dtype=dtype, buffer=shm.buf)

    # send some messages over the logger
    logger.debug(f"Worker {number}: initialized!")
    logger.info(f"Worker {number}: Shared memory '{shm_name}' has size {shared_array.size}")
    time.sleep(1)
    shared_array[number] = number + 1
    logger.info(f"Worker {number}: {shared_array}")
    logger.warn(f"Worker {number} done!")
    return

def logger_process(log_name, log_queue, log_level):
    root = logging.getLogger(log_name)
    # root.setLevel(log_level)
    console = logging.StreamHandler()
    console.setFormatter(ColorFormatter())
    root.addHandler(console)

    while True:
        try:
            record = log_queue.get()

            # check for termination
            if record is None:
                break

            # log specified message
            root.handle(record)
        except:
            import sys, traceback
            print('Error in logger process', file=sys.stderr)
            traceback.print_exc(file=sys.stderr)

    return

#* === MAIN THREAD OPERATIONS === *#

if __name__ == '__main__':
    my_pipes = []
    for i in range(10):
        my_pipes.append(Pipe())
    
    # pprint(my_pipes)
    my_pipes = np.asarray(my_pipes, dtype=multiprocessing.connection.Connection)
    pprint(my_pipes)
    # # create a multiprocessing shared memory allocation
    # size = 10
    # dtype = np.dtype(np.int8)
    # nbytes = int(size * dtype.itemsize)
    # shm_name = 'test_memory'
    # shm = shared_memory.SharedMemory(create=True, size=nbytes, name=shm_name)
    # shared_array = np.ndarray(size, dtype=dtype, buffer=shm.buf)
    # shared_array[...] = 0

    # # create multiprocessing queue for log records
    # log_queue = Queue()
    # log_name = 'test_logger'
    # log_level = logging.DEBUG

    # # start the logger process
    # log_process = Process(target=logger_process, 
    #                       args=(log_name, log_queue, log_level))
    # log_process.start()

    # # create worker processes
    # workers = []
    # for number in range(4):
    #     workers.append(Process(target=worker_process, 
    #                            args=(number, log_name, log_queue, shm_name, size, dtype)))
    #     workers[number].start()
    
    # # wait for process completion
    # for w in workers:
    #     w.join()

    # # end the log queue
    # log_queue.put(None)
    # log_process.join()

    # # delete shared memory
    # shm.close()
    # shm.unlink()