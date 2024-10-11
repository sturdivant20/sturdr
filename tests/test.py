import multiprocessing
from multiprocessing import Process, Queue
import time

class Chef(Process):
    def __init__(self, queue: Queue, chicken: list):
        Process.__init__(self)
        self.queue = queue
        self.chicken = chicken
        
    def cook_chicken(self, chicken):
        for i in range(len(chicken)):
            chicken[i] = 1
        return chicken
    
    def run(self):
        cooked_chicken = self.cook_chicken(self.chicken)
        print("Chef is finished cooking!")
        self.queue.put(cooked_chicken)
        
if __name__ == '__main__':
    ts = time.time()
    chefs = []
    queue = Queue()
    chicken_to_cook = 4
    
    for i in range(chicken_to_cook):
        raw_chicken = [0] * 10000000
        chefs.append(Chef(queue, raw_chicken))
    
    for chef in chefs:
        chef.start()
        
    while chicken_to_cook > 0:
        cooked_chicken = queue.get() # wait for something in queue
        print(f"cooked_chicken = {cooked_chicken[59]}")
        chicken_to_cook -= 1
    print(f"All chefs finished! t = {round(time.time()-ts,3)} s")