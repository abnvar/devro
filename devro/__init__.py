__version__ = '0.0.3'

import time
import threading
def checkOverload(maxThreads = 10):
    while True:
        if len(threading.enumerate()) > maxThreads:
            raise RuntimeError('System overload, degraded performance.\n Please use setInterval at lower frequency to avoid this.')
        time.sleep(1)

t = threading.Thread(target=checkOverload)
t.daemon = True
t.start()
