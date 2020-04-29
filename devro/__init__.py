__version__ = '0.0.3'

import time
import threading
def checkOverload():
    while True:
        if len(threading.enumerate()) > 10:
            raise RuntimeError('System overload, degraded performance.\n Please use setInterval at lower frequency to avoid this.')
        time.sleep(1)

t = threading.Thread(target=checkOverload)
t.daemon = True
t.start()
