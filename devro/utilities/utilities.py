import threading

class Scheduler():
    def __init__(self):
        self.active = True

    def setInterval(self, func, dt):
        def func_wrapper():
            if self.active is True:
                self.setInterval(func, dt)
            func()
        t = threading.Timer(dt, func_wrapper)
        t.daemon = True
        t.start()
        return t

    def setTimer(self, func, dt):
        t = threading.Timer(dt, func)
        t.daemon = True
        t.start()
        return t

    def cancel(self):
        self.active = False
