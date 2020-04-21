import tkinter
import threading

class Threader (threading.Thread):
    def __init__(self, threadName, window):
        threading.Thread.__init__(self)
        self.threadName = threadName
        self.window = window

    def run(self):
        self.window.mainloop()

def main():
    window = tkinter.Tk()
    window.title("Environment")
    dispThread = Threader('environment', window)
    dispThread.start()
