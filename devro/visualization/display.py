import tkinter
from PIL import ImageTk, Image
import numpy as np
import cv2

def main(func):
    window = tkinter.Tk()
    window.title("Environment")
    window.after(100, func)
    window.mainloop()


import tkinter as tk
import threading

class Window(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        self.frame = None
        self.start()

    def callback(self):
        self.root.quit()

    def setFrame(self, img):
        self.frame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(img.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        # self.canvas.create_image(0,0, anchor='nw', image=self.frame)
        self.canvas.itemconfig(self.imageref, image = self.frame)

    def run(self):
        self.root = tk.Tk()
        self.root.title = 'Env'
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.canvas = tk.Canvas(self.root, width = 720, height = 720)
        self.canvas.pack()
        self.frame = np.zeros((720,720))
        self.frame = np.stack((self.frame,)*3, axis=-1).astype(np.uint8)
        self.frame =  ImageTk.PhotoImage(image=Image.fromarray(self.frame))
        self.imageref = self.canvas.create_image(0,0, anchor='nw', image=self.frame)

        self.root.mainloop()
