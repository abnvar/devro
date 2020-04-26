import cv2
import tkinter as tk
import threading
import numpy as np
from PIL import ImageTk, Image

class Window(threading.Thread):
    def __init__(self, name, height = 720, dt = 100):
        threading.Thread.__init__(self)
        self.daemon = True
        self.name = name
        self.start()
        self.height = height
        self.dt = dt
        self.envImg = None
        self.scannerImg = None
        self.envFrame = None
        self.scannerFrame = None

    def callback(self):
        self.root.quit()

    def setEnvFrame(self, img):
        self.envImg = img

    def setScannerFrame(self, img):
        self.scannerImg = img

    def refresh(self):
        self.envFrame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.envImg.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        self.canvas.itemconfig(self.envref, image = self.envFrame)
        self.scannerFrame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.scannerImg.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        self.canvas.itemconfig(self.scannerref, image = self.scannerFrame)
        self.root.update()
        self.root.after(self.dt, self.refresh)

    def run(self):
        self.root = tk.Tk()
        self.root.title(self.name)
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.canvas = tk.Canvas(self.root, width = self.height*2, height = self.height)
        self.canvas.pack()

        blank = np.zeros((self.height,self.height))
        blank = np.stack((blank,)*3, axis=-1).astype(np.uint8)

        self.envFrame = ImageTk.PhotoImage(image=Image.fromarray(blank))
        self.scannerFrame =  ImageTk.PhotoImage(image=Image.fromarray(blank))

        self.envref = self.canvas.create_image(0,0, anchor='nw', image=self.envFrame)
        self.scannerref = self.canvas.create_image(self.height,0, anchor='nw', image=self.scannerFrame)

        self.root.after(self.dt, self.refresh)
        self.root.mainloop()
