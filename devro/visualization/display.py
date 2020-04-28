import cv2
import tkinter as tk
from tkinter import Button
import threading
import numpy as np
from PIL import ImageTk, Image

class Window(threading.Thread):

    def __init__(self, name, height = 720, dt = 100, endSimFunc = None, addObstacle  = None, toggleSimFunc = None, scale = 1):
        threading.Thread.__init__(self)
        self.daemon = True
        self.name = name
        self.start()
        self.height = height
        self.dt = dt
        self.endSimFunc = endSimFunc
        self.addObstacle = addObstacle
        self.toggleSimFunc = toggleSimFunc
        self.scale = scale

    def close(self):
        self.root.quit()
        self.root.destroy()

    def callback(self):
        self.root.quit()

    def setEnvFrame(self, img):
        self.envImg = cv2.resize(img, (int(img.shape[0]*self.scale), int(img.shape[1]*self.scale)))

    def setScannerFrame(self, img):
        self.scannerImg = cv2.resize(img, (int(img.shape[0]*self.scale), int(img.shape[1]*self.scale)))

    def setSlamFrame(self, img):
        self.slamImg = cv2.resize(img, (int(img.shape[0]*self.scale), int(img.shape[1]*self.scale)))

    def refresh(self):
        self.envFrame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.envImg.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        self.canvas1.itemconfig(self.envref, image = self.envFrame)
        self.scannerFrame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.scannerImg.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        self.canvas2.itemconfig(self.scannerref, image = self.scannerFrame)
        self.slamFrame = ImageTk.PhotoImage(image=Image.fromarray(cv2.cvtColor(self.slamImg.astype(np.uint8), cv2.COLOR_BGR2RGB)))
        self.canvas3.itemconfig(self.slamref, image = self.slamFrame)
        self.root.update()
        self.root.after(self.dt, self.refresh)

    def toggleSim(self):
        if self.toggleButton['text'] == "Pause Simulation":
            self.toggleButton.configure(text = "Start Simulation")
        else:
            self.toggleButton.configure(text = "Pause Simulation")
        self.toggleSimFunc()

    def run(self):
        self.root = tk.Tk()
        self.root.title(self.name)
        self.root.protocol("WM_DELETE_WINDOW", self.callback)

        self.canvas1 = tk.Canvas(self.root, width = self.scale*self.height, height = self.scale*self.height)
        self.canvas1.grid(row=0,column=0)
        self.canvas1.bind("<Button-1>", self.addObstacle)

        
        self.canvas2 = tk.Canvas(self.root, width = self.scale*self.height, height = self.scale*self.height)
        self.canvas2.grid(row=0,column=1)
        self.canvas3 = tk.Canvas(self.root, width = self.scale*self.height, height = self.scale*self.height)
        self.canvas3.grid(row=1,column=0)

        self.stopButton = Button(self.root, text = "End Simulation", command = self.endSimFunc, anchor = 'w')
        self.stopButton.configure(activebackground = "#33B5E5")
        self.stopButtonWindow = self.canvas2.create_window(self.scale*self.height-10, 10, anchor='ne', window=self.stopButton)

        self.toggleButton = Button(self.root, text = "Pause Simulation", command = self.toggleSim, anchor = 'w')
        self.toggleButton.configure(activebackground = "#33B5E5")
        self.toggleButtonWindow = self.canvas2.create_window(self.scale*self.height-150, 10, anchor='ne', window=self.toggleButton)

        blank_2d = np.zeros((int(self.height), int(self.height)))
        blank = np.stack((blank_2d,)*3, axis=-1).astype(np.uint8)

        self.slamImg = blank
        self.envFrame = ImageTk.PhotoImage(image=Image.fromarray(blank))
        self.scannerFrame = ImageTk.PhotoImage(image=Image.fromarray(blank))
        self.slamFrame = ImageTk.PhotoImage(image=Image.fromarray(blank))

        self.envref = self.canvas1.create_image(0,0, anchor='nw', image=self.envFrame)
        self.scannerref = self.canvas2.create_image(0,0, anchor='nw', image=self.scannerFrame)
        self.slamref = self.canvas3.create_image(0,0, anchor='nw', image=self.slamFrame)

        self.root.after(self.dt, self.refresh)
        self.root.mainloop()
