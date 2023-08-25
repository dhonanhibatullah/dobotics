import tkinter as tk
import yaml


# DoboticsPybulletRobotApp Class
# This class is made for initiating the pybullet simulation through a simple tkinter-based app.
class DoboticsPybulletRobotApp:



    def __init__(self) -> None:
        # Initiate the app
        self.root   = tk.Tk()
        self.root.title('Dobotics Pybullet Simulation Start')
        self.root.geometry('400x300')
        self.root.protocol('WM_DELETE_WINDOW', self.quit)




    def fetchYaml(self) -> None:
        pass



    def quit(self) -> None:
        self.root.destroy()