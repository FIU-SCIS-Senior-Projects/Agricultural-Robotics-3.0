from gui import GUI
from Tkinter import Tk

WIN_WIDTH = 880
WIN_HEIGHT = 760

def main():
    root = Tk()
    root.title("D.F.C. - Drone Flight Controller")
    root.geometry("{}x{}".format(WIN_WIDTH, WIN_HEIGHT))
    drone_GUI = GUI(root, WIN_WIDTH, WIN_HEIGHT)
    root.mainloop()

if __name__ == "__main__":
    main()
