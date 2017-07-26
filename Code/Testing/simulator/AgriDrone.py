from gui import DCMainApp
import Tkinter as tk

WIN_WIDTH = 880
WIN_HEIGHT = 760


def main():
    # Initialize GUI
    root = tk.Tk()
    root.geometry("{}x{}".format(WIN_WIDTH, WIN_HEIGHT)) #GUI window dimensions
    drone_GUI = DCMainApp(root, WIN_WIDTH, WIN_HEIGHT)

    # Run GUI
    root.mainloop()


if __name__ == "__main__":
    main()
