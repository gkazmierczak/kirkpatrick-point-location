import tkinter as tk
from tkinter.filedialog import askopenfilename
from kirkpatrick import Kirkpatrick


def chooseFile():
    filename = askopenfilename()
    locator = Kirkpatrick(filename, True)
    locator.pickLocatePoint()
    root.quit()


def startFromScratch():
    locator = Kirkpatrick(stepVisualization=True)
    locator.pickLocatePoint()
    root.quit()


root = tk.Tk()
root.title("Kirkpatrick")
tk.Label(root,
         text="Select option").grid(row=0)

tk.Button(root, text='Load from file', command=chooseFile).grid(
    row=1, column=0, pady=6, padx=10)
tk.Button(root, text='Start from scratch', command=startFromScratch).grid(
    row=1, column=1, pady=6, padx=10)

if __name__ == "__main__":
    root.protocol("WM_DELETE_WINDOW", exit)
    tk.mainloop()
