import tkinter as tk

root = tk.Tk()

p1 = tk.PhotoImage(file="/root/.icons/duckietown.png")

# Setting icon of master window
root.iconphoto(False, p1)

# setting the windows size
root.geometry("250x150")
root.title("VLF control tool")

root.mainloop()
