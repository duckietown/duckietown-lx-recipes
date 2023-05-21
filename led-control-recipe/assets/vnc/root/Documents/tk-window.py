import tkinter as tk
import os

root = tk.Tk()

p1 = tk.PhotoImage(file="/root/.icons/duckietown.png")

# Setting icon of master window
root.iconphoto(False, p1)

# setting the windows size
root.geometry("600x200")
root.title("VLS control tool")

DB_NAME = os.getenv("VEHICLE_NAME")

# Create a frame to center the window message
frame = tk.Frame(root, width=550, height=150)
frame.grid(row=0, column=0, sticky="NW")
label = tk.Label(root, text=f"Success! LEDs updating on {DB_NAME}", font='Arial 14 bold')
label.place(relx=0.5, rely=0.5, anchor=tk.CENTER)

root.mainloop()
