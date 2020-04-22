# coding=utf-8
import subprocess
import tkinter as tk

root = tk.Tk()

xval, yval, zval, omegaval = 0, 0, 0, 0


def xchange(value):
    global xval
    xval = value
    recalc()


def ychange(value):
    global yval
    yval = value
    recalc()


def zchange(value):
    global zval
    zval = value
    recalc()


def omegachange(value):
    global omegaval
    omegaval = value
    recalc()


def recalc():
    inp = "\\n".join(map(str, [xval, yval, zval, omegaval]))
    command = f"echo \"{inp}\" | /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug/robotArm"
    outlabel["text"] = subprocess.getoutput(command)


xscale = tk.Scale(root, from_=-50, to=50, orient=tk.HORIZONTAL, command=xchange, length=1500)
yscale = tk.Scale(root, from_=-50, to=50, orient=tk.HORIZONTAL, command=ychange, length=1500)
zscale = tk.Scale(root, from_=-50, to=50, orient=tk.HORIZONTAL, command=zchange, length=1500)
omegascale = tk.Scale(root, from_=-50, to=50, orient=tk.HORIZONTAL, command=omegachange, length=1500)

xscale.grid(column=1, row=0)
yscale.grid(column=1, row=1)
zscale.grid(column=1, row=2)
omegascale.grid(column=1, row=3)

outlabel = tk.Label(root)

tk.Label(root, text="x").grid(column=0, row=0)
tk.Label(root, text="y").grid(column=0, row=1)
tk.Label(root, text="z").grid(column=0, row=2)
tk.Label(root, text="omega").grid(column=0, row=3)
outlabel.grid(column=0, row=4, columnspan=2)

root.mainloop()
