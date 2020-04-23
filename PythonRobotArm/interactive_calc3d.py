# coding=utf-8
import math
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


def turn_points(points, center, angle):
    cx, cy = center
    res = []
    for px, py in points:
        dist = math.sqrt((cx - px) ** 2 + (cy - py) ** 2)
        old_angl = math.degrees(math.atan2(-(cy - py), -(cx - px)))
        new_angl = old_angl + angle
        nx = math.cos(math.radians(new_angl)) * dist + cx
        ny = -math.sin(math.radians(new_angl)) * dist + cy
        res.append((nx, ny))
    return res


def draw3lines(alpha, beta, gamma):
    for cid in canv_ids:
        canv.delete(cid)
    xl, yl = 1250, 1250
    for length, angle, col in zip([600, 400, 200], [alpha, alpha - beta, alpha - beta - gamma],
                                  ["red", "green", "blue"]):
        nx = -math.cos(math.radians(angle)) * length + xl
        ny = -math.sin(math.radians(angle)) * length + yl
        canv_ids.append(canv.create_line(xl, yl, nx, ny, fill=col, width=3))
        xl, yl = nx, ny
    points = turn_points(((xl - 20, yl - 20), (xl, yl), (xl - 20, yl + 20), (xl - 10, yl)), (xl, yl), float(omegaval))
    canv_ids.append(canv.create_polygon(*points))


def recalc():
    inp = "\\n".join(map(str, [xval, yval, zval, omegaval]))
    command = f"echo \"{inp}\" | /home/bab21/PycharmProjects/robotArm/pc_cpp/cmake-build-debug/robotArm"
    output = subprocess.getoutput(command)
    outlabel["text"] = output
    output = output.splitlines(False)
    if "nan" not in "".join(output[1:4]):
        alpha = float(output[1][output[1].index("=") + 1:-3])
        beta = float(output[2][output[2].index("=") + 1:-3])
        gamma = float(output[3][output[3].index("=") + 1:-3])
        draw3lines(alpha, beta, gamma)


xscale = tk.Scale(root, from_=-50, to=100, orient=tk.HORIZONTAL, command=xchange, length=1500)
yscale = tk.Scale(root, from_=-50, to=100, orient=tk.HORIZONTAL, command=ychange, length=1500)
zscale = tk.Scale(root, from_=-50, to=100, orient=tk.HORIZONTAL, command=zchange, length=1500)
omegascale = tk.Scale(root, from_=-50, to=50, orient=tk.HORIZONTAL, command=omegachange, length=1500)

xscale.grid(column=1, row=0)
yscale.grid(column=1, row=1)
zscale.grid(column=1, row=2)
omegascale.grid(column=1, row=3)

outlabel = tk.Label(root)
canv = tk.Canvas(width=1500, height=1500, background="#cccccc")
canv_ids = []

tk.Label(root, text="x").grid(column=0, row=0)
tk.Label(root, text="y").grid(column=0, row=1)
tk.Label(root, text="z").grid(column=0, row=2)
tk.Label(root, text="omega").grid(column=0, row=3)
outlabel.grid(column=0, row=4)
canv.grid(column=1, row=4)

for x in range(0, 1501, 10):
    for y in range(0, 1501, 10):
        canv.create_oval(x - 1, y - 1, x + 1, y + 1)

root.mainloop()
