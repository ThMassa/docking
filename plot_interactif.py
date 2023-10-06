import numpy as np
import matplotlib.pyplot as plt

x = np.arange(-10,10)
y = x**2

fig = plt.figure()
ax = fig.add_subplot(111)
ax.plot(x,y)

coords = []

start = None
end = None

def onclick(event):
    global ix, iy, start, end
    ix, iy = event.xdata, event.ydata
    print (f'x = {ix}, y = {iy}')

    if event.button == 1:
        start = [ix, iy]
    elif event.button == 3:
        end = [ix, iy]

    if start is not None and end is not None:
        ax.plot([start[0], end[0]], [start[1], end[1]])
    # ax.scatter(ix,iy)
    plt.pause(0.2)

    global coords
        # fig.canvas.mpl_disconnect(cid)
    return coords
cid = fig.canvas.mpl_connect('button_press_event', onclick)
plt.show()