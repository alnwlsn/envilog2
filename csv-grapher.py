import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib import style
import os

def tail(f, lines=1, _buffer=4098):
    """Tail a file and get X lines from the end"""
    # place holder for the lines found
    lines_found = []

    # block counter will be multiplied by buffer
    # to get the block size from the end
    block_counter = -1

    # loop until we find X lines
    while len(lines_found) < lines:
        try:
            f.seek(block_counter * _buffer, os.SEEK_END)
        except IOError:  # either file is too small, or too many lines requested
            f.seek(0)
            lines_found = f.readlines()
            break

        lines_found = f.readlines()

        # we found enough lines, get out
        # Removed this line because it was redundant the while will catch
        # it, I left it for history
        # if len(lines_found) > lines:
        #    break

        # decrement the block counter to get the
        # next X bytes
        block_counter -= 1

    return lines_found[-lines:]

# style.use('fivethirtyeight')

fig = plt.figure()
#fig.set_figheight(5)
#fig.set_figwidth(10)
ax1 = fig.add_subplot(1,1,1)
fig.subplots_adjust(left=0.05, bottom=0.1, right=0.98, top=0.98)

def animate(i):
    file = open('tcp.txt','r')
    lines = tail(file,1000)
    x = range(-len(lines),0)
    a = []
    b = []
    c = []
    d = []
    for line in lines:
        if len(line) > 1:
            elements = line.split(',')
            a.append(float(elements[0]))
            b.append(float(elements[1]))
            c.append(float(elements[2]))
            d.append(float(elements[3]))
    ax1.clear()
    ax1.plot(x, a, label="Therm A "+str(a[-1])+"°F")
    ax1.plot(x, b, label="Therm B "+str(b[-1])+"°F")
    ax1.plot(x, c, label="Therm C "+str(c[-1])+"°F")
    ax1.plot(x, d, label="Therm D "+str(d[-1])+"°F")
    ax1.set_ylabel('Temperature (°F)')
    ax1.set_xlabel('Time (minutes)')
    ax1.legend()
    # plt.ylim([0-5, 255+5])
    # ax1.set_yticks(range(0,257,16))
    ax1.grid(True)

ani = animation.FuncAnimation(fig, animate, interval=1000)
plt.show()