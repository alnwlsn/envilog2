from datetime import datetime
import matplotlib.pyplot as plt
import numpy as np
import datetime
x1 = np.linspace(0.0, 5.0, 100)
y1 = np.cos(2 * np.pi * x1) * np.exp(-x1)

f = open("envilog-2021-12-05-indoor.csv","r")
lines = f.readlines()
f.close()
t = []
therm1 = []
therm2 = []
press = []
humid = []
for line in lines:
    e = line.split(',')
    t.append(datetime.datetime(year=int(e[2])+2000,month=int(e[3]),day=int(e[4]),hour=int(e[5]),minute=int(e[6]),second=int(e[7])))
    therm1.append(float(e[8])*(9/5)+32) #temperature sensors, converted to deg F
    therm2.append(float(e[9])*(9/5)+32)
    press.append(float(e[10]))
    humid.append(float(e[11]))

# fig, ax = plt.subplots(figsize=(5, 3))
# fig.subplots_adjust(bottom=0.15, left=0.2)
# ax.plot(x1, y1)
# ax.set_xlabel('time [s]')
# ax.set_ylabel('Damped oscillation [V]')
# plt.show()

# import datetime
fig, ax = plt.subplots(figsize=(5, 3), tight_layout=True)
# time = [base + datetime.timedelta(days=x) for x in range(len(x1))]
ax.plot(t, therm1)
ax.plot(t, therm2)
ax.tick_params(axis='x', rotation=60)
ax.set_xlabel("Time")
ax.set_ylabel("Temperature (°F)")
ax.grid(True)
plt.show()

