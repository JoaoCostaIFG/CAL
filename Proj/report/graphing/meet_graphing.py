#!/usr/bin/env python3

import matplotlib.pyplot as plt

from matplotlib.ticker import EngFormatter

numWorkers = [
    10,
    30,
    50,
    70,
    90,
    100,
    200,
    300,
    400,
    500,
    600,
    700,
    800,
    900,
    1000,
]

meetAssignTimes = [
    1.0046e07,
    3.51677e07,
    5.8615e07,
    8.47892e07,
    1.06018e08,
    1.17184e08,
    2.37843e08,
    3.49182e08,
    4.70382e08,
    5.97449e08,
    7.22752e08,
    8.40306e08,
    9.71121e08,
    1.09607e09,
    1.24083e09,
]

meetAssignTimes = [n * 1e-09 for n in meetAssignTimes]

fig = plt.figure(figsize=(16, 8))
fig.suptitle("Meeting Point Assignment Performance")

ax = fig.add_subplot()
ax.plot(numWorkers, meetAssignTimes, c="blue")
ax.set_xlabel("Number of Workers")
ax.set_ylabel("Time")
ax.set_ylim(0, None)
ax.yaxis.set_major_formatter(EngFormatter(unit="s"))
ax.grid()

fig.savefig("meetAssign_performance.png", dpi="figure", quality=80, optimize=True)
#  plt.show()
