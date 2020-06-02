#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.ticker import EngFormatter

tspMeetingPoints = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

tspTimes = [
    np.average([31969, 33580, 34377, 32328, 32869, 32603, 41665, 32762, 33407, 35658]),
    np.average([70714, 67996, 69947, 72937, 80505, 73957, 70843, 70545, 70623, 72859]),
    np.average(
        [121002, 124261, 122500, 126656, 125185, 125127, 123927, 123083, 122507, 120726]
    ),
    np.average(
        [205356, 232344, 201650, 208778, 189053, 188935, 198574, 191591, 197522, 198432]
    ),
    np.average(
        [268051, 269613, 271428, 270674, 266965, 303634, 281206, 268988, 271883, 272244]
    ),
    np.average(
        [365400, 369924, 377922, 368308, 375921, 375013, 363988, 364477, 362271, 364271]
    ),
    np.average(
        [484776, 487289, 484016, 484291, 501516, 504873, 492101, 531584, 474604, 476980]
    ),
    np.average(
        [628135, 604055, 614675, 602041, 610582, 601202, 605178, 634959, 609293, 603414]
    ),
    np.average(
        [765119, 748071, 771534, 760540, 753347, 818062, 769048, 781590, 754341, 761219]
    ),
    np.average(
        [913101, 936942, 920256, 924779, 909937, 960202, 918773, 956371, 929109, 911922]
    ),
]

tspTimes = [n * 1e-9 for n in tspTimes]

fig = plt.figure(figsize=(16, 8))
fig.suptitle("TSP Performance")

ax = fig.add_subplot()
ax.plot(tspMeetingPoints, tspTimes, c="blue")
ax.set_xlabel("Number of Meeting Points")
ax.set_ylabel("Time")
ax.set_ylim(0, None)
ax.yaxis.set_major_formatter(EngFormatter(unit="s"))
ax.grid()

fig.savefig("tsp_performance.png", dpi="figure", quality=80, optimize=True)
# plt.show()
