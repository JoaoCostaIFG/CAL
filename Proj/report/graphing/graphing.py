#!/usr/bin/env python3

import matplotlib.pyplot as plt
import numpy as np

from matplotlib.ticker import EngFormatter

full = [
    np.average(
        [
            1.38015e06,
            1.28871e06,
            1.32505e06,
            1.38922e06,
            1.37597e06,
            1.36651e06,
            1.3323e06,
            1.34485e06,
            1.33189e06,
            1.31008e06,
        ]
    ),
    np.average(
        [
            1.59086e06,
            1.62808e06,
            1.63362e06,
            1.63451e06,
            1.65111e06,
            1.75726e06,
            1.57551e06,
            1.59254e06,
            1.54744e06,
            1.63731e06,
        ]
    ),
    np.average(
        [
            1.59086e06,
            1.62808e06,
            1.63362e06,
            1.63451e06,
            1.65111e06,
            1.75726e06,
            1.57551e06,
            1.59254e06,
            1.54744e06,
            1.63731e06,
        ]
    ),
    np.average(
        [
            1.53598e06,
            1.47881e06,
            1.55526e06,
            1.4651e06,
            1.56675e06,
            1.51597e06,
            1.55785e06,
            1.47506e06,
            1.55883e06,
            1.50892e06,
        ]
    ),
]
full = [n * 1e-9 for n in full]

early = [
    np.average(
        [143020, 139634, 141917, 154373, 140789, 139922, 140852, 150091, 154093, 140549]
    ),
    np.average(
        [417478, 390941, 385598, 385000, 406854, 398426, 397343, 388423, 376561, 371064]
    ),
    np.average(
        [749674, 762201, 761847, 726036, 727502, 757013, 757049, 726959, 723564, 737577]
    ),
    np.average(
        [
            1.55573e06,
            1.46608e06,
            1.54865e06,
            1.52372e06,
            1.47104e06,
            1.51084e06,
            1.57561e06,
            1.49643e06,
            1.49641e06,
            1.5605e06,
        ]
    ),
]
early = [n * 1e-9 for n in early]

bi = [
    np.average(
        [118313, 117919, 111748, 124221, 115267, 117075, 113951, 114963, 113873, 114120]
    ),
    np.average(
        [262289, 249333, 255682, 273630, 280905, 263205, 252300, 249203, 267291, 253235]
    ),
    np.average(
        [481957, 473322, 506070, 484730, 481887, 476892, 488565, 519428, 510748, 468998]
    ),
    np.average(
        [
            1.38015e06,
            1.28871e06,
            1.32505e06,
            1.38922e06,
            1.37597e06,
            1.36651e06,
            1.3323e06,
            1.34485e06,
            1.33189e06,
            1.31008e06,
        ]
    ),
]
bi = [n * 1e-9 for n in bi]

fullNodes = [12226, 12226, 12226, 12226]
earlyNodes = [353, 2354, 5090, 12021]
biNodes = [90, 1326, 2990, 10422]

optPathDistance = [764.45, 2147.76, 3189.76, 10796.6]
optPathNodes = [9, 23, 36, 125]
nodes = [(2575, 885), (2575, 590), (2575, 2607), (2791, 1831)]


# VERSION A
# fig=plt.figure()
# ax=fig.add_subplot()
# ax.plot(fullNodes, full, c="blue", label="Full Dijkstra")
# ax.plot(earlyNodes, early, c="green", label="Early Stop Dijkstra")
# ax.plot(biNodes, bi, c="red", label="Bidirectional Dijkstra")
# ax.set_xlabel("Number of nodes covered")
# ax.set_ylabel("Time (ns)")

# VERSION B
fig, axs = plt.subplots(2, 2, figsize=(18, 18))
fig.suptitle("Dijkstra Performance")

# No. nodes -> Time
axs[0, 0].plot(optPathNodes, full, c="blue", label="Full Dijkstra")
axs[0, 0].plot(optPathNodes, early, c="green", label="Early Stop Dijkstra")
axs[0, 0].plot(optPathNodes, bi, c="red", label="Bidirectional Dijkstra")
axs[0, 0].set_xlabel("Number of nodes of optimal path")
axs[0, 0].set_ylabel("Time")
axs[0, 0].set_ylim(0, None)
axs[0, 0].yaxis.set_major_formatter(EngFormatter(unit="s"))
axs[0, 0].legend()
axs[0, 0].grid()
axs[0, 0].set_title("By number of nodes of optimal path")

# Distance -> Time
axs[0, 1].plot(optPathDistance, full, c="blue", label="Full Dijkstra")
axs[0, 1].plot(optPathDistance, early, c="green", label="Early Stop Dijkstra")
axs[0, 1].plot(optPathDistance, bi, c="red", label="Bidirectional Dijkstra")
axs[0, 1].set_xlabel("Optimal path distance")
axs[0, 1].set_ylabel("Time")
axs[0, 1].set_ylim(0, None)
axs[0, 1].yaxis.set_major_formatter(EngFormatter(unit="s"))
axs[0, 0].xaxis.set_major_formatter(EngFormatter(unit="m"))
axs[0, 1].legend()
axs[0, 1].grid()
axs[0, 1].set_title("By distance of optimal path")

# No. nodes -> Nodes analyzed
axs[1, 0].plot(optPathNodes, fullNodes, c="blue", label="Full Dijkstra")
axs[1, 0].plot(optPathNodes, earlyNodes, c="green", label="Early Stop Dijkstra")
axs[1, 0].plot(optPathNodes, biNodes, c="red", label="Bidirectional Dijkstra")
axs[1, 0].set_xlabel("Number of nodes of optimal path")
axs[1, 0].set_ylabel("Number of nodes analyzed")
axs[1, 0].set_ylim(0, None)
axs[1, 0].ticklabel_format(axis="y", style="plain")
axs[1, 0].legend()
axs[1, 0].grid()

# Distance -> Nodes analyzed
axs[1, 1].plot(optPathDistance, fullNodes, c="blue", label="Full Dijkstra")
axs[1, 1].plot(optPathDistance, earlyNodes, c="green", label="Early Stop Dijkstra")
axs[1, 1].plot(optPathDistance, biNodes, c="red", label="Bidirectional Dijkstra")
axs[1, 1].set_xlabel("Optimal path distance")
axs[1, 1].set_ylabel("Number of nodes analyzed")
axs[1, 1].set_ylim(0, None)
axs[1, 1].xaxis.set_major_formatter(EngFormatter(unit="m"))
axs[1, 1].legend()
axs[1, 1].grid()

fig.savefig("dijkstra_performance.png", dpi="figure", quality=80, optimize=True)
# plt.show()
