#!/usr/bin/env python3

import json
from math import tan, log, pi

origin = open("osm-graph-parser/out.json", "r")

nodef = open("../resources/porto/nodes", "w")
edgef = open("../resources/porto/edges", "w")

osm = json.load(origin)

# lat = φ
# lon = λ

mapWidth = 2e12
mapHeight = 1e12

index = 0
for node in osm:
    longitude = node["lo"]
    latitude = node["la"]

    # get x value
    x = (longitude + 180) * (mapWidth / 360)

    # convert from degrees to radians
    latRad = latitude * pi / 180

    # get y value
    mercN = log(tan((pi / 4) + (latRad / 2)))
    y = (mapHeight / 2) - (mapWidth * mercN / (2 * pi))

    nodef.write(f"{index} {x} {y}\n")
    for edge in node["e"]:
        edgef.write(f"{index} {edge['i']} {edge['w']/100.0}\n")
    index += 1
