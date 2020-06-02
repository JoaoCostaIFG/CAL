#!/usr/bin/env python3

import osmium as osm


class OSMHandler(osm.SimpleHandler):
    nodef = open("nodef", "w+")
    edgef = open("edgef", "w+")
    edge_set = set()
    node_set = set()

    def __init__(self):
        osm.SimpleHandler.__init__(self)
        self.osm_data = []

    def is_highway(self, tags):
        return "highway" in tags

    def is_edge(self, tags):
        if not self.is_highway(tags):
            return False

        val = tags["highway"]
        return (
            val == "motorway"
            or val == "trunk"
            or val == "primary"
            or val == "secondary"
            or val == "tertiary"
        )

    min_id = 99999999999

    def node(self, n):
        if self.is_highway(n.tags):
            if n.id < self.min_id:
                self.min_id = n.id
            self.node_set.add(f"{n.id} {n.location.x} {n.location.y}\n")

    def way(self, w):
        if not self.is_edge(w.tags):
            return

        is_oneway = "oneway" in w.tags and w.tags["oneway"] == "yes"

        for i in range(1, len(w.nodes)):
            self.edge_set.add(f"{w.nodes[i-1]} {w.nodes[i]}\n")
            if not is_oneway:
                self.edge_set.add(f"{w.nodes[i]} {w.nodes[i-1]}\n")

    def relation(self, r):
        return


osmhandler = OSMHandler()
# scan the input file and fills the handler list accordingly
osmhandler.apply_file("porto_new.xml")

writen_nodes = set()
for node in osmhandler.node_set:
    node_decomp = node.split()

    new_id = (int)(node_decomp[0])
    writen_nodes.add(new_id)
    new_id -= osmhandler.min_id

    osmhandler.nodef.write(f"{new_id} {node_decomp[1]} {node_decomp[2]}\n")

for edge in osmhandler.edge_set:
    edge_decomp = edge.split()
    new_id1 = (int)(edge_decomp[0])
    new_id2 = (int)(edge_decomp[1])

    if new_id1 in writen_nodes and new_id2 in writen_nodes:
        new_id1 -= osmhandler.min_id
        new_id2 -= osmhandler.min_id
        osmhandler.edgef.write(f"{new_id1} {new_id2}\n")

osmhandler.nodef.close()
osmhandler.edgef.close()
