#!/usr/bin/python3

#  f = open("o_porto.xml", "r")
f = open("o_porto_2.xml", "r")
out = open("abc.xml", "w+")
lineDict = dict()


def handleWay(f):
    line = f.readline()
    firstLine = f.tell()  # Save line

    while not line.strip().startswith("</way>"):  # Count number of nodes
        out.write(line)
        if line.strip().startswith("<nd"):
            break
        line = f.readline()

    if line.strip().startswith("</way>"):  # only one node
        return

    line2 = f.readline()
    while True:
        line1 = line2
        line2 = f.readline()
        if not line2.strip().startswith("<nd"):
            out.write(line1)
            out.write(line2)
            break
        else:
            n = [int(s) for s in line1.split('"') if s.isdigit()][0]
            if lineDict[n] > 1:
                out.write(line1)

    line = f.readline()
    while not line.strip().startswith("</way>"):  # Count number of nodes
        out.write(line)
        if line.strip().startswith("<nd"):
            break
        line = f.readline()

    out.write(line)


def addSet(f):
    line = f.readline()
    while not line.strip().startswith("</way>"):  # Count number of nodes
        if line.strip().startswith("<nd"):
            n = [int(s) for s in line.split('"') if s.isdigit()][0]
            if n not in lineDict:
                lineDict[n] = 1
            else:
                lineDict[n] += 1

        line = f.readline()


line = f.readline()
while line != "":
    #  print(line)
    if line.strip().startswith("<way"):
        addSet(f)
    line = f.readline()
print(lineDict)

line = f.seek(0)
line = f.readline()
while line != "":
    print(line)
    out.write(line)
    if line.strip().startswith("<way"):
        handleWay(f)
    line = f.readline()
