#include <exception>
#include <fstream>
#include <iostream>
#include <sstream>

#include "include/GraphImporter.h"
#include "include/MeetingPoint.h"

using namespace std;

Graph<unsigned> *GraphImporter::import(const string nodes_file_name,
                                       const string edges_file_name) {
  ifstream nodes_file(nodes_file_name);
  if (!nodes_file.is_open())
    throw NoSuchFileException(nodes_file_name);

  ifstream edges_file(edges_file_name);
  if (!edges_file.is_open())
    throw NoSuchFileException(edges_file_name);

  Graph<unsigned> *g = new Graph<unsigned>;

  importNodes(g, nodes_file);
  importEdges(g, edges_file);

  return g;
}

void GraphImporter::importNodes(Graph<unsigned> *g, ifstream &nodes_file) {
#ifdef DEBUG_ENABLED
  size_t vertex_cnt = 0;
#endif

  string line;
  unsigned info;
  double x, y;
  while (getline(nodes_file, line)) {
    istringstream iss(line);
    iss >> info >> x >> y;

    if (x < minX)
      minX = x;
    if (y < minY)
      minY = y;
    if (x > maxX)
      maxX = x;
    if (y > maxY)
      maxY = y;

    // if (!g.addVertex(info, x, y))
    if (!g->addVertexInOrder(info, Coordinate(x, y)))
      cerr << "Couldn't add node: " << info << " " << x << " " << y << endl;
#ifdef DEBUG_ENABLED
    else
      ++vertex_cnt;
#endif
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished importing " << vertex_cnt << " nodes." << endl;
#endif
}

void GraphImporter::importEdges(Graph<unsigned> *g,
                                ifstream &edges_file) const {
#ifdef DEBUG_ENABLED
  size_t edge_cnt = 0;
#endif

  string line;
  unsigned in1, in2;
  double w;
  while (getline(edges_file, line)) {
    istringstream iss(line);
  iss >> in1 >> in2 >> w;

    // if (!g.addEdge(in1, in2))
    if (!g->addEdgeInOrder(in1, in2, w))
      cerr << "Couldn't add edge: " << in1 << " " << in2 << "." << endl;
#ifdef DEBUG_ENABLED
    else
      ++edge_cnt;
#endif
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished importing " << edge_cnt << " edges." << endl;
#endif
}

vector<MeetingPoint>
GraphImporter::importBusStops(Graph<unsigned> *g,
                              const string &busstop_file_name) const {
#ifdef DEBUG_ENABLED
  size_t bus_stop_cnt = 0;
#endif
  ifstream bus_file(busstop_file_name);
  if (!bus_file.is_open())
    throw NoSuchFileException(busstop_file_name);

  string line;
  Vertex<unsigned> *v;
  vector<MeetingPoint> bus_stops;
  unsigned in;
  while (getline(bus_file, line)) {
    istringstream iss(line);

    iss >> in;
    if ((v = g->findVertex(in)) == nullptr) {
      cerr << "Couldn't find node: " << in << "." << endl;
    } else {
      v->addType(BUSSTOP);
      bus_stops.push_back(MeetingPoint(v));
#ifdef DEBUG_ENABLED
      ++bus_stop_cnt;
#endif
    }
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished importing " << bus_stop_cnt << " bus stop locations."
       << endl;
#endif

  return bus_stops;
}

vector<Vertex<unsigned> *>
GraphImporter::importWorkers(Graph<unsigned> *g, const string &worker_file_name,
                             unsigned &Cn) const {
#ifdef DEBUG_ENABLED
  size_t worker_cnt = 0;
#endif

  ifstream worker_file(worker_file_name);
  if (!worker_file.is_open())
    throw NoSuchFileException(worker_file_name);

  worker_file >> Cn;
  worker_file.ignore(1000, '\n');
  string line;

  Vertex<unsigned> *v;
  vector<Vertex<unsigned> *> workers;
  double x, y;
  while (getline(worker_file, line)) {
    istringstream iss(line);

    iss >> x >> y;
    x = ((COORD_MAX_X - COORD_MIN_X) / (maxX - minX) *
                  (x - maxX) +
                  COORD_MAX_X);
    y = ((COORD_MAX_Y - COORD_MIN_Y) / (maxY - minY) *
                  (y - maxY) +
                  COORD_MAX_Y);
    if ((v = g->findVertex(Coordinate(x, y))) == nullptr) {
      cerr << "Couldn't find node X: " << x << " Y: " << y << endl;
    } else {
      v->addType(RESIDENCE);
      workers.push_back(v);
#ifdef DEBUG_ENABLED
      ++worker_cnt;
#endif
    }
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished importing " << worker_cnt
       << " worker's residence locations." << endl;
#endif

  return workers;
}

vector<Vertex<unsigned> *> GraphImporter::getRandVert(Graph<unsigned> *g,
                                                      unsigned num) const {
  vector<Vertex<unsigned> *> res;
  res.reserve(num);
  set<size_t> taken_numbers;
  int n;

  if (num > g->getVertexSet().size())
    num = g->getVertexSet().size();

  for (size_t i = 0; i < num; ++i) {
    do {
      n = rand() % g->getVertexSet().size();
    } while (taken_numbers.find(n) != taken_numbers.end());

    taken_numbers.insert(n);
    res.push_back(g->getVertexSet().at(n));
  }

  return res;
}
