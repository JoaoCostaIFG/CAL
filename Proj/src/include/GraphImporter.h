#ifndef GRAPHIMPORTER_H
#define GRAPHIMPORTER_H

#include <exception>
#include <string>

#include "Graph.h"
#include "MeetingPoint.h"

using namespace std;

class GraphImporter {
private:
  double minX = INF, minY = INF;
  double maxX = NINF, maxY = NINF;

  void importNodes(Graph<unsigned> *g, ifstream &nodes_file);
  void importEdges(Graph<unsigned> *g, ifstream &edges_file) const;

public:
  GraphImporter() = default;
  /**
   * @brief Import a graph from files.
   * @param nodes_file_name File that contains the nodes information.
   * @param edges_file_name File that contains the edges information.
   * @return The imported graph.
   */
  Graph<unsigned> *import(const string nodes_file_name,
                          const string edges_file_name);
  /**
   * @brief Import bus stop nodes.
   * @param g Graph to import bus stops to.
   * @param busstop_file_name Name of the file that contains the bus stops.
   * @return A vector of meeting points containing the Vertexes that are bus stops.
   */
  vector<MeetingPoint> importBusStops(Graph<unsigned> *g,
                                      const string &busstop_file_name) const;

  /**
   * @brief Import a company information from a file.
   * @param g Graph to import info to.
   * @param worker_file_name Company info file.
   * @param Cn Vertex that represents location of target company.
   * @return A list of the imported workers' residence.
   */
  vector<Vertex<unsigned> *> importWorkers(Graph<unsigned> *g,
                                           const string &worker_file_name,
                                           unsigned &Cn) const;

  /**
   * Returns num random Vertexes from the graph G.
   */
  vector<Vertex<unsigned> *> getRandVert(Graph<unsigned> *g,
                                         unsigned num) const;

  double getMinX() const { return minX; }
  double getMinY() const { return minY; }
  double getMaxX() const { return maxX; }
  double getMaxY() const { return maxY; }
};

/**************** EXCEPTIONS ****************/
class NoSuchFileException : public runtime_error {
public:
  NoSuchFileException(const string file_name)
      : runtime_error("File " + file_name + " doesn't exist/isn't readale") {}
};

#endif // GRAPHIMPORTER_H
