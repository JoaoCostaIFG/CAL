#ifndef PROBLEMSOLVER_H
#define PROBLEMSOLVER_H

#include <exception>
#include <string>
#include <utility>

#include "Graph.h"
#include "GraphImporter.h"
#include "MeetingPoint.h"

#define DEAD_EDGE_TYPE pair<unsigned, unsigned>
#define ASSIGNED_BUSES_TYPE vector<vector<unsigned>>

typedef struct {
  double dist;
  vector<unsigned> path;
} meetingPath;

class ProblemSolver {
private:
  GraphImporter importer;
  Graph<unsigned> *g;
  vector<Vertex<unsigned> *> workers;
  vector<DEAD_EDGE_TYPE> dead_edges;

  double d_max = 0;
  unsigned V0 = 0;
  unsigned Cn = 0;

  set<Vertex<unsigned> *> used_vertexes;
  vector<MeetingPoint> meeting_points;
  vector<Vertex<unsigned> *> bus_stops;
  vector<vector<meetingPath>> meeting_path;
  vector<unsigned> bus_capacities;

  /**
   * @brief Read config from file.
   * @param config_file Config file name.
   */
  void parseConfig(const string &config_file);
  /**
   * @brief Used by assignMeetingPoints to reset temporary/auxiliary info.
   */
  void resetUsedVertexes(void);
  /**
   * @brief parses information from the Graph g to get path and distance between
   *        the Vertexes s and t.
   */
  void calcMeetPath(meetingPath &mp, unsigned const s, unsigned const t) const;
  /**
   * @brief Get information of the Vertex with index index.
   */
  unsigned indexToInfo(const unsigned index) const;
  /**
   * @brief Get strongly connected component that contains V0 from the Graph g.
   */
  Graph<unsigned> *getConnect(void);

public:
  ProblemSolver(const string &nodes_file, const string &edges_file,
                const string &config_file)
      : importer(), g(importer.import(nodes_file, edges_file)) {
    parseConfig(config_file);
  }

  /**
   * Reads a file containing the edges that are currently inaccessible due to
   * to constructions (or other temporary problems).
   */
  void readDeadEdges(const string &dead_edges_file);
  vector<DEAD_EDGE_TYPE> &getDeadEdges(void) { return dead_edges; }
  /**
   * @brief Normalize the Graph g coordinates based on graphImporter reported
   *        maximum and minimum values.
   */
  void normalize(void);
  /**
   * @brief Get a random configuration.
   */
  void genRandConf(const size_t num_bus_stops, const size_t num_workers,
                   const double d_max);
  /**
   * @brief Import bus stops from a file.
   */
  void importBusStops(const string &bus_file);
  /**
   * @brief Import company information from a file.
   */
  void importCompany(const string &worker_file);

  /**
   * @brief Performs many operations to get the imported Graph g ready for use
   *        in the ProblemSolver. Clears empty Vertex (see Graph.h), the
   * strongly connected component, etc..
   */
  void fixGraph(void);

  Graph<unsigned> *getGraph(void) { return this->g; }
  unsigned getV0(void) { return this->V0; }
  unsigned getCn(void) { return this->Cn; }
  void setV0(unsigned V0) { this->V0 = V0; }
  void setCn(unsigned Cn) { this->Cn = Cn; }

  /**
   * @brief Converts a given path of Vertex represented by their info into a
   *        list of all the Vertexes that must be passed on the shortest path
   *        (vertexes between them).
   */
  vector<unsigned> reconstructPath(const vector<unsigned> &path);

  /**
   * @brief Alternative version of the DFS with some pre-processing used by
   *        assignMeetingPoints.
   */
  void DFS(Vertex<unsigned> *worker, Vertex<unsigned> *curr_v,
           MutablePriorityQueue<MeetingPoint> &q, unsigned total = 0);
  /**
   * @brief Assigns MeetingPoints to all worker's based on their residence,
   *        d_max and all the bus stop locations.
   */
  void assignMeetingPoints(void);

  /**
   * @brief Calculates distance and shortest path between all pairs of
   * MeetingPoints, V0 and Cn.
   */
  void calcMeetingDists(void);
  /**
   * @brief Initializes the data structures to use with TSP
   */
  size_t tspInit(vector<bool> &visited);
  /**
   * @brief Uses the Nearest Neighbor algorithm to solve the TSP (travelling
   *        salesman problem) on the graph that constains only: the assigned
   *        MeetingPoints, V0 and Cn. Solves an alternate version of the TSP
   *        that starts the tour on V0 and end on Cn.
   */
  vector<unsigned> tsp(void);
  /**
   * @brief Assigns MeetingPoints to each bus.
   */
  vector<vector<unsigned>> assignBuses(const vector<unsigned> &main_path);
  /**
   * @brief Calculate the shortest path for a bus (given the MeetingPoints
   *        assigned to it).
   */
  vector<unsigned> genBusPath(unsigned start, unsigned end,
                              const vector<unsigned> &p);

  /**
   * @brief Clear a previously calculated solution (so we can calculate a new
   *        one).
   */
  void clearSolution();
};

/**************** EXCEPTIONS ****************/
class NoPathFound : public runtime_error {
public:
  NoPathFound(const unsigned s, const unsigned t)
      : runtime_error("No path found from " + to_string(s) + " to " +
                      to_string(t)) {}
};

class NoSuchVertex : public runtime_error {
public:
  NoSuchVertex(const unsigned v)
      : runtime_error("No vertex with info " + to_string(v) +
                      " in the graph.") {}
};

class WorkersWithoutMeetingPoint : public runtime_error {
public:
  WorkersWithoutMeetingPoint(const unsigned v)
      : runtime_error(to_string(v) +
                      " workers weren't assigned to a meeting point") {}
};

#endif // PROBLEMSOLVER_H
