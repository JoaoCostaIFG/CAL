#ifndef GRAPH_H
#define GRAPH_H

#include <algorithm>
#include <iostream>
#include <limits>
#include <stack>
#include <unordered_set>
#include <vector>

#include "Coordinate.h"
#include "MutablePriorityQueue.h"

#define COORD_MIN_X -6000
#define COORD_MAX_X 6000
#define COORD_MIN_Y -6000
#define COORD_MAX_Y 6000

using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

#define INF std::numeric_limits<double>::max()
#define NINF std::numeric_limits<double>::min()

/************************* Vertex  **************************/

enum VertexType {
  // NORMAL_VERTEX Set empty => Normal Vertex
  BUSSTOP,
  RESIDENCE,
  COMPANY,
  GARAGE,
  OBSTRUCTION,
  MEETINGPOINT,
};

template <class T> class Vertex {
  T info; // contents

  bool visited = false;    // auxiliary field
  bool bi_visited = false; // Bi-directional Dijkstra
  double dist = 0;
  Vertex<T> *path = nullptr;
  vector<Edge<T> *> adj; // outgoing edges
  vector<Edge<T> *> inc; // incoming edges
  int queueIndex = 0;    // required by MutablePriorityQueue

  unordered_set<VertexType> types;
  Coordinate coord;

  /**
   * @brief Auxiliary function to add an outgoing edge to a vertex (this),
   *        with a given destination vertex (d) and edge weight (w).
   */
  void addEdge(Vertex<T> *dest, double w, int edgeId);

public:
  Vertex(T in) : info(in), coord() {}
  Vertex(T in, double x, double y) : info(in), coord(x, y) {}
  Vertex(T in, Coordinate coord) : info(in), coord(coord) {}

  /**
   * @brief  Get the info of this vertex.
   * @return The info of the vertex.
   */
  T getInfo() const { return this->info; }
  /**
   * @brief   Get the dist (distance to the origin of a shortest path algorithm)
   *          of this vertex.
   * @return  The current dist field.
   */
  double getDist() const { return this->dist; }
  /**
   * @brief   Get the previous Vertex on a calculated path.
   * @return  NULL, if this Vertex is the origin of the path,
   *          the previous Vertex on the path, otherwise.
   */
  Vertex *getPath() const { return this->path; }
  /**
   * @brief   Get the list of outgoing adjacent edges to this Vertex.
   * @return  The list of outgoing adjacent edges to this Vertex.
   */
  vector<Edge<T> *> getAdj(void) const { return this->adj; }
  /**
   * @brief   Get the list of incoming adjacent edges of this Vertex.
   * @return  The list of incoming adjacent edges of this Vertex.
   */
  vector<Edge<T> *> getInc(void) const { return this->inc; }

  unordered_set<VertexType> getTypes(void) const { return this->types; }
  /**
   * @brief   Tag this Vertex with a new type (also keeps old tags).
   * @param type The new type to tag the Vertex with.
   */
  void addType(VertexType type) { this->types.insert(type); }
  void removeType(VertexType type) { this->types.erase(type); }
  bool containsType(VertexType type) { return this->types.find(type) != this->types.end(); }

  void setCoords(Coordinate coord) { this->coord = coord; }
  Coordinate getCoords(void) const { return this->coord; }

  bool operator<(Vertex<T> &vertex) const; // required by MutablePriorityQueue

  friend class Graph<T>;
  friend class MutablePriorityQueue<Vertex<T>>;
  friend class ProblemSolver;
};

template <class T> void Vertex<T>::addEdge(Vertex<T> *d, double w, int edgeId) {
  Edge<T> *e = new Edge<T>(this, d, w, edgeId);
  adj.push_back(e);
  d->inc.push_back(e);
}

template <class T> bool Vertex<T>::operator<(Vertex<T> &vertex) const {
  return this->dist < vertex.dist;
}

/********************** Edge  ****************************/

enum TypeEdge {
  NORMAL_EDGE,
  ROUTE,
};

template <class T> class Edge {
  Vertex<T> *orig; // TODO bydirectional dijkstra
  Vertex<T> *dest;
  double weight;
  int edgeId;
  TypeEdge type = NORMAL_EDGE;

public:
  Edge(Vertex<T> *o, Vertex<T> *d, double w, unsigned edgeId)
      : orig(o), dest(d), weight(w), edgeId(edgeId) {}

  double getWeight() const { return weight; }
  Vertex<T> *getDest(void) const { return this->dest; }

  TypeEdge getType(void) const { return this->type; }
  /**
   * @brief Tag an edge with a type. Old type is cleared.
   * @param type New type to tag edge with.
   */
  void setType(TypeEdge type) { this->type = type; }

  int getEdgeId() const { return this->edgeId; }

  friend class Graph<T>;
  friend class Vertex<T>;
};

/*************************** Graph  **************************/

template <class T> class Graph {
  int edgeIdCounter = 0;
  vector<Vertex<T> *> vertexSet;
  /**
   * Initializes single source shortest path data (path, dist).
   * Receives the content of the source vertex and returns a pointer to the
   * source vertex. Used by all single-source shortest path algorithms.
   */
  Vertex<T> *initSingleSource(const T &orig);
  /**
   * Analyzes an edge in single source shortest path algorithm.
   * Returns true if the target vertex was relaxed (dist, path).
   * Used by all single-source shortest path algorithms.
   */
  bool relax(Vertex<T> *v, Vertex<T> *w, double weight);
  /**
   * @brief   Get index in vertexSet of Vertex with info in.
   * @param in  The info of the Vertex to search.
   * @return  The index (-1 if not found).
   */
  int findVertexIdx(const T &in) const;
  /**
   * Initializes double source shortest path data (path, dist, visited).
   * Receives the content of the source vertex and returns a pointer to the
   * source vertex. Used by the bidirectional Dijkstra algorithm.
   */
  bool initBidirectionalDijkstra(const T &origin, const T &dest, Vertex<T> *&s,
                                 Vertex<T> *&d);
  /**
   * Fixes the path of calculated by the bidirectional Dijkstra. Is always ran
   * after a successful Bidirectional Dijkstra.
   */
  bool dijkstraBidirectionalFixPath(Vertex<T> *midPoint, Vertex<T> *nextDest,
                                    double weight);

public:
  ~Graph() {
    for (Vertex<T> *v : this->vertexSet) {
      for (Edge<T> *e : v->adj)
        delete e;
      delete v;
    }
  }
  vector<Vertex<T> *> getVertexSet() const { return vertexSet; }

  Vertex<T> *findVertex(const T &in) const;
  Vertex<T> *findVertex(const Coordinate &c) const;
  bool addVertex(const T &in);
  bool addVertex(const T &in, double x, double y);
  bool addVertex(const T &in, Coordinate coord);
  bool addVertexInOrder(const T &in, Coordinate coord);

  bool addEdge(const T &sourc, const T &dest);
  bool addEdge(const T &sourc, const T &dest, double w);
  bool addEdge(Vertex<T> *v1, Vertex<T> *v2, double w);
  bool addEdgeInOrder(const T &sourc, const T &dest, double w);

  /**
   * @brief Generates the inverted graph of this graph containing only
   *        the given Vertexes.
   * @param vertexes The Vertexes that will be part of the graph.
   * @return The inverted graph.
   */
  Graph<T> *genInvertedGraph(vector<Vertex<T> *> &vertexes) const;
  /**
   * @brief Remove Vertexes that have no incoming or outgoing edges.
   */
  void clearEmptyVert(void);
  /**
   * @brief Normalizes the graph coordinates to the range defined by
   *        the following constants: COORD_MIN_X, COORD_MAX_X,
   *        COORD_MIN_Y, COORD_MAX_Y.
   * @param minX The minimum X coordinate value of graph Vertexes.
   * @param minY The minimum Y coordinate value of graph Vertexes.
   * @param maxX The maximum X coordinate value of graph Vertexes.
   * @param maxY The maximum Y coordinate value of graph Vertexes.
   */
  void normalizeCoords(double minX, double minY, double maxX, double maxY);

  /**
   * @brief Depth first search of the graph.
   * @param source  The vertex to start the search on.
   * @return The list of Vertexes reached by the DFS.
   */
  vector<Vertex<T> *> DFS(T const source) const;
  /**
   * Dijkstra shortest path starting on s and going to all reachable Vertexes
   * from s (full dijkstra).
   */
  bool dijkstraShortestPath(const T &s);
  /**
   * Dijkstra shortest path starting on s, going to all reachable Vertexes
   * from s and stopping after dest is reached (or there are no more Vertexes
   * to process) (early stop dijkstra).
   */
  bool dijkstraShortestPath(const T &origin, const T &dest);
  /**
   * Dijkstra shortest path starting on 2 sources: origin and dest. Stops
   * successfully after the 2 paths meet "in the middle". Otherwise,
   * processes all reachable Vertexes from both sources and returns a
   * failed state.
   */
  bool dijkstraBidirectional(const T &origin, const T &dest);
  /**
   * Get path from one vertex to another.
   */
  vector<T> getPath(const T &origin, const T &dest) const;
  /**
   * Get path from one vertex to another and the distance between them.
   */
  vector<T> getPath(const T &origin, const T &dest, double *d) const;
};

/**************** CLEAN-UP GRAPH ****************/
template <class T>
Graph<T> *Graph<T>::genInvertedGraph(vector<Vertex<T> *> &vertexes) const {
  Graph<T> *invertedG = new Graph<T>;

  for (Vertex<T> *v : vertexes)
    invertedG->addVertex(v->info, v->coord);

  for (Vertex<T> *v : vertexes) {
    for (Edge<T> *e : v->adj) {
      if (invertedG->findVertex(e->getDest()->info) != nullptr)
        invertedG->addEdge(e->getDest()->info, v->info, e->getWeight());
    }
  }

  return invertedG;
}

template <class T> void Graph<T>::clearEmptyVert(void) {
  for (auto it = vertexSet.begin(); it!=vertexSet.end();) {
    if ((*it)->adj.empty() && (*it)->inc.empty()) {
      delete (*it);
      it = vertexSet.erase(it);
    }
    else
      ++it;
  }
}

template <class T>
void Graph<T>::normalizeCoords(double minX, double minY, double maxX,
                               double maxY) {
  for (Vertex<T> *v : this->vertexSet) {
    v->coord.setX((COORD_MAX_X - COORD_MIN_X) / (maxX - minX) *
                      (v->coord.getX() - maxX) +
                  COORD_MAX_X);
    v->coord.setY((COORD_MAX_Y - COORD_MIN_Y) / (maxY - minY) *
                      (v->coord.getY() - maxY) +
                  COORD_MAX_Y);
  }
}

/**************** FIND VERTEX ****************/
template <class T> Vertex<T> *Graph<T>::findVertex(const T &in) const {
  for (auto v : vertexSet)
    if (v->info == in)
      return v;
  return nullptr;
}

template <class T> Vertex<T> *Graph<T>::findVertex(const Coordinate &c) const {
  double min_dist = INF;
  Vertex<T> *res = nullptr;

  for (auto v : vertexSet) {
    double dist = c.dist(v->coord);
    if (dist == 0) {
      return v;
    } else if (dist < min_dist) {
      min_dist = dist;
      res = v;
    }
  }

  return res;
}

template <class T> int Graph<T>::findVertexIdx(const T &in) const {
  for (unsigned i = 0; i < vertexSet.size(); i++)
    if (vertexSet[i]->info == in)
      return i;
  return -1;
}

/**************** Add Vertex ****************/
template <class T> bool Graph<T>::addVertex(const T &in) {
  if (findVertex(in) != nullptr)
    return false;
  vertexSet.push_back(new Vertex<T>(in));
  return true;
}

template <class T> bool Graph<T>::addVertex(const T &in, double x, double y) {
  if (findVertex(in) != nullptr)
    return false;
  vertexSet.push_back(new Vertex<T>(in, x, y));
  return true;
}

template <class T> bool Graph<T>::addVertex(const T &in, Coordinate coord) {
  if (findVertex(in) != nullptr)
    return false;
  vertexSet.push_back(new Vertex<T>(in, coord));
  return true;
}

template <class T>
bool Graph<T>::addVertexInOrder(const T &in, Coordinate coord) {
  if (in > vertexSet.size())
    return false;
  vertexSet.push_back(new Vertex<T>(in, coord));
  return true;
}

/**************** Add Edge ****************/
template <class T> bool Graph<T>::addEdge(const T &sourc, const T &dest) {
  auto v1 = findVertex(sourc);
  auto v2 = findVertex(dest);
  if (v1 == nullptr || v2 == nullptr)
    return false;

  double w = v1->coord.dist(v2->coord);
  return addEdge(v1, v2, w);
}

template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
  auto v1 = findVertex(sourc);
  auto v2 = findVertex(dest);
  return addEdge(v1, v2, w);
}

template <class T>
bool Graph<T>::addEdge(Vertex<T> *v1, Vertex<T> *v2, double w) {
  if (v1 == nullptr || v2 == nullptr)
    return false;
  v1->addEdge(v2, w, ++edgeIdCounter);
  return true;
}

template <class T>
bool Graph<T>::addEdgeInOrder(const T &sourc, const T &dest, double w) {
  if (sourc >= vertexSet.size() || dest >= vertexSet.size())
    return false;
  return addEdge(vertexSet[sourc], vertexSet[dest], w);
}

/**************** DFS ****************/
template <class T> vector<Vertex<T> *> Graph<T>::DFS(T const source) const {
  vector<Vertex<T> *> res;

  Vertex<T> *s = findVertex(source);
  if (s == nullptr)
    return res;

  for (Vertex<T> *v : vertexSet)
    v->visited = false;

  stack<Vertex<T> *> stack;
  stack.push(s);
  while (!stack.empty()) {
    auto v = stack.top();
    stack.pop();

    if (!v->visited) {
      res.push_back(v);
      v->visited = true;
    }

    for (Edge<T> *e : v->adj) {
      if (!e->dest->visited)
        stack.push(e->dest);
    }
  }

  return res;
}

/**************** Single Source Shortest Path algorithms ************/

template <class T> Vertex<T> *Graph<T>::initSingleSource(const T &origin) {
  for (auto v : vertexSet) {
    v->dist = INF;
    v->path = nullptr;
  }

  auto s = findVertex(origin);
  if (s == nullptr)
    return nullptr;

  s->dist = 0;
  return s;
}

template <class T>
inline bool Graph<T>::relax(Vertex<T> *v, Vertex<T> *w, double weight) {
  if (v->dist + weight < w->dist) {
    w->dist = v->dist + weight;
    w->path = v;
    return true;
  }
  return false;
}

template <class T> bool Graph<T>::dijkstraShortestPath(const T &origin) {
  auto s = initSingleSource(origin);
  if (s == nullptr)
    return false;

  MutablePriorityQueue<Vertex<T>> q;
  q.insert(s);
  Vertex<T> *v;
  while (!q.empty()) {
    v = q.extractMin();
    for (Edge<T> *e : v->adj) {
      auto oldDist = e->dest->dist;
      if (relax(v, e->dest, e->weight)) {
        if (oldDist == INF)
          q.insert(e->dest);
        else
          q.decreaseKey(e->dest);
      }
    }
  }

  return true;
}

template <class T>
bool Graph<T>::dijkstraShortestPath(const T &origin, const T &dest) {
  auto s = initSingleSource(origin);
  if (s == nullptr || findVertex(dest) == nullptr)
    return false;

  bool touched_dest = false;
  MutablePriorityQueue<Vertex<T>> q;
  q.insert(s);
  Vertex<T> *v = nullptr;
  while (!q.empty() && (v == nullptr || v->info != dest)) {
    v = q.extractMin();
    if (v->info == dest)
      touched_dest = true;

    for (Edge<T> *e : v->adj) {
      auto oldDist = e->dest->dist;
      if (relax(v, e->dest, e->weight)) {
        if (oldDist == INF)
          q.insert(e->dest);
        else
          q.decreaseKey(e->dest);
      }
    }
  }

  return touched_dest;
}

template <class T>
bool Graph<T>::initBidirectionalDijkstra(const T &origin, const T &dest,
                                         Vertex<T> *&s, Vertex<T> *&d) {
  for (Vertex<T> *v : vertexSet) {
    v->dist = INF;
    v->visited = false;
    v->bi_visited = false;
    v->path = nullptr;
  }

  s = findVertex(origin);
  if (s == nullptr)
    return false;

  d = findVertex(dest);
  if (d == nullptr)
    return false;

  s->dist = 0;
  d->dist = 0;
  return true;
}

template <class T>
bool Graph<T>::dijkstraBidirectionalFixPath(Vertex<T> *midPoint,
                                            Vertex<T> *nextDest,
                                            double weight) {
  double oldDist;
  double w = weight;
  Vertex<T> *next = nextDest->path;

  do {
    oldDist = nextDest->dist;
    nextDest->dist = midPoint->dist + w;
    next = nextDest->path;
    nextDest->path = midPoint;

    midPoint = nextDest;
    nextDest = next;
    if (nextDest != nullptr)
      w = oldDist - nextDest->dist;
  } while (next != nullptr);

  return true;
}

template <class T>
bool Graph<T>::dijkstraBidirectional(const T &origin, const T &dest) {
  Vertex<T> *s = nullptr, *d = nullptr;

  if (!initBidirectionalDijkstra(origin, dest, s, d))
    return false;

  // if (origin == dest)
  // return true;

  MutablePriorityQueue<Vertex<T>> sq;
  sq.insert(s);
  MutablePriorityQueue<Vertex<T>> dq;
  dq.insert(d);

  Vertex<T> *sTop = nullptr, *dTop = nullptr;
  while (!sq.empty() && !dq.empty()) {
    sTop = sq.extractMin();
    sTop->visited = true;
    for (Edge<T> *e : sTop->adj) {
      if (e->dest->bi_visited) { // Found path
        return dijkstraBidirectionalFixPath(sTop, e->dest, e->weight);
      }
      if (relax(sTop, e->dest, e->weight)) {
        if (!e->dest->visited) {
          sq.insert(e->dest);
          e->dest->visited = true;
        } else
          sq.decreaseKey(e->dest);
      }
    }

    dTop = dq.extractMin();
    dTop->bi_visited = true;
    for (Edge<T> *e : dTop->inc) {
      if (e->orig->visited) { // Found path
        return dijkstraBidirectionalFixPath(e->orig, dTop, e->weight);
      }
      if (relax(dTop, e->orig, e->weight)) {
        if (!e->orig->bi_visited) {
          dq.insert(e->orig);
          e->orig->bi_visited = true;
        } else
          dq.decreaseKey(e->orig);
      }
    }
  }

  return false;
}

template <class T>
vector<T> Graph<T>::getPath(const T &origin, const T &dest) const {
  vector<T> res;
  auto v = findVertex(dest);
  auto s = findVertex(origin);

  // missing or disconnected
  if (s == nullptr || v == nullptr || v->dist == INF || v->info == s->info)
    return res;

  for (; v != nullptr; v = v->path) {
    res.push_back(v->info);
    if (v->info == s->info)
      break;
  }
  reverse(res.begin(), res.end());
  return res;
}

template <class T>
vector<T> Graph<T>::getPath(const T &origin, const T &dest, double *d) const {
  *d = INF;
  vector<T> res;
  auto v = findVertex(dest);
  auto s = findVertex(origin);

  // missing or disconnected
  if (s == nullptr || v == nullptr || v->dist == INF || v->info == s->info)
    return res;

  *d = v->dist;

  for (; v != nullptr; v = v->path) {
    res.push_back(v->info);
    if (v->info == s->info)
      break;
  }
  reverse(res.begin(), res.end());
  return res;
}

#endif // GRAPH_H
