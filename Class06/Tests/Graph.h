/*
 * Graph.h
 */
#ifndef GRAPH_H_
#define GRAPH_H_

#include "MutablePriorityQueue.h"
#include <cmath>
#include <iostream>
#include <limits>
#include <list>
#include <queue>
#include <vector>

using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

template <class T> class Vertex {
  T info;              // content of the vertex
  vector<Edge<T>> adj; // outgoing edges

  size_t id = 0;

  double dist = 0;
  Vertex<T> *path = NULL;
  int queueIndex = 0; // required by MutablePriorityQueue

  bool visited = false;    // auxiliary field
  bool processing = false; // auxiliary field

  void addEdge(Vertex<T> *dest, double w);

public:
  Vertex(T in);
  Vertex(T in, size_t id);
  T getInfo() const;
  double getDist() const;
  Vertex *getPath() const;

  bool
  operator<(Vertex<T> &vertex) const; // // required by MutablePriorityQueue
  friend class Graph<T>;
  friend class MutablePriorityQueue<Vertex<T>>;
};

template <class T> Vertex<T>::Vertex(T in) : info(in) {}
template <class T> Vertex<T>::Vertex(T in, size_t id) : info(in), id(id) {}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T> void Vertex<T>::addEdge(Vertex<T> *d, double w) {
  adj.push_back(Edge<T>(d, w));
}

template <class T> bool Vertex<T>::operator<(Vertex<T> &vertex) const {
  return this->dist < vertex.dist;
}

template <class T> T Vertex<T>::getInfo() const { return this->info; }

template <class T> double Vertex<T>::getDist() const { return this->dist; }

template <class T> Vertex<T> *Vertex<T>::getPath() const { return this->path; }

/********************** Edge  ****************************/

template <class T> class Edge {
  Vertex<T> *dest; // destination vertex
  double weight;   // edge weight
public:
  Edge(Vertex<T> *d, double w);
  friend class Graph<T>;
  friend class Vertex<T>;
};

template <class T> Edge<T>::Edge(Vertex<T> *d, double w) : dest(d), weight(w) {}

/*************************** Graph  **************************/

template <class T> class Graph {
  vector<Vertex<T> *> vertexSet; // vertex set
  vector<double> d_mtr;
  vector<Vertex<T> *> p_mtr;

public:
  Vertex<T> *findVertex(const T &in) const;
  size_t findVertexIndex(const T &in) const;
  bool addVertex(const T &in);
  bool addEdge(const T &sourc, const T &dest, double w);
  int getNumVertex() const;
  vector<Vertex<T> *> getVertexSet() const;

  void printamos() const;

  // Fp05 - single source
  void unweightedShortestPath(const T &s);  // TODO...
  void dijkstraShortestPath(const T &s);    // TODO...
  void bellmanFordShortestPath(const T &s); // TODO...
  vector<T> getPathTo(const T &dest) const; // TODO...

  // Fp05 - all pairs
  void floydWarshallShortestPath(); // TODO...
  vector<T> getfloydWarshallPath(const T &origin,
                                 const T &dest) const; // TODO...
};

template <class T> int Graph<T>::getNumVertex() const {
  return vertexSet.size();
}

template <class T> vector<Vertex<T> *> Graph<T>::getVertexSet() const {
  return vertexSet;
}

/*
 * Auxiliary function to find a vertex with a given content.
 */
template <class T> Vertex<T> *Graph<T>::findVertex(const T &in) const {
  for (auto v : vertexSet)
    if (v->info == in)
      return v;
  return NULL;
}

/*
 *  Adds a vertex with a given content or info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already
 * exists.
 */
template <class T> bool Graph<T>::addVertex(const T &in) {
  if (findVertex(in) != NULL)
    return false;
  vertexSet.push_back(new Vertex<T>(in, vertexSet.size()));
  return true;
}

/*
 * Adds an edge to a graph (this), given the contents of the source and
 * destination vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex
 * does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
  auto v1 = findVertex(sourc);
  auto v2 = findVertex(dest);
  if (v1 == NULL || v2 == NULL)
    return false;
  v1->addEdge(v2, w);
  return true;
}

/**************** Single Source Shortest Path algorithms ************/

template <class T> void Graph<T>::unweightedShortestPath(const T &orig) {
  for (Vertex<T> *vertex : vertexSet) {
    vertex->dist = std::numeric_limits<double>::max();
    vertex->path = NULL;
  }

  Vertex<T> *v;
  if ((v = findVertex(orig)) == NULL)
    return;
  v->dist = 0;
  queue<Vertex<T> *> q;
  q.push(v);

  while (!q.empty()) {
    v = q.front();
    q.pop();
    for (Edge<T> edge : v->adj) {
      if (edge.dest->dist == std::numeric_limits<double>::max()) {
        q.push(edge.dest);
        edge.dest->dist = v->dist + 1;
        edge.dest->path = v;
      }
    }
  }
}

template <class T> void Graph<T>::dijkstraShortestPath(const T &origin) {
  for (Vertex<T> *vertex : vertexSet) {
    vertex->dist = std::numeric_limits<double>::max();
    vertex->path = NULL;
    vertex->visited = false;
  }

  Vertex<T> *v;
  if ((v = findVertex(origin)) == NULL)
    return;
  v->queueIndex = 0;
  v->dist = 0;
  v->visited = true;

  MutablePriorityQueue<Vertex<T>> q;
  q.insert(v);

  while (!q.empty()) {
    v = q.extractMin();
    for (Edge<T> edge : v->adj) {
      if (edge.dest->dist > v->dist + edge.weight) {
        edge.dest->dist = v->dist + edge.weight;
        edge.dest->path = v;

        if (!edge.dest->visited) {
          edge.dest->visited = true;
          q.insert(edge.dest);
        } else
          q.decreaseKey(edge.dest);
      }
    }
  }
}

template <class T> void Graph<T>::bellmanFordShortestPath(const T &orig) {
  for (Vertex<T> *v : vertexSet) {
    v->dist = std::numeric_limits<double>::max();
    v->path = NULL;
  }

  Vertex<T> *v;
  if ((v = findVertex(orig)) == NULL)
    return;
  v->dist = 0;

  for (int i = 1; i < vertexSet.size(); ++i) {
    for (Vertex<T> *v : vertexSet) {
      for (Edge<T> edge : v->adj) {
        if (edge.dest->dist > v->dist + edge.weight) {
          edge.dest->dist = v->dist + edge.weight;
          edge.dest->path = v;
        }
      }
    }
  }

  // verify cycles of negative weight
  for (Vertex<T> *v : vertexSet)
    for (Edge<T> edge : v->adj)
      if (edge.dest->dist > v->dist + edge.weight)
        cout << "FAILURE: there are cycles of negative weigt" << endl;
}

template <class T> vector<T> Graph<T>::getPathTo(const T &dest) const {
  vector<T> res;

  Vertex<T> *v_d;
  if ((v_d = findVertex(dest)) == NULL)
    return res;
  res.push_back(v_d->info);

  while (v_d->path != NULL) {
    res.push_back(v_d->path->info);
    v_d = v_d->path;
  }

  reverse(res.begin(), res.end());
  return res;
}

/**************** All Pairs Shortest Path  ***************/

template <class T> void Graph<T>::printamos() const {
  for (size_t i = 0; i < vertexSet.size(); ++i) {
    for (size_t j = 0; j < vertexSet.size(); ++j) {
      cout << d_mtr[i * vertexSet.size() + j] << '\t';
    }
    cout << endl;
  }
  for (size_t i = 0; i < vertexSet.size(); ++i) {
    for (size_t j = 0; j < vertexSet.size(); ++j) {
      if (p_mtr[i * vertexSet.size() + j] == NULL)
        cout << 'N' << '\t';
      else
        cout << p_mtr[i * vertexSet.size() + j]->info << '\t';
    }
    cout << endl;
  }
}

template <class T> size_t Graph<T>::findVertexIndex(const T &in) const {
  size_t i;
  for (i = 0; i < vertexSet.size(); ++i)
    if (vertexSet[i]->info == in)
      break;

  return i;
}

template <class T> void Graph<T>::floydWarshallShortestPath() {
  // init
  size_t max_size = pow(vertexSet.size(), 2);
  d_mtr.reserve(max_size);
  p_mtr.reserve(max_size);
  for (size_t i = 0; i < max_size; ++i) { // init: dist. = inf. && ante. = NULL
    d_mtr[i] = std::numeric_limits<double>::max();
    p_mtr[i] = NULL;
  }
  for (Vertex<T> *v : vertexSet) {
    for (Edge<T> e : v->adj) { // for each edge
      d_mtr[v->id * vertexSet.size() + e.dest->id] = e.weight;
      p_mtr[v->id * vertexSet.size() + e.dest->id] = e.dest;
    }
    // for each vertex
    d_mtr[v->id * vertexSet.size() + v->id] = 0;
    p_mtr[v->id * vertexSet.size() + v->id] = v;
  }

  // calc
  for (size_t k = 1; k < vertexSet.size(); ++k) {
    for (size_t i = 0; i < vertexSet.size(); ++i) {
      for (size_t j = 0; j < vertexSet.size(); ++j) {
        if (d_mtr[i * vertexSet.size() + j] >
            d_mtr[i * vertexSet.size() + k] + d_mtr[k * vertexSet.size() + j]) {
          d_mtr[i * vertexSet.size() + j] =
              d_mtr[i * vertexSet.size() + k] + d_mtr[k * vertexSet.size() + j];
          p_mtr[i * vertexSet.size() + j] = p_mtr[i * vertexSet.size() + k];
        }
      }
    }
  }
}

template <class T>
vector<T> Graph<T>::getfloydWarshallPath(const T &orig, const T &dest) const {
  vector<T> res;

  size_t src_i = findVertexIndex(orig);
  size_t dest_i = findVertexIndex(dest);

  if (p_mtr[src_i * vertexSet.size() + dest_i] == NULL)
    return res;

  res.push_back(orig);
  while (src_i != dest_i) {
    src_i = p_mtr[src_i * vertexSet.size() + dest_i]->id;
    res.push_back(vertexSet[src_i]->info);
  }

  return res;
}

#endif /* GRAPH_H_ */
