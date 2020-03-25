/*
 * Graph.h
 */
#ifndef GRAPH_H_
#define GRAPH_H_

#include <queue>
#include <vector>
using namespace std;

template <class T> class Edge;
template <class T> class Graph;
template <class T> class Vertex;

/****************** Provided structures  ********************/

template <class T> class Vertex {
  T info;              // contents
  vector<Edge<T>> adj; // list of outgoing edges
  bool visited;        // auxiliary field used by dfs and bfs
  int indegree;        // auxiliary field used by topsort
  bool processing;     // auxiliary field used by isDAG

  void addEdge(Vertex<T> *dest, double w);
  bool removeEdgeTo(Vertex<T> *d);

public:
  Vertex(T in);
  friend class Graph<T>;
};

template <class T> class Edge {
  Vertex<T> *dest; // destination vertex
  double weight;   // edge weight
public:
  Edge(Vertex<T> *d, double w);
  friend class Graph<T>;
  friend class Vertex<T>;
};

template <class T> class Graph {
  vector<Vertex<T> *> vertexSet; // vertex set

  void dfsVisit(Vertex<T> *v, vector<T> &res) const;
  Vertex<T> *findVertex(const T &in) const;
  bool dfsIsDAG(Vertex<T> *v) const;

public:
  int getNumVertex() const;
  bool addVertex(const T &in);
  bool removeVertex(const T &in);
  bool addEdge(const T &sourc, const T &dest, double w);
  bool removeEdge(const T &sourc, const T &dest);
  vector<T> dfs() const;
  vector<T> bfs(const T &source) const;
  vector<T> topsort() const;
  int maxNewChildren(const T &source, T &inf) const;
  bool isDAG() const;
};

/****************** Provided constructors and functions ********************/

template <class T> Vertex<T>::Vertex(T in) : info(in) {}

template <class T> Edge<T>::Edge(Vertex<T> *d, double w) : dest(d), weight(w) {}

template <class T> int Graph<T>::getNumVertex() const {
  return vertexSet.size();
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

/****************** 1a) addVertex ********************/

/*
 *  Adds a vertex with a given content/info (in) to a graph (this).
 *  Returns true if successful, and false if a vertex with that content already
 * exists.
 */
template <class T> bool Graph<T>::addVertex(const T &in) {
  if (findVertex(in))
    return false;
  vertexSet.push_back(new Vertex<T>(in));
  return true;
}

/****************** 1b) addEdge ********************/

/*
 * Adds an edge to a graph (this), given the contents of the source (sourc) and
 * destination (dest) vertices and the edge weight (w).
 * Returns true if successful, and false if the source or destination vertex
 * does not exist.
 */
template <class T>
bool Graph<T>::addEdge(const T &sourc, const T &dest, double w) {
  Vertex<T> *src = findVertex(sourc);
  Vertex<T> *dst = findVertex(dest);
  if (!src || !dst)
    return false;
  src->addEdge(dst, w);
  return true;
}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
template <class T> void Vertex<T>::addEdge(Vertex<T> *d, double w) {
  adj.push_back(Edge<T>(d, w));
}

/****************** 1c) removeEdge ********************/

/*
 * Removes an edge from a graph (this).
 * The edge is identified by the source (sourc) and destination (dest) contents.
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T> bool Graph<T>::removeEdge(const T &sourc, const T &dest) {
  Vertex<T> *src = findVertex(sourc);
  Vertex<T> *dst = findVertex(dest);
  if (!src || !dst)
    return false;
  return src->removeEdgeTo(dst);
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
template <class T> bool Vertex<T>::removeEdgeTo(Vertex<T> *d) {
  for (auto it = adj.begin(); it != adj.end(); ++it) {
    if (it->dest->info == d->info) {
      adj.erase(it);
      return true;
    }
  }
  return false;
}

/****************** 1d) removeVertex ********************/

/*
 *  Removes a vertex with a given content (in) from a graph (this), and
 *  all outgoing and incoming edges.
 *  Returns true if successful, and false if such vertex does not exist.
 */
template <class T> bool Graph<T>::removeVertex(const T &in) {
  // TODO (10 lines)
  // HINT: use an iterator to scan the "vertexSet" vector and then erase the
  // vertex. HINT: take advantage of "removeEdgeTo" to remove incoming edges.
  Vertex<T> to_rm = Vertex<T>(in);
  bool found = false;
  for (auto it = vertexSet.begin(); it != vertexSet.end(); ++it) {
    (*it)->removeEdgeTo(&to_rm);
    if ((*it)->info == in) {
      found = true;
      it = vertexSet.erase(it);
    }
  }
  return found;
}

/****************** 2a) dfs ********************/

/*
 * Performs a depth-first search (dfs) in a graph (this).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T> vector<T> Graph<T>::dfs() const {
  // TODO (7 lines)

  for (int i = 0; i < this->vertexSet.size(); ++i)
    vertexSet[i]->visited = false;
  vector<T> res;
  for (int i = 0; i < this->vertexSet.size(); ++i)
    if (!vertexSet[i]->visited)
      dfsVisit(vertexSet[i], res);

  return res;
}

/*
 * Auxiliary function that visits a vertex (v) and its adjacent not yet visited,
 * recursively. Updates a parameter with the list of visited node contents.
 */
template <class T> void Graph<T>::dfsVisit(Vertex<T> *v, vector<T> &res) const {
  // TODO (7 lines)

  res.push_back(v->info);
  v->visited = true;
  for (int i = 0; i < v->adj.size(); ++i)
    if (!v->adj[i].dest->visited)
      dfsVisit(v->adj[i].dest, res);
}

/****************** 2b) bfs ********************/

/*
 * Performs a breadth-first search (bfs) in a graph (this), starting
 * from the vertex with the given source contents (source).
 * Returns a vector with the contents of the vertices by dfs order.
 * Follows the algorithm described in theoretical classes.
 */
template <class T> vector<T> Graph<T>::bfs(const T &source) const {
  // TODO (22 lines)
  // HINT: Use the flag "visited" to mark newly discovered vertices .
  // HINT: Use the "queue<>" class to temporarily store the vertices.

  for (Vertex<T> *vertex : vertexSet)
    vertex->visited = false;
  vector<T> res;
  Vertex<T> *dest;
  if ((dest = findVertex(source)) == NULL)
    return res;
  queue<Vertex<T> *> q;
  q.push(findVertex(source));
  q.front()->visited = true;
  res.push_back(q.front()->info);
  while (!q.empty()) {
    dest = q.front();
    q.pop();
    for (int i = 0; i < dest->adj.size(); ++i) {
      if (!dest->adj[i].dest->visited) {
        res.push_back(dest->adj[i].dest->info);
        q.push(dest->adj[i].dest);
        dest->adj[i].dest->visited = true;
      }
    }
  }

  return res;
}

/****************** 2c) toposort ********************/

/*
 * Performs a topological sorting of the vertices of a graph (this).
 * Returns a vector with the contents of the vertices by topological order.
 * If the graph has cycles, returns an empty vector.
 * Follows the algorithm described in theoretical classes.
 */

template <class T> vector<T> Graph<T>::topsort() const {
  // TODO (26 lines)

  for (Vertex<T> *vertex : vertexSet)
    vertex->indegree = 0;
  for (Vertex<T> *vertex : vertexSet)
    for (Edge<T> edge : vertex->adj)
      ++edge.dest->indegree;

  queue<Vertex<T> *> q;
  for (int i = 0; i < this->vertexSet.size(); ++i)
    if (vertexSet[i]->indegree == 0)
      q.push(vertexSet[i]);

  vector<T> res;
  Vertex<T> *v;
  while (!q.empty()) {
    v = q.front();
    q.pop();
    res.push_back(v->info);
    for (Edge<T> edge : v->adj) {
      --edge.dest->indegree;
      if (edge.dest->indegree == 0)
        q.push(edge.dest);
    }
  }

  if (res.size() != this->vertexSet.size()) // fail
    res.clear();
  return res;
}

/****************** 3a) maxNewChildren (HOME WORK)  ********************/

/*
 * Performs a breadth-first search in a graph (this), starting
 * from the vertex with the given source contents (source).
 * During the search, determines the vertex that has a maximum number
 * of new children (adjacent not previously visited), and returns the
 * contents of that vertex (inf) and the number of new children (return value).
 */

template <class T> int Graph<T>::maxNewChildren(const T &source, T &inf) const {
  // TODO (28 lines, mostly reused)
  for (Vertex<T> *vertex : vertexSet)
    vertex->visited = false;
  Vertex<T> *dest;
  if ((dest = findVertex(source)) == NULL)
    return 0;
  queue<Vertex<T> *> q;
  q.push(findVertex(source));
  q.front()->visited = true;

  int curr_max = 0, temp_max = 0;
  while (!q.empty()) {
    dest = q.front();
    q.pop();
    temp_max = 0;
    for (int i = 0; i < dest->adj.size(); ++i) {
      if (!dest->adj[i].dest->visited) {
        ++temp_max;
        q.push(dest->adj[i].dest);
        dest->adj[i].dest->visited = true;
      }
    }

    if (temp_max > curr_max) {
      curr_max = temp_max;
      inf = dest->info;
    }
  }

  return curr_max;
}

/****************** 3b) isDAG   (HOME WORK)  ********************/

/*
 * Performs a depth-first search in a graph (this), to determine if the graph
 * is acyclic (acyclic directed graph or DAG).
 * During the search, a cycle is found if an edge connects to a vertex
 * that is being processed in the the stack of recursive calls (see theoretical
 * classes). Returns true if the graph is acyclic, and false otherwise.
 */

template <class T> bool Graph<T>::isDAG() const {
  // TODO (9 lines, mostly reused)
  // HINT: use the auxiliary field "processing" to mark the vertices in the
  // stack.

  for (Vertex<T> *vertex : vertexSet) {
    vertex->visited = false;
    vertex->processing = false;
  }

  for (Vertex<T> *vertex : vertexSet)
    if (!vertex->visited)
      if (!dfsIsDAG(vertex))
        return false;

  return true;
}

/**
 * Auxiliary function that visits a vertex (v) and its adjacent not yet visited,
 * recursively. Returns false (not acyclic) if an edge to a vertex in the stack
 * is found.
 */
template <class T> bool Graph<T>::dfsIsDAG(Vertex<T> *v) const {
  // TODO (12 lines, mostly reused)

  v->visited = true;
  v->processing = true;

  for (Edge<T> edge : v->adj) {
    if (edge.dest->processing)
      return false;
    if (!edge.dest->visited)
      if (!dfsIsDAG(edge.dest))
        return false;
  }

  v->processing = false;
  return true;
}

#endif /* GRAPH_H_ */
