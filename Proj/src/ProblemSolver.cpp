#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>

#include "include/ProblemSolver.h"

using namespace std;

/**************** IMPORT ****************/
void ProblemSolver::normalize(void) {
  g->normalizeCoords(importer.getMinX(), importer.getMinY(), importer.getMaxX(),
                     importer.getMaxY());
#ifdef DEBUG_ENABLED
  cerr << "Finished normalizing coordinates. X: (" << importer.getMinX() << "; "
       << importer.getMaxX() << "). Y: (" << importer.getMinY() << "; "
       << importer.getMaxY() << ")." << endl;
#endif
}

void ProblemSolver::genRandConf(const size_t num_bus_stops,
                                const size_t num_workers, const double d_max) {
  // Bus Stops
  this->meeting_points.clear();
  for (Vertex<unsigned> *v : importer.getRandVert(g, num_bus_stops)) {
    v->addType(BUSSTOP);
    this->meeting_points.push_back(MeetingPoint(v));
  }

  // Workers
  this->workers = importer.getRandVert(g, num_workers);
  this->d_max = d_max;
}

void ProblemSolver::parseConfig(const string &config_file) {
  ifstream config(config_file);
  if (!config.is_open())
    throw NoSuchFileException(config_file);

  config >> this->d_max;
  config.ignore(1000, '\n');
  config >> this->V0;

  Vertex<unsigned int> *v = g->findVertex(this->getV0());
  if (v == nullptr) {
    cerr << "Failed to find the Garage's (source) vertex." << endl;
    throw NoSuchVertex(this->getV0());
  }
  v->addType(GARAGE);

  unsigned cap;
  config >> cap;
  config.ignore(1000, '\n');
  while (!config.eof()) {
    this->bus_capacities.push_back(cap);
    config >> cap;
    config.ignore(1000, '\n');
  }

  config.close();

#ifdef DEBUG_ENABLED
  cerr << "Finished parsing config." << endl;
#endif
}

void ProblemSolver::readDeadEdges(const string &dead_edges_file) {
#ifdef DEBUG_ENABLED
  size_t num_dead_edges = 0;
#endif

  ifstream deadedges_fstream(dead_edges_file);
  if (!deadedges_fstream.is_open()) {
    cerr << "The imported graph has no dead edges." << endl;
    return;
    // throw NoSuchFileException(dead_edges_file);
  }

  Vertex<unsigned> *src, *dest;
  string line;
  unsigned info_src, info_dest;
  while (getline(deadedges_fstream, line)) {
    istringstream iss(line);
    iss >> info_src >> info_dest;

    this->dead_edges.emplace_back(info_src, info_dest);

    if ((src = g->findVertex(info_src)) == nullptr ||
        (dest = g->findVertex(info_dest)) == nullptr) {
      cerr << "No Such edge to mark as dead: " << info_src << " -> "
           << info_dest << endl;
      continue;
    }

    /* erase edge in outgoing way */
    for (auto it = src->adj.begin(); it != src->adj.end(); ++it) {
      if ((*it)->getDest()->info == info_dest) {
        delete (*it);
        src->adj.erase(it);
        break;
      }
    }

    /* erase edge in incoming way */
    for (auto it = dest->inc.begin(); it != dest->inc.end(); ++it) {
      if ((*it)->getDest()->info == info_src) {
        delete (*it);
        dest->inc.erase(it);
        break;
      }
    }
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished reading " << num_dead_edges << " dead edges." << endl;
#endif
}

void ProblemSolver::importBusStops(const string &bus_file) {
  this->meeting_points = importer.importBusStops(g, bus_file);
  for (MeetingPoint mp : this->meeting_points)
    this->bus_stops.push_back(mp.getVertex());
}

void ProblemSolver::importCompany(const string &worker_file) {

  this->workers = importer.importWorkers(g, worker_file, this->Cn);

  Vertex<unsigned int> *v = g->findVertex(this->Cn);
  if (v == nullptr) {
    cerr << "Failed to find the Company's (destination) vertex." << endl;
    throw NoSuchVertex(this->Cn);
  }
  v->addType(COMPANY);
}

/**************** CLEANUP ****************/
Graph<unsigned> *ProblemSolver::getConnect(void) {
  auto dfsRes = g->DFS(V0);
  auto invG = g->genInvertedGraph(dfsRes);

  auto invDfsRes = invG->DFS(V0);
  auto fortG = invG->genInvertedGraph(invDfsRes);

  delete invG;
  return fortG;
}

void ProblemSolver::fixGraph(void) {
  /*
  - Pesquisa em profundidade no grafo G determina floresta de expansão,
  numerando vértices em pós-ordem => V0 vai ser o highest num
  - Inverter todas as arestas de G => Gr
  - Segunda pesquisa em profundidade, em Gr, começando sempre pelo
  vértice de numeração mais alta ainda não visitado => 1st é V0
  - Cada árvore obtida é um componente fortemente conexo, i.e., a partir
  de um qualquer dos nós pode chegar-se a todos os outros
  - Now also normalizes coords cuz yeah

  Conclusão:
  DFS (sem forcar visit a todos os nos) em g comecando em V0. Inverter g =>
  invG. DFS (again, sem forcar visit a todos os nos) em ingG comecando em V0.
  => grafo obtido é fortemente conexo e contem a garagem (V0).
  Note: can verify logo se Cn pertence a esse new grafo e dar exception se n
  pertencer.
  */
  this->normalize();

  Graph<unsigned> *fortG = getConnect();
  delete this->g;
  this->g = fortG;

  auto garage = g->findVertex(V0);
  if (garage == nullptr)
    throw NoSuchVertex(V0);
  garage->addType(GARAGE);

  // delete verts with no connections
  g->clearEmptyVert();

  // erase deleted dead edges
  for (auto it = dead_edges.begin(); it != dead_edges.end();) {
    if (g->findVertex((*it).first) == nullptr || g->findVertex((*it).second) == nullptr)
      it = dead_edges.erase(it);
    else
      ++it;
  }
}

/**************** ASSIGN MEETING POINTS ****************/
void ProblemSolver::resetUsedVertexes(void) {
  for (Vertex<unsigned> *v : this->used_vertexes) {
    v->visited = false;
    // v->dist = 0; TODO Vamos precisar de dar reset a isto?
    // v->path = NULL;
  }
  this->used_vertexes.clear();
}

void ProblemSolver::DFS(Vertex<unsigned> *worker, Vertex<unsigned> *curr_v,
                        MutablePriorityQueue<MeetingPoint> &q, unsigned total) {
  // TODO Trabalhores caminham bidirecionalmente??????????
  if (total > d_max)
    return;

  curr_v->visited = true;
  this->used_vertexes.insert(curr_v);

  MeetingPoint mp(curr_v);
  auto it = find(meeting_points.begin(), meeting_points.end(), mp);

  if (it != meeting_points.end()) {
    it->addWorker(worker);
    if (it->isProcessed()) {
      q.decreaseKey(&(*it));
    } else {
      it->setProcessed(true);
      q.insert(&(*it));
    }
  }

  for (Edge<unsigned> *e : curr_v->adj) {
    if (!e->getDest()->visited) {
      DFS(worker, e->getDest(), q, total + e->getWeight());
    }
  }
}

void ProblemSolver::assignMeetingPoints(void) {
  MutablePriorityQueue<MeetingPoint> q;

  for (Vertex<unsigned> *v : this->g->getVertexSet())
    v->visited = false;

  for (Vertex<unsigned> *w : this->workers) {
    resetUsedVertexes();
    DFS(w, w, q);
  }

  size_t worker_cnt = 0;
  while (!q.empty()) {
    MeetingPoint *m = q.extractMin();
    for (Vertex<unsigned> *w : m->getWorkers()) {
      ++worker_cnt;
      for (MeetingPoint &m_to_update : this->meeting_points) {
        if (m_to_update.isProcessed() && m_to_update != *m) {
          if (m_to_update.removeWorker(w))
            q.decreaseKey(&m_to_update);
        }
      }
    }
  }

  if (worker_cnt < this->workers.size())
    throw WorkersWithoutMeetingPoint(this->workers.size() - worker_cnt);

  auto it =
      remove_if(meeting_points.begin(), meeting_points.end(),
                [](MeetingPoint &m) { return m.getWorkers().size() == 0; });
  this->meeting_points.erase(it, meeting_points.end());

  for (auto v : meeting_points)
    v.getVertex()->addType(MEETINGPOINT);
}

/**************** CALC SHORTEST MEETING PATHS ****************/
void ProblemSolver::calcMeetPath(meetingPath &mp, unsigned const s,
                                 unsigned const t) const {
  mp.path = g->getPath(s, t, &mp.dist);
  if (mp.path.size() == 0 || mp.path[0] != s) // didn't find a path
    mp.dist = -1;
}

void ProblemSolver::calcMeetingDists(void) {
  meeting_path.clear();

  size_t n = meeting_points.size();
  meeting_path.resize((n + 1) + 1);
  for (size_t i = 0; i < n + 2; ++i)
    meeting_path[i].resize((n + 1) + 1);

  unsigned s, t;
  for (size_t i = 0; i < n; ++i) {
    s = meeting_points[i].getInfo();
    for (size_t j = 0; j < n; ++j) {
      if (i == j)
        continue;

      t = meeting_points[j].getInfo();
      if (!g->dijkstraBidirectional(s, t))
        throw NoPathFound(s, t);
      calcMeetPath(meeting_path[i][j], s, t);
    }

    if (!g->dijkstraBidirectional(s, V0))
      throw NoPathFound(s, V0);
    calcMeetPath(meeting_path[i][n], s, V0);

    if (!g->dijkstraBidirectional(s, Cn))
      throw NoPathFound(s, Cn);
    calcMeetPath(meeting_path[i][n + 1], s, Cn);
  }

  for (size_t i = 0; i < n; ++i) {
    t = meeting_points[i].getInfo();
    if (!g->dijkstraBidirectional(V0, t))
      throw NoPathFound(V0, t);
    calcMeetPath(meeting_path[n][i], V0, t);
  }

  for (size_t i = 0; i < n; ++i) {
    t = meeting_points[i].getInfo();
    if (!g->dijkstraBidirectional(Cn, t))
      throw NoPathFound(Cn, t);
    calcMeetPath(meeting_path[n + 1][i], Cn, t);
  }

  if (!g->dijkstraBidirectional(V0, Cn))
    throw NoPathFound(V0, Cn);
  calcMeetPath(meeting_path[n][n + 1], V0, Cn);

  if (!g->dijkstraBidirectional(Cn, V0))
    throw NoPathFound(Cn, V0);
  calcMeetPath(meeting_path[n + 1][n], Cn, V0);
}

/**************** TSP ****************/
size_t ProblemSolver::tspInit(vector<bool> &visited) {
  size_t n = meeting_points.size();
  visited = vector<bool>(n + 2, false);

  vector<meetingPath> end2start_vert;
  meetingPath unreachable_vert = {-1, vector<unsigned>()};
  for (size_t i = 0; i < n + 2; ++i) {
    end2start_vert.push_back(unreachable_vert);
    meeting_path[i].push_back(unreachable_vert);
  }
  end2start_vert.push_back(unreachable_vert); // end2start selfconnect forbidden
  meeting_path.push_back(end2start_vert);

  meeting_path[n + 2][n].dist = 0;     // -> V0
  meeting_path[n + 1][n + 2].dist = 0; // <- Cn

  return n;
}

unsigned ProblemSolver::indexToInfo(const unsigned index) const {
  size_t n = this->meeting_points.size();
  if (index > n + 1)
    return 0;

  if (index == n)
    return this->V0;
  else if (index == n + 1)
    return this->Cn;

  return this->meeting_points.at(index).getInfo();
}

vector<unsigned> ProblemSolver::reconstructPath(const vector<unsigned> &path) {
  vector<unsigned> res;
  if (path.size() < 2)
    return path;

  unsigned start = path[0], end;
  size_t i = 1;
  while (i < path.size()) {
    end = path.at(i);
    auto target_path = meeting_path[start][end].path;
    res.insert(res.end(), target_path.begin(), target_path.end());
    ++i;
    start = end;
  }

  return res;
}

vector<unsigned> ProblemSolver::tsp(void) {
  vector<bool> visited;
  size_t n = tspInit(visited);

  vector<unsigned> path;
  unsigned curr_vert = n + 1;       // start at Cn
  unsigned curr_target = curr_vert; // init for exception in case of failure
  visited[curr_vert] = true;

  for (size_t num_visited = 1; num_visited <= n + 2; ++num_visited) {
    double min = INF;
    bool found_target = false;

    for (size_t i = 0; i <= n + 2; ++i) {
      double check_dist = meeting_path[curr_vert][i].dist;
      if (check_dist >= 0 && check_dist < min && !visited[i]) {
        found_target = true;
        min = check_dist;
        curr_target = i;
      }
    }

    if (!found_target)
      throw NoPathFound(indexToInfo(curr_vert), indexToInfo(curr_target));

    path.push_back(curr_target);

    curr_vert = curr_target;
    visited[curr_vert] = true;
  }

  path.erase(path.begin()); // Remove aux
  if (meeting_path[curr_vert][n + 1].dist < 0)
    throw NoPathFound(indexToInfo(curr_vert), indexToInfo(n + 1));
  path.push_back(n + 1);

  return path;
}

/**************** ASSIGN BUSES ****************/
vector<unsigned> ProblemSolver::genBusPath(unsigned start, unsigned end,
                                           const vector<unsigned> &p) {
  vector<unsigned> res, to_add;
  if (p.size() == 0)
    return res;

  unsigned firstMP = indexToInfo(p[0]), lastMP = indexToInfo(p[p.size() - 1]);
  // Garage to first meeting point
  if (start != firstMP) {
    g->dijkstraShortestPath(start);
    res = g->getPath(start, firstMP);
  }

  // Meeting point paths
  if (p.size() > 1) {
    to_add = this->reconstructPath(p);
    res.insert(res.end(), to_add.begin(), to_add.end());
  }

  // lastMP meeting point to company
  if (end != lastMP) {
    g->dijkstraShortestPath(lastMP);
    to_add = g->getPath(lastMP, end);
    res.insert(res.end(), to_add.begin(), to_add.end());
  }

  return res;
}

vector<vector<unsigned>>
ProblemSolver::assignBuses(const vector<unsigned> &main_path) {
  vector<vector<unsigned>> res;
  vector<unsigned> curr_path;
  unsigned curr_bus = 0;
  unsigned curr_cap = this->bus_capacities.at(curr_bus);
  // main_path.erase(main_path.begin());
  // main_path.erase(main_path.end());

  for (size_t i = 1; i < main_path.size() - 1; ++i) {
    if (curr_cap == 0) { // Assign next bus
      ++curr_bus;
      if (curr_bus == this->bus_capacities.size())
        throw runtime_error(
            "Exceded all bus capacities without filling every meeting point");

      curr_cap = this->bus_capacities.at(curr_bus);
      res.push_back(genBusPath(this->V0, this->Cn, curr_path));
      curr_path.clear();
    }
    curr_path.push_back(main_path.at(i));
    --curr_cap;
  }
  if (curr_path.size() > 0)
    res.push_back(genBusPath(this->V0, this->Cn, curr_path));

  return res;
}

void ProblemSolver::clearSolution() {
  this->resetUsedVertexes();

  for (Vertex<unsigned> *v : this->workers)
    v->removeType(RESIDENCE);
  this->workers.clear();

  for (MeetingPoint mp : this->meeting_points)
    mp.getVertex()->removeType(MEETINGPOINT);

  // Reset meeting point information
  this->meeting_points.clear();
  for (Vertex<unsigned> *v : this->bus_stops)
    this->meeting_points.push_back(MeetingPoint(v));

  //  for (vector<meetingPath> mp : this->meeting_path) {
  //    for (meetingPath mpath : mp)
  //      mpath.path.clear();
  //    mp.clear();
  //  }
  this->meeting_path.clear();

  Vertex<unsigned> *enterprise = this->g->findVertex(this->Cn);
  if (enterprise != nullptr)
    enterprise->removeType(COMPANY);

  this->Cn = 0;
}
