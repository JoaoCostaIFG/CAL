#include <chrono>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>

#include "include/Graph.h"
#include "include/GraphRenderer.h"
#include "include/ProblemSolver.h"

#define DFLT_BUS_FILE "../resources/porto/bus_stops"
#define DFLT_COMPANIES_FILE "../resources/porto/companies"
#define DFLT_CONFIG_FILE "../resources/config"
#define DFLT_DEAD_EDGE_FILE "../resources/porto/dead_edges"
#define DFLT_EDGE_FILE "../resources/porto/edges"
#define DFLT_NODE_FILE "../resources/porto/nodes"

ASSIGNED_BUSES_TYPE solve(ProblemSolver &ps) {
  ps.calcMeetingDists();
  return ps.assignBuses(ps.tsp());
}

void render(ProblemSolver &ps, ASSIGNED_BUSES_TYPE &buses,
            GraphRenderer &renderer) {

  renderer.busDraw(ps.getGraph());
  renderer.residenceDraw(ps.getGraph());
  renderer.meetingDraw(ps.getGraph());
  for (vector<unsigned> p : buses)
    renderer.routeDraw(ps.getGraph(), p);
  renderer.garageDraw(ps.getV0());
  renderer.companyDraw(ps.getCn());
  renderer.rearrange();
}

void clearSolution(ProblemSolver &ps, ASSIGNED_BUSES_TYPE &buses,
                   GraphRenderer &renderer) {
  for (vector<unsigned> p : buses)
    renderer.routeReset(ps.getGraph(), p);
  renderer.vertsReset(ps.getGraph());
  renderer.rearrange();
  ps.clearSolution();
}

void importRand(ProblemSolver &ps) {
  cout << "Seed (blank for time(null))? ";
  string line;
  unsigned int seed;
  if (line == "") { // blank
    seed = time(NULL);
  } else {
    istringstream ss(line);
    ss >> seed;
  }

  bool repeat = false;
  do {
    srand(seed);
    cout << "Seed: " << seed << endl;
    size_t num_verts = ps.getGraph()->getVertexSet().size();
    size_t worker_num = rand() % num_verts / 100;
    cout << worker_num << " workers generated." << endl;
    size_t bus_num = worker_num * 20;
    cout << bus_num << " bus stops generated." << endl;
    ps.setV0(rand() % num_verts);
    ps.setCn(rand() % num_verts);

    cout << "What's the max distance a worker can be from a meeting point? ";
    double d_max;
    cin >> d_max;
    cin.ignore(1000, '\n');

    repeat = false;
    try {
      ps.genRandConf(bus_num, worker_num, d_max);
      ps.assignMeetingPoints();
    } catch (WorkersWithoutMeetingPoint &e) {
      repeat = true;
      cout << e.what() << endl;
      cout << endl
           << "The max distance you chose doesn't work for this seed. Try a "
              "bigger one."
           << endl;
    }
  } while (repeat);
  cout << "Finished generating a random configuration." << endl;
}

bool getConfirmation(const string &s, bool yes_is_dflt) {
  char op = 0;
  do {
    cout << s;
    op = getchar();
    if (op != '\n')
      cin.ignore(1000, '\n');

    op = tolower(op);
    switch (op) {
    case '\n':
      return yes_is_dflt;
    case 'n':
      return false;
    case 'y':
      return true;
    default:
      cout << "unkown answer " << op << endl;
      break;
    }
  } while (true);
}

void full_cycle(ProblemSolver &ps, GraphRenderer &renderer) {
  ASSIGNED_BUSES_TYPE buses = solve(ps);
  render(ps, buses, renderer);
  getchar();
  renderer.close();
}

vector<string> read_companies() {
  cout << "Companies file (empty for default: " << DFLT_COMPANIES_FILE << "): ";
  cout.flush();

  string companies_filename;
  getline(cin, companies_filename);
  if (companies_filename.empty())
    companies_filename = DFLT_COMPANIES_FILE;

  vector<string> companies;
  ifstream companies_file(companies_filename);
  if (!companies_file.is_open()) {
    cerr << "Couldn't open file '" << companies_filename << "' for reading." << endl;
    exit(EXIT_FAILURE);
  }
  string line;
  while (getline(companies_file, line))
    companies.push_back(line);

  return companies;
}

ProblemSolver base_graph_import() {
  ProblemSolver ps(DFLT_NODE_FILE, DFLT_EDGE_FILE, DFLT_CONFIG_FILE);
  ps.readDeadEdges(DFLT_DEAD_EDGE_FILE);
  ps.fixGraph();

  return ps;
}

int main(int argc, char *argv[]) {
  ProblemSolver ps = base_graph_import();

  /* draw base graph */
  cout << "Creating a graphviewer window. Please wait.." << endl;
  GraphRenderer renderer;
  renderer.setTagVerts(
      getConfirmation("Show Vertex information in graphviewer? [Y/n] ", true));
  cout << "Drawing graph. Please wait.." << endl;
  renderer.fullDraw(ps.getGraph());
  renderer.deadEdgeDraw(ps.getDeadEdges());

  /* check if we're going random */
  if (getConfirmation("Go random? [y/N] ", false)) {
    importRand(ps);
    full_cycle(ps, renderer);
    return EXIT_SUCCESS;
  } else { // otherwise import (deterministic) bus stops
    ps.importBusStops(DFLT_BUS_FILE);
  }

  /* import companies to generate solutions for */
  vector<string> companies = read_companies();
  if (companies.empty()) {
    cout << "No company found" << endl;
    return EXIT_FAILURE;
  }

  /* don't querry user if there's only 1 company in the company's file */
  if (companies.size() == 1) {
    cout << "Only found 1 problem file '" << companies[0] << "'. Calculating solution." << endl;
    ps.importCompany(companies[0]);
    ps.assignMeetingPoints();
    full_cycle(ps, renderer);
    return EXIT_SUCCESS;
  }

  /* list found problems so user can easily select the ones they want */
  cout << "Found the following 'problems' to solve:" << endl;
  vector<ASSIGNED_BUSES_TYPE> solutions;
  solutions.reserve(companies.size());
  for (size_t i = 0; i < companies.size(); ++i) {
    solutions.push_back(ASSIGNED_BUSES_TYPE());
    cout << '\t' << i << " - " << companies.at(i) << endl;
  }

  /* generate solutions */
  int lastSelected = -1;
  int selected;
  do {
    cout << "Select a company [0, " << companies.size() - 1 << "] (-1 to exit) "
         << endl;
    cin >> selected;
    cin.ignore(1000, '\n');

    if (selected < 0 || selected >= (int)companies.size())
      continue;

    cout << "Selected \"" << companies.at(selected) << '"' << endl;

    if (lastSelected != -1)
      clearSolution(ps, solutions.at(lastSelected), renderer);

    ps.importCompany(companies.at(selected));
    ps.assignMeetingPoints();
    if (solutions.at(selected).empty())
      solutions.at(selected) = solve(ps);
    render(ps, solutions.at(selected), renderer);

    lastSelected = selected;
  } while (selected != -1);

  renderer.close();
  cout << "Closed graphviewer window. Please note that graphviewer can stay "
          "open in the background (sorry) so you should run 'killall java' or "
          "something similar"
       << endl;
  return EXIT_SUCCESS;
}
