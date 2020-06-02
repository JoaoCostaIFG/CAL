#include <string>
#include <utility>

#include "include/GraphRenderer.h"

#define INTERACTIVE true
#define NOTINTERACTIVE false

#define VX_COLOUR DARK_GRAY
#define ED_COLOUR RED
#define DEAD_ED_COLOUR BLACK
#define BUS_COLOUR BLUE
#define RESIDENCE_COLOUR ORANGE
#define MEETING_COLOR GREEN
#define ROUTE_COLOR GREEN
#define GARAGE_COLOR CYAN
#define COMPANY_COLOR RED
#define EDGE_WIDTH 1
#define PATH_WIDTH 5

using namespace std;

GraphRenderer::GraphRenderer() : gv(10000, 10000, NOTINTERACTIVE, GV_PORT) {
  this->display(600, 600);
}

void GraphRenderer::display(const unsigned &width, const unsigned &height) {
  gv.createWindow(width, height);
  gv.defineVertexColor(VX_COLOUR);
  gv.defineEdgeColor(ED_COLOUR);
}

void GraphRenderer::close(void) { gv.closeWindow(); }

void GraphRenderer::fullDraw(const Graph<unsigned> *g) {
  for (Vertex<unsigned> *v : g->getVertexSet()) {
    gv.addNode(v->getInfo(), v->getCoords().getX(), v->getCoords().getY());
    if (this->tag_verts)
      gv.setVertexLabel(v->getInfo(), to_string(v->getInfo()));
  }

  for (Vertex<unsigned> *v : g->getVertexSet()) {
    for (Edge<unsigned> *e : v->getAdj())
      gv.addEdge(e->getEdgeId(), v->getInfo(), e->getDest()->getInfo(),
                 EdgeType::DIRECTED);
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished full draw." << endl;
#endif
}

void GraphRenderer::busDraw(const Graph<unsigned> *g) {
  for (Vertex<unsigned> *v : g->getVertexSet()) {
    unordered_set<VertexType> types = v->getTypes();
    if (types.find(BUSSTOP) != types.end()) {
      gv.setVertexColor(v->getInfo(), BUS_COLOUR);
    } else if (types.find(MEETINGPOINT) != types.end())
      gv.setVertexIcon(v->getInfo(), "../resources/icons/bus.png");
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished bus stop draw." << endl;
#endif
}

void GraphRenderer::residenceDraw(const Graph<unsigned> *g) {
  for (Vertex<unsigned> *v : g->getVertexSet()) {
    unordered_set<VertexType> types = v->getTypes();
    if (types.find(RESIDENCE) != types.end())
      // gv.setVertexIcon(v->getInfo(), "../resources/icons/bus.png");
      gv.setVertexColor(v->getInfo(), RESIDENCE_COLOUR);
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished residence draw." << endl;
#endif
}

void GraphRenderer::meetingDraw(const Graph<unsigned> *g) {
  for (Vertex<unsigned> *v : g->getVertexSet()) {
    unordered_set<VertexType> types = v->getTypes();
    if (types.find(MEETINGPOINT) != types.end())
      // gv.setVertexIcon(v->getInfo(), "../resources/icons/bus.png");
      gv.setVertexColor(v->getInfo(), MEETING_COLOR);
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished meeting point draw." << endl;
#endif
}

void GraphRenderer::routeDraw(const Graph<unsigned> *g,
                              vector<unsigned> vertex_path) {

  if (vertex_path.size() <= 1)
    throw;

  Vertex<unsigned> *start = g->findVertex(vertex_path.at(0));
  if (start == nullptr)
    throw;

  size_t i = 0;
  for (auto it = vertex_path.begin() + 1; it != vertex_path.end(); ++it) {
    for (Edge<unsigned> *edge : start->getAdj()) {
      if (edge->getDest()->getInfo() == *it) {
        int edgeId = edge->getEdgeId();
        gv.setEdgeColor(edgeId, ROUTE_COLOR);
        gv.setEdgeThickness(edgeId, PATH_WIDTH);
        gv.setEdgeLabel(edgeId, to_string(i++));
        start = edge->getDest();
        break;
      }
    }
  }

#ifdef DEBUG_ENABLED
  cerr << "Finished route draw." << endl;
#endif
}

void GraphRenderer::routeReset(const Graph<unsigned int> *g,
                               vector<unsigned int> vertex_path) {
  if (vertex_path.size() <= 1)
    throw;

  Vertex<unsigned> *start = g->findVertex(vertex_path.at(0));
  if (start == nullptr)
    throw;

  //  size_t i = 0;
  for (auto it = vertex_path.begin() + 1; it != vertex_path.end(); ++it) {
    for (Edge<unsigned> *edge : start->getAdj()) {
      if (edge->getDest()->getInfo() == *it) {
        int edgeId = edge->getEdgeId();
        gv.setEdgeColor(edgeId, ED_COLOUR);
        gv.setEdgeThickness(edgeId, EDGE_WIDTH);
        gv.setEdgeLabel(edgeId, "");
        start = edge->getDest();
        break;
      }
    }
  }
}

void GraphRenderer::garageDraw(const unsigned v0) {
  gv.setVertexColor(v0, GARAGE_COLOR);

#ifdef DEBUG_ENABLED
  cerr << "Finished garage draw." << endl;
#endif
}

void GraphRenderer::companyDraw(const unsigned cn) {
  gv.setVertexColor(cn, COMPANY_COLOR);

#ifdef DEBUG_ENABLED
  cerr << "Finished company draw." << endl;
#endif
}

void GraphRenderer::vertsReset(const Graph<unsigned int> *g) {
  for (Vertex<unsigned> *v : g->getVertexSet()) {
    if (v->containsType(RESIDENCE) || v->containsType(MEETINGPOINT) ||
        v->containsType(COMPANY)) {
      if (v->containsType(BUSSTOP))
        gv.setVertexColor(v->getInfo(), BUS_COLOUR);
      else
        gv.setVertexColor(v->getInfo(), VX_COLOUR);
    }
  }
}

void GraphRenderer::deadEdgeDraw(const vector<pair<unsigned, unsigned>> &dead_edges) {
  // this edges don't exist in the real graph
  ssize_t i = -1;
  for (pair<unsigned, unsigned> de : dead_edges) {
    gv.addEdge(i, de.first, de.second, EdgeType::DIRECTED);
    gv.setEdgeColor(i, DEAD_ED_COLOUR);
    --i;
  }
}
