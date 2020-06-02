#ifndef GRAPHRENDERER_H
#define GRAPHRENDERER_H

#include "../lib/graphviewer.h"
#include "Graph.h"

#define GV_PORT 65525

class GraphRenderer {
private:
  GraphViewer gv;
  GraphViewer createViewer(bool is_interactive, char const *const vertex_color,
                           char const *const edge_color) const;
  void display(const unsigned &width, const unsigned &height);
  bool tag_verts = false;

public:
  GraphRenderer();
  /**
   * @brief Should graphviewer tag the vertices with their info.
   */
  void setTagVerts(bool new_tag) { tag_verts = new_tag; }
  /**
   * @brief Close the graphviewer window.
   */
  void close(void);
  /**
   * @brief Add all graph nodes and edges to the graphviewer with the default
   * colors.
   * @param g Graph to get info from.
   */
  void fullDraw(const Graph<unsigned> *g);
  /**
   * @brief Colors Vertexes that are tagged as bus stops.
   * @param g Graph to get info from.
   */
  void busDraw(const Graph<unsigned> *g);
  /**
   * @brief Colors Vertexes that are tagged as residences.
   * @param g Graph to get info from.
   */
  void residenceDraw(const Graph<unsigned> *g);
  /**
   * @brief Colors Vertexes that are tagged as meeting point.
   * @param g Graph to get info from.
   */
  void meetingDraw(const Graph<unsigned> *g);
  /**
   * @brief Draws a route represented by the Vertexes that are part of it.
   * @param g Graph to get info from.
   * @param vertex_path Path to draw.
   */
  void routeDraw(const Graph<unsigned> *g, vector<unsigned> vertex_path);
  /**
   * @brief Colors the given Vertex as garage.
   * @param v0
   */
  void garageDraw(const unsigned v0);
  /**
   * @brief Colors the given Vertex as company.
   * @param cn
   */
  void companyDraw(const unsigned cn);

  void deadEdgeDraw(const vector<pair<unsigned, unsigned>> &dead_edges);
  /**
   * @brief Calls rearrange on the graphviewer window.
   */
  void rearrange(void) {
    gv.rearrange();
#ifdef DEBUG_ENABLED
    cerr << "Finished rearrange." << endl;
#endif
  }
  void vertsReset(const Graph<unsigned int> *g);
  void routeReset(const Graph<unsigned int> *g,
                  vector<unsigned int> vertex_path);
};

#endif // GRAPHRENDERER_H
