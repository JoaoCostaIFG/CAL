#include <cstdio>
#include <fstream>
#include <iostream>
#include <sstream>

#include "graphviewer.h"

void exercicio1();
void exercicio2();
void exercicio3();

void closewindow(GraphViewer *gv) {
  getchar();
  gv->closeWindow();
}

void exercicio1() {
  /* basics */
  GraphViewer *gv = new GraphViewer(600, 600, true);

  gv->setBackground("../resources/gato.png");
  gv->createWindow(600, 600);

  // colors
  gv->defineVertexColor("blue");
  gv->defineEdgeColor("black");

  // create vertex
  gv->addNode(0);
  gv->addNode(1);
  gv->rearrange(); // update graph

  // bidirectional edge (0 -- 1)
  gv->addEdge(0, 0, 1, EdgeType::UNDIRECTED);
  // directed edge (1 -> 0)
  gv->addEdge(1, 1, 0, EdgeType::DIRECTED);

  // gv->removeNode(1); // remove vert

  gv->addNode(2);
  gv->setVertexColor(2, "green");            // vertex 2 is green
  gv->setVertexLabel(2, "This is a vertex"); // vertex label
  gv->rearrange();                           // update graph

  gv->addEdge(2, 0, 2, EdgeType::UNDIRECTED);
  gv->setEdgeColor(2, "yellow");          // edge 2 is yellow
  gv->setEdgeLabel(2, "This is an edge"); // edge label

  closewindow(gv);
}

void exercicio2() {
  /* animation */
  // 3rd arg false so we can manually set vertex pos
  GraphViewer *gv = new GraphViewer(600, 600, false);
  gv->createWindow(600, 600);

  /* draw stickman */
  // verts
  gv->addNode(0, 300, 50);
  gv->addNode(1, 318, 58);
  gv->addNode(2, 325, 75);
  gv->addNode(3, 318, 93);
  gv->addNode(4, 300, 100);
  gv->addNode(5, 282, 93);
  gv->addNode(6, 275, 75);
  gv->addNode(7, 282, 58);
  gv->addNode(8, 150, 200);
  gv->addNode(9, 300, 200);
  gv->addNode(10, 450, 200);
  gv->addNode(11, 300, 400);
  gv->addNode(12, 200, 550);
  gv->addNode(13, 400, 550);
  // edges
  gv->addEdge(0, 0, 1, EdgeType::UNDIRECTED);
  gv->addEdge(1, 1, 2, EdgeType::UNDIRECTED);
  gv->addEdge(2, 2, 3, EdgeType::UNDIRECTED);
  gv->addEdge(3, 3, 4, EdgeType::UNDIRECTED);
  gv->addEdge(4, 4, 5, EdgeType::UNDIRECTED);
  gv->addEdge(5, 5, 6, EdgeType::UNDIRECTED);
  gv->addEdge(6, 6, 7, EdgeType::UNDIRECTED);
  gv->addEdge(7, 7, 0, EdgeType::UNDIRECTED);
  gv->addEdge(8, 4, 9, EdgeType::UNDIRECTED);
  gv->addEdge(9, 9, 8, EdgeType::UNDIRECTED);
  gv->addEdge(10, 9, 10, EdgeType::UNDIRECTED);
  gv->addEdge(11, 9, 11, EdgeType::UNDIRECTED);
  gv->addEdge(12, 11, 12, EdgeType::UNDIRECTED);
  gv->addEdge(13, 11, 13, EdgeType::UNDIRECTED);

  for (size_t i = 0; i < 10; ++i) {
    if (i % 2 == 0) { // frame 2
      gv->removeNode(12);
      gv->removeNode(13);

      gv->addNode(14, 250, 550);
      gv->addNode(15, 350, 550);
      gv->addEdge(14, 11, 14, EdgeType::UNDIRECTED);
      gv->addEdge(15, 11, 15, EdgeType::UNDIRECTED);
    } else { // frame 1
      gv->removeNode(14);
      gv->removeNode(15);

      gv->addNode(12, 200, 550);
      gv->addNode(13, 400, 550);
      gv->addEdge(12, 11, 12, EdgeType::UNDIRECTED);
      gv->addEdge(13, 11, 13, EdgeType::UNDIRECTED);
    }

    gv->rearrange(); // update graph
    sleep(1);
  }

  closewindow(gv);
}

void exercicio3() {
  /* read from files */
  // 3rd arg false so we can manually set vertex pos
  GraphViewer *gv = new GraphViewer(600, 600, false);
  gv->createWindow(600, 600);

  std::string line;
  int id, a, b;

  // verts
  std::ifstream vert_file("../resources/mapa1/nos.txt");
  while (std::getline(vert_file, line)) {
    std::istringstream iss(line);

    iss >> id;
    iss.ignore(1000, ';');
    iss >> a;
    iss.ignore(1000, ';');
    iss >> b;

    std::cout << id << " " << a << " " << b << endl;
    gv->addNode(id, a, b);
  }
  vert_file.close();

  // edges
  std::ifstream edge_file("../resources/mapa1/arestas.txt");
  while (std::getline(edge_file, line)) {
    std::istringstream iss(line);

    iss >> id;
    iss.ignore(1000, ';');
    iss >> a;
    iss.ignore(1000, ';');
    iss >> b;

    gv->addEdge(id, a, b, EdgeType::UNDIRECTED);
  }
  edge_file.close();

  gv->rearrange(); //update graph
  closewindow(gv);
}

void exercicio4() {
  /* read from files */
  GraphViewer *gv = new GraphViewer(600, 600, true);
  gv->createWindow(600, 600);

  std::string line;
  int id, a, b, label;

  // edges
  std::ifstream edge_file("../resources/mapa2/edges.txt");
  while (std::getline(edge_file, line)) {
    std::istringstream iss(line);

    iss >> id;
    iss.ignore(1000, ';');
    iss >> a;
    iss.ignore(1000, ';');
    iss >> b;
    iss.ignore(1000, ';');
    iss >> label;

    gv->addEdge(id, a, b, EdgeType::UNDIRECTED);
    gv->setEdgeLabel(id, std::to_string(label)); // edge label
  }
  edge_file.close();

  gv->rearrange(); //update graph
  closewindow(gv);
}

int main() {
  // exercicio1();

  exercicio2();

  // exercicio3();

  // exercicio4();

  return 0;
}
