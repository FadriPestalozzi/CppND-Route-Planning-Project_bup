#ifndef ROUTE_MODEL_H
#define ROUTE_MODEL_H

#include <limits>
#include <cmath>
#include <unordered_map>
#include "model.h"
#include <iostream>

// inheriting fro Model
class RouteModel : public Model {

  public:
    // RouteModel class has subclass Node
    // Node inherits from Model struct also called Node
    class Node : public Model::Node {
      public:
        // extending Model struct Node by adding
        // pointer to parent of Node and values h,g,visited
        Node* parent = nullptr; 
        float h_value = std::numeric_limits<float>::max();
        float g_value = 0.0;
        bool visited = false;
        // vector of node pointers called neighbors
        std::vector<Node *> neighbors;

        void FindNeighbors(); // to populate vector neighbors
        float distance(Node other) const {
            // pythagoras
            return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
        }

        // constructors
        Node(){} // default constructor
        // constructor to create RouteModelNode from existing Node
        // constructor list to initialize input variables
        Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), parent_model(search_model), index(idx) {}

      private:
        int index;
        Node * FindNeighbor(std::vector<int> node_indices);
        // pointer to RouteModel to which this Node belongs
        RouteModel * parent_model = nullptr; 
    };

    // RouteModel constructor
    RouteModel(const std::vector<std::byte> &xml);
    // method FindClosestNode to get model node closest to xy
    Node &FindClosestNode(float x, float y);
    // getter function returning vector of all model nodes
    auto &SNodes() { return m_Nodes; } 
    // store path found for rendering
    std::vector<Node> path;
    
  private:
    // private methode creating map from nodes to rodes they belong to --> find neighbors
    void CreateNodeToRoadHashmap();
    std::unordered_map<int, std::vector<const Model::Road *>> node_to_road;
    // declare vector of road pointers
    // vector<const Model::Road *>
    std::vector<Node> m_Nodes; // each RouteModel has vector m_Nodes
};

#endif
