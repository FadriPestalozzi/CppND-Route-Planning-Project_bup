#include "route_model.h"
#include <iostream>

// class constructor
// calling Model constructor with xml data
// : Model(xml) == inheriting methods from Model
RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
    // Create RouteModel nodes and store in vector m_Nodes
    int counter = 0;
    // this_model->Nodes is a getter function, getting all nodes stored in Model
    for (Model::Node node : this->Nodes()) {
        // for each node call constructor Node()
        // this = pointer to current RouteModel
        // ROUTE_MODEL_H --- Node(int idx, RouteModel * search_model, Model::Node node) : Model::Node(node), 
        // parent_model(search_model), index(idx) {}
        // --> parent_model defined by search_model
        // initialize &m_Nodes with (node_idx, *RouteModel, current_node)
        m_Nodes.emplace_back(Node(counter, this, node));
        // emplace_back returns reference
        // push_back may create temp copy
        counter++;
    }
    CreateNodeToRoadHashmap();
}


// create node_to_road hashmap
// mapping index of each node to the road it is on
void RouteModel::CreateNodeToRoadHashmap() {
    // Roads() = get all model roads
    // iterate through all roads
    for (const Model::Road &road : Roads()) {
        // exclude footways
        if (road.type != Model::Road::Type::Footway) {
            // from all Ways() 
            // access index of current [road.way]
            // iterate through all .nodes on that way
            for (int node_idx : Ways()[road.way].nodes) {
                // check if current node_idx already in hashmap
                if (node_to_road.find(node_idx) == node_to_road.end()) {
                    // if index not in hashmap
                    // create empty vector of roads
                    // and push road into that vector 
                    node_to_road[node_idx] = std::vector<const Model::Road *> ();
                }
                // build hasmap of node indices and road addresses
                node_to_road[node_idx].push_back(&road);
            }
        }
    }
}


RouteModel::Node *RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
    Node *closest_node = nullptr;
    Node node;

    for (int node_index : node_indices) {
        // node object at node_index 
        // from vector of nodes SNodes
        node = parent_model->SNodes()[node_index];
        if (this->distance(node) != 0 && !node.visited) {
            // if new distance closer than previous record, 
            // replace closest_node with current node
            if (closest_node == nullptr || this->distance(node) < this->distance(*closest_node)) {
                closest_node = &parent_model->SNodes()[node_index];
            }
        }
    }
    // pointer to node in vector node_indices closest to current Node
    return closest_node; 
}

// for each of the roads
// on hashmap node_to_road
// with index of current node [this->index]
// call FindNeighbor on vector of nodes belonging to that road
void RouteModel::Node::FindNeighbors() {
    for (auto & road : parent_model->node_to_road[this->index]) {
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        // add new neighbor to back of vector neighbors 
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);
        }
    }
}


RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
    Node input;
    input.x = x;
    input.y = y;

    // min_dist = h-value
    // largest possible value for type int is std::numeric_limits<int>::max()
    float min_dist = std::numeric_limits<float>::max();
    float dist;
    int closest_idx; // index of currently closest node

    // iterate over all roads which are not type Footway
    for (const Model::Road &road : Roads()) {
        if (road.type != Model::Road::Type::Footway) {
            // for all nodes on a way
            // check distance to input node at node_idx
            for (int node_idx : Ways()[road.way].nodes) {
                dist = input.distance(SNodes()[node_idx]);
                // update if new minimum distance
                if (dist < min_dist) {
                    closest_idx = node_idx;
                    min_dist = dist;
                }
            }
        }
    }
    // return node at closest index
    return SNodes()[closest_idx];
}