#include "route_planner.h"
#include <algorithm>
#include <iostream>
using std::cout;

// class constructor with : initializer_list
RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y) : m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    // declaration in header file---route_planner.h---RoutePlanner---Private
    // RouteModel::Node *start_node;
    // In ROUTE_PLANNER_H the RouteModel::Node attributes *start_node and *end_node are defined as *pointer
    // that's why we need to store output &address 
    start_node = &(m_Model.FindClosestNode(start_x,start_y)); //  parentheses for better readability
    end_node = &(m_Model.FindClosestNode(end_x,end_y));
}


// TODO 3: Implement the CalculateHValue method.
// Tips:

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    
    // h-value = distance to end_node
    
    // Node objects have a distance method 
    // node.distance() = distance to other node
    
    // ROUTE_MODEL_H --- distance between current_node.xy and other_node.xy
    /*float distance(Node other) const {
        // pythagoras
        return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
    }*/
    
    // operator -> 
        // used to access attribute/method from object pointer
    // this = used in member function to access outer object address
        // since distance() is function in route_model
        // this = pointer to current model
    // this->end_node = from current model, access *end_node
        // ROUTE_PLANNER_H --- RouteModel::Node *end_node;
        // dereference *node to evaluate distance(node value)
    float H_current = node->distance(*(this->end_node));  // alternative:    distance(*end_node)
    
    // cout << "H-value between current_node and end_node= " << H_current <<"\n";
    
    return H_current;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips: 

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {

    // use FindNeighbors() method of current_node to populate vector current_node.neighbors

    // ROUTE_MODEL_H --- parent_model
    // RouteModel * parent_model = nullptr; 

    /* ROUTE_MODEL_CPP --- vector neighbors
    void RouteModel::Node::FindNeighbors() {
        // for all nodes in road 
    for (auto & road : parent_model->node_to_road[this->index]) {
        // 
        RouteModel::Node *new_neighbor = this->FindNeighbor(parent_model->Ways()[road->way].nodes);
        // add new neighbor to back of vector neighbors 
        if (new_neighbor) {
            this->neighbors.emplace_back(new_neighbor);} // use emplace_back() instead of push_back() avoids unnecessary copying
    }}
    */

    current_node->FindNeighbors();

    // *neighbor_node is a pointer to type RouteModel::Node
    // evaluate .neighbors method on *current_node
    // For each *neighbor_node in *current_node.neighbors, set parent and values h,g
    for (RouteModel::Node *neighbor_node : current_node->neighbors) {

        // only add neighbor_node if not yet visited = !visited
        // if(!neighbor_node->visited){ 
        // this check is redundant, FindNeighbor within FindNeighbors only generates nodes that are not yet visited 

            // ROUTE_MODEL_H --- Node* parent = nullptr;
            // every node has a *parent, initialized as nullptr
            // store current_node as parent of all newly visited neighors
            neighbor_node->parent = current_node;

            // use CalculateHValue to get h-Value of this *neighbor_node
            // ROUTE_PLANNER_H --- float CalculateHValue(RouteModel::Node const *node);
            neighbor_node->h_value = this->CalculateHValue(neighbor_node);

            // g-value is sum of path distances
            neighbor_node->g_value = current_node->g_value + current_node->distance(*(neighbor_node));

            // add neighbor_node to open_list of this *current_node
            this->open_list.push_back(neighbor_node);

            // set the node's visited attribute to true
            neighbor_node->visited = true;

        // }
    }
}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
RouteModel::Node *RoutePlanner::NextNode() {
    // Sort open_list according to f=h+g
    std::sort((this->open_list).begin(),  // address of first array element 
              (this->open_list).end(),    // address of last array element 
              [] // lambda-introducer = capture clause
                 // [ ] = no variables accessed/captured from surroundings
              (RouteModel::Node* node_1,   // lambda declarator = parameter list
               RouteModel::Node* node_2) {
                // sorting in descending order, start with greatest sum
               return node_1->h_value + node_1->g_value > node_2->h_value + node_2->g_value;
               });
    
    // Create a pointer to the node in the list with the lowest sum.
    // after sorting, the lowest sum value is at the back
    RouteModel::Node *node_min_f = (this->open_list).back();

    // remove that node from the open_list
    (this->open_list).pop_back();

    // return pointer to node with minimum f=h+g
    return node_min_f;
}


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:

// method ConstructFinalPath returns vector of type node 
// taking pointer to current (final) node as argument
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    
    // initialize distance and vector path_found
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here. START

    // initialize iteration node
    RouteModel::Node *iter_node = current_node;

    // iteratively follow the chain of node parents backwards until starting node reached
    while(iter_node != this->start_node){
    
        // For each node in the chain, add the distance from the node to its parent to the distance variable.
        distance += iter_node->distance(*(iter_node->parent)); // distance(type dereferenced_node)

        // build backwards path
        path_found.insert(path_found.end(), *iter_node);

        // next parent iteration node
        iter_node = iter_node->parent;
    }

    // add start_node
    path_found.push_back(*iter_node);

    // ensure correct order of path_found
    // first to last == start_node to end_node
    reverse(path_found.begin(), path_found.end());

    // TODO: Implement your solution here. END

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:

void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.

    // set start
    open_list.push_back(start_node);
    start_node->visited=true;

    // search until end 
    while (open_list.size()>0){

        // - Use the NextNode() method to sort the open_list and return the next node.
        current_node = NextNode();
            /*
            // return pointer to node with minimum f=h+g
            return node_min_f;
            */

        // check if end reached
        if(current_node==end_node){

            // - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
            std::vector<RouteModel::Node> path_final = ConstructFinalPath(current_node);
                /*
                std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node)
                return path_found;
                */

            // - Store the final path in the m_Model.path attribute before the method exits. 
            // This path will then be displayed on the map tile.
            m_Model.path = path_final;

            // exit loop
            return;
        }

        // if end not reached, use AddNeighbors method to add all of the neighbors of the current node to the open_list.
        AddNeighbors(current_node);
            /*
            std::vector<RouteModel::Node*> open_list

            // add neighbor_node to open_list of this *current_node
            this->open_list.push_back(neighbor_node);
            */
    }
}