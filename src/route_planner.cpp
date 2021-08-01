#include "route_planner.h"

void NodeSort(std::vector<RouteModel::Node*> *v){
 sort(v->begin(), v->end(), Compare);
}

bool Compare(RouteModel::Node* node1, RouteModel::Node* node2){
  int f1 = node1->h_value + node1->g_value;
  int f2 = node2->h_value + node2->g_value;
  return f1 > f2;
}

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Find closest nodes to start and end nodes
    start_node = &(m_Model.FindClosestNode(start_x, start_y));
    end_node = &(m_Model.FindClosestNode(end_x, end_y));
}

// Calculate H value of node
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
	return node->distance(*end_node);
}

// Search neighbours of current node which have not been visited
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
	current_node->FindNeighbors();
  	for (RouteModel::Node* neighbor : current_node->neighbors) {
      neighbor->parent = current_node;
      neighbor->h_value = CalculateHValue(neighbor);
      neighbor->g_value = current_node->g_value + current_node->distance(*neighbor);
      this->open_list.push_back(neighbor);
      neighbor->visited = true;
    }
}

// Determine the next node to progress to.
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort open_list
  NodeSort(&(this->open_list));
  // Node with lowest f-value
  RouteModel::Node* next_node = this->open_list.back();
  // Remove that node from open_list
  this->open_list.pop_back();
  return next_node;
}

// Construct final path by tracing back through the parents of each node
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.
  	while(current_node->parent){
      
      // Add parent node to path_found vector
      path_found.push_back(*current_node);
      
      // Add distance
      distance += current_node->distance((*current_node->parent));
      
      // Move on to parent node
      current_node = current_node->parent;
    }
  	
  	path_found.push_back(*start_node);
  
  	// path_found vector starts with end node, so order must be reversed
  	std::reverse(path_found.begin(), path_found.end());

  
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}

// A* search algorithm implementation
void RoutePlanner::AStarSearch() {
    RouteModel::Node *current_node = nullptr;

  	// Initialise with start_node
	this->open_list.push_back(start_node);
  
  	while(this->open_list.size()){
      // Get the next node
      current_node = NextNode();
      
      // Check if end_node reached
      if(current_node->x == end_node->x && current_node->y == end_node->y){
        m_Model.path = ConstructFinalPath(current_node);
      }
      
      // Expand search to current_node's neighbors if end_node not reached
      AddNeighbors(current_node);
    }
  

}
