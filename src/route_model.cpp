#include "route_model.h"
#include <iostream>

RouteModel::RouteModel(const std::vector<std::byte> &xml) : Model(xml) {
  int counter = 0;
  for (Model::Node node : this->Nodes()) {
    m_Nodes.push_back(Node(counter, this, node));
    counter++;
  }
  CreateNodeToRoadHashmap();
}

void RouteModel::CreateNodeToRoadHashmap() {
  for (const Model::Road& road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_index : Ways()[road.way].nodes) {
        if (node_to_road.find(node_index) == node_to_road.end()) {
          node_to_road[node_index] = std::vector<const Model::Road*> ();
        }
        node_to_road[node_index].push_back(&road);
      }
    }
  }
}

RouteModel::Node* RouteModel::Node::FindNeighbor(std::vector<int> node_indices) {
  Node *closest_node = nullptr;
  Node node;
  for (int node_index : node_indices) {
    node = parent_model->SNodes()[node_index];
    float dist = this->distance(node);
    if (this->distance(node) != 0 && !node.visited) {
      if (closest_node == nullptr || (this->distance(node) < this->distance(*closest_node))) {
        closest_node = &(parent_model->SNodes()[node_index]);
      }
    }
  }
  return closest_node;
}
          
void RouteModel::Node::FindNeighbors() {
  for (auto& road : parent_model->node_to_road[this->index]) {
    RouteModel::Node *neighbor = FindNeighbor(parent_model->Ways()[road->way].nodes);
    if (neighbor) {
      this->neighbors.emplace_back(neighbor);
    }
  }
}
          
RouteModel::Node &RouteModel::FindClosestNode(float x, float y) {
  Node input;
  input.x = x;
  input.y = y;
  float min_dist = std::numeric_limits<float>::max();
  int closest_idx;
  for (const Model::Road & road : Roads()) {
    if (road.type != Model::Road::Type::Footway) {
      for (int node_index : Ways()[road.way].nodes) {
        if (input.distance(SNodes()[node_index]) < min_dist) {
          closest_idx = node_index;
          min_dist = input.distance(SNodes()[node_index]);
        }
      }
    }
  }
  return SNodes()[closest_idx];
}
