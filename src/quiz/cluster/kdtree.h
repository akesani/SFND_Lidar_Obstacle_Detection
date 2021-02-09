/* \author Aaron Brown */
// Quiz on implementing kd tree

#include <cmath>

#include "../../render/render.h"

// Structure to represent node of kd tree
struct Node {
  std::vector<float> point;
  int id;
  Node* left;
  Node* right;

  Node(std::vector<float> arr, int setId)
      : point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node* root;

  KdTree() : root(NULL) {}

  void insertHelper(Node*& node, uint depth, std::vector<float> point, int id) {
    if (node == NULL) {
      node = new Node(point, id);
    } else {
      uint cd = depth % 2;
      if (point[cd] < (node->point[cd])) {
        insertHelper(node->left, ++depth, point, id);
      } else {
        insertHelper(node->right, ++depth, point, id);
      }
    }
  }

  void insert(std::vector<float> point, int id) {
    insertHelper(root, 0, point, id);
  }

  void searchHelper(Node*& node, uint depth, std::vector<int>& ids,
                    std::vector<float> target, float distanceTol) {
    if (node != NULL) {
      float abs_x_distance = abs(target[0] - node->point[0]);
      float abs_y_distance = abs(target[1] - node->point[1]);
      if (abs_x_distance <= distanceTol || abs_y_distance <= distanceTol) {
        float abs_distance = sqrt(abs_x_distance * abs_x_distance +
                                  abs_y_distance * abs_y_distance);
        if (abs_distance <= distanceTol) {
          ids.emplace_back(node->id);
        }
      }
      uint cd = depth % 2;
      if ((target[cd] - distanceTol) < node->point[cd]) {
        searchHelper(node->left, ++depth, ids, target, distanceTol);
      }
      if ((target[cd] + distanceTol) > node->point[cd]) {
        searchHelper(node->right, ++depth, ids, target, distanceTol);
      }
    }
    return;
  }

  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol) {
    std::vector<int> ids;
    searchHelper(root, 0, ids, target, distanceTol);
    return ids;
  }
};
