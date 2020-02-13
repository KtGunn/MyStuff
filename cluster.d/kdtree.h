/* \author Aaron Brown */
// Quiz on implementing kd tree


#include "../src.d/render/render.h"
//#include "../../render/render.h"


// Structure to represent node of kd tree
struct Node {
  
  // Contents of the node, coords & ID
  std::vector<float> point;
  int id;
  
  // Pointer to the attached nodes
  Node* left;
  Node* right;
  
  // Node constructor
  Node (std::vector<float> arr, int setId)
  :	point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
  Node* root;
  
  KdTree() : root (NULL) {}
  
  // Arguments:
  //  id: numeric count of point in the cloud (index)
  //  point: xyz coordinates point
  //
  void insert (std::vector<float> point, int id)
  {
    // TODO: Fill in this function to insert a new point into the tree
    // the function should create a new node and place correctly with in the root 
    insertHelper (&root, 0,  point, id) ;
    return;
  }
  
  
  // This is s recursive function
  void insertHelper (Node** node, uint depth, std::vector<float> point, int id) {
    
    if (*node == nullptr) {
      
      // Create and assign the new node
      *node = new Node (point, id);
      
    } else {
      
      // Compare based on x[0]/y[1]
      uint Zero1 = depth % 2;
      
      if (point[Zero1] < ((*node)->point[Zero1])) {
        // Go left
        insertHelper (&((*node)->left), 1+depth, point, id);
        
      } else {
        // Go right
        insertHelper (&((*node)->right), 1+depth, point, id);
      }
    }
    
    return;
  }
  
  // return a list of point ids in the tree that are within distance of target
  std::vector<int> search(std::vector<float> target, float distanceTol)
  {
    std::vector<int> ids;
    
    // Initialize
    Node* ptNode = root;
    float tx = target[0];
    float ty = target[1];
    
    uint counter = 0;
    searchHelper (root, target,  distanceTol, counter);
    std::cout << " Count stands at " << counter << std::endl;
    
    return ids;
  }
  
  // ONLY print out results
  void searchHelper (Node* ptNode, std::vector<float> target, float dTol, uint& counter)
  {
    ++counter;
      
    // SUPER important
    if ( ptNode == nullptr) {
      return;
    }
    
    // Initialize
    float tx = target[0];
    float ty = target[1];
    
    if ( ptNode->point[0] - tx > dTol ) {
      // We are right of target -> go smaller/left
      searchHelper (ptNode->left, target, dTol, counter);
      
    } else if ( tx - ptNode->point[0] > dTol ) {
      // We are left of target -> go bigger/right
      searchHelper (ptNode->right, target, dTol, counter);
      
    } else {
      // We are W/I x-range
      //
      if ( ptNode->point[1] - ty > dTol) {
        // We are above target -> go smaller/left
        searchHelper (ptNode->left, target, dTol, counter);
        
      } else if ( ty - ptNode->point[1] > dTol ) {
        // We are below target -> go bigger/right
        searchHelper (ptNode->right, target, dTol, counter);
        
      } else {
        // We are INSIDE box
        // if ||target - point|| < dTol; add index to vector
        // Go left
        // Go right also
        std::cout << " Got one id=" << ptNode->id << std::endl;
        
        searchHelper (ptNode->right, target, dTol, counter);
        searchHelper (ptNode->left, target, dTol, counter);
      }
    }
    
    return;
  }
};




