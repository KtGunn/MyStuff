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
    
    std::cout << " TOP OF SEARCH\n";
    uint depth = 0;
    searchHelper (root, target, distanceTol, depth, ids);

    for (int index : ids ) {
      std::cout << " ids = " << index << std::endl;
    }
    return ids;
  }
  
  // ONLY print out results
  void searchHelper (Node* ptNode, std::vector<float> target, float dTol, uint depth, std::vector<int>& ids)
  {
    std::cout << " In Helper\n";
    // SUPER important
    if ( ptNode == nullptr) {
      return;
    }
    
    std::cout << "   more in helper\n";

    // Initialize
    float tx = target[0];
    float ty = target[1];

    float nx = ptNode->point[0];
    float ny = ptNode->point[1];

    if ( fabs(tx-nx) < dTol && fabs(ty-ny) < dTol) {
      std::cout << "  INSIDE BOX\n";
      // We're with the square box
      float dx = tx-nx;
      float dy = ty-ny;
      if ( sqrt ( dx*dx + dy*dy ) < dTol ) {
        // We're with in the radius of dTol -- we got one!
        ids.push_back (ptNode->id);
      }
    }


    uint Zero1 = depth % 2;
    float tc = target[Zero1];

    // std::cout << " nodeC = " <<  ptNode->point[Zero1] << " targetC = " << tc << " tol= " <<  dTol << std::endl;

    if ( ptNode->point[Zero1] - tc < dTol ) {
      // We are right of target -> go smaller/left
      searchHelper (ptNode->left, target, dTol, depth+1, ids);
    } 
    if ( tc - ptNode->point[Zero1] < dTol ) {
      // We are left of target -> go bigger/right
      searchHelper (ptNode->right, target, dTol, depth+1, ids);
    }

    return;
  }
};




