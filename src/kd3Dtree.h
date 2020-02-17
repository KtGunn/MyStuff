
// 3D extension of kdtree.h file used in clustering quiz
//    created by Aaron Brown

// Only once
#pragma once

// Structure to represent node of kd tree

struct Node {
    
    // Contents of the node, XYZ-coords & ID
    pcl::PointXYZ point;
    int id;
    
    // Pointer to the attached nodes
    Node* left;
    Node* right;
    
    // Node constructor
    Node (pcl::PointXYZ arr, int setId)
	:	point(arr), id(setId), left(NULL), right(NULL) {}
};

struct KdTree {
    Node* root;
    
    KdTree() : root (NULL) {}
    
    // Arguments:
    //  id: numeric count of point in the cloud (index)
    //  3dpoint: xyz coordinates point
    //
    void insert (pcl::PointXYZ point3D, int id)
    {
	insertHelper (&root, 0,  point3D, id) ;
	return;
    }
    
    // This is a recursive function
    void insertHelper (Node** node, uint depth, pcl::PointXYZ point3D, int id) {
	
	if (*node == nullptr) {
	    
	    // Create and assign the new node
	    *node = new Node (point3D, id);
	    
	} else {
	    
	    // Compare based on x[0]/y[1]/z[2] :: '% 3' replaces '% 2'
	    uint Zero12 = depth % 3;
	    
	    float flPt[3] = {point3D.x, point3D.y, point3D.z};
	    float flNp[3] = {((*node)->point).x, ((*node)->point).y, ((*node)->point).z};

	    if (flPt[Zero12] < flNp[Zero12]) {
		// Go left
		insertHelper (&((*node)->left), 1+depth, point3D, id);
		
	    } else {
		// Go right
		insertHelper (&((*node)->right), 1+depth, point3D, id);
	    }
	}
	
	return;
    }
    
    // return a list of point ids in the tree that are within distance of target
    std::vector<int> search (pcl::PointXYZ target, float distanceTol)
    {
	std::vector<int> ids;
	
	// Initialize
	Node* ptNode = root;
	
	uint depth = 0;
	uint counter = 0; // counts the number of calls to 'searchHelper'
	searchHelper (root, target, distanceTol, depth, ids, counter);

	return ids;
    }
    
    // This is a recursive function
    void searchHelper (Node* ptNode, pcl::PointXYZ target, float dTol, uint depth, std::vector<int>& ids, uint& counter)
    {
	
	// For laughs & giggles we count how often the recurse
	++counter;
	
	// SUPER important
	if ( ptNode == nullptr) {
	    return;
	}
	
	// Initialize
	float tx = target.x;
	float ty = target.y;
	float tz = target.z;

	float nx = ptNode->point.x;
	float ny = ptNode->point.y;
	float nz = ptNode->point.z;
	
	// Quick test of w/i square box
	if ( (nx >= (tx-dTol) && nx <= (tx+dTol)) &&	\
	     (ny >= (ty-dTol) && ny <= (ty+dTol)) &&	\
	     (nz >= (tz-dTol) && nz <= (tz+dTol)) ) {
	    
	    float dx = tx-nx;
	    float dy = ty-ny;
	    float dz = tz-nz;
	    if ( sqrt ( dx*dx + dy*dy + dz*dz) < dTol ) {
		// We're with in the radius of dTol -- we got one!
		ids.push_back (ptNode->id);
	    }
	}
	
	uint Zero12 = depth % 3; // '% 3' for 3d replaces '% 2' of the quiz

	float flTarget[3] = {target.x, target.y, target.z};
	float tc = flTarget[Zero12];
	
	float flNodeP[3] = {ptNode->point.x, ptNode->point.y, ptNode->point.z};

	if ( flNodeP[Zero12] - tc < dTol ) {
	    searchHelper (ptNode->right, target, dTol, depth+1, ids, counter);
	} 
	if ( tc - flNodeP[Zero12] < dTol ) {
	    searchHelper (ptNode->left, target, dTol, depth+1, ids, counter);
	}

	return;
  }
};




