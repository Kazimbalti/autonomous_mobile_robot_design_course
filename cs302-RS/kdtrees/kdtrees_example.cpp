//  CS302: Data Structures using C++
//  Robotics Seminar
//  KD Trees example

#include<bits/stdc++.h>  // not good practice but OK for class purposes
using namespace std; 

const int k = 2; 

// Node presentation of the kd-tree
struct Node 
{ 
	int point[k]; // store k-dimensional point
	Node *left, *right;     // point to left/right child
}; 

// Creating of a kd-tree node
struct Node* newNode(int arr[]) 
{ 
	struct Node* temp = new Node; 

	for (int i=0; i<k; i++) 
	temp->point[i] = arr[i]; 

	temp->left = temp->right = NULL;  // initialized with no children
	return temp; 
} 

//  Insertion: inserts a new node and returns the root of the modified tree
//  The depth parameter is used to decide the axis of comparison
Node *insertRec(Node *root, int point[], unsigned depth) 
{ 
	// check if the tree is empty
	if (root == NULL) 
	return newNode(point); 

	// Calculate current dimension (cd) of comparison 
	unsigned cd = depth % k; 

	// Compare the new point with the root on current dimension ('cd')
	// and decide the left or right subtree
	if (point[cd] < (root->point[cd])) 
		root->left = insertRec(root->left, point, depth + 1); 
	else
		root->right = insertRec(root->right, point, depth + 1); 

	return root; 
} 

// Insertion: Insert a new point with given point parameters in
//  KD tree and return the new root. It employes recursive calls 
//  of insertRec
Node* insert(Node *root, int point[]) 
{ 
	return insertRec(root, point, 0); 
} 

//  Utility method: Determine if two points are the 
//  same inside a KD-dimensional space 
bool arePointsSame(int point1[], int point2[]) 
{ 
	// Compare individual point coord values 
	for (int i = 0; i < k; ++i) 
		if (point1[i] != point2[i]) 
			return false; 

	return true; 
} 

// Search: Is a point ("point[]") inside the kd-tree? 
// The depth parameter is used to determine the current axis. 
bool searchRec(Node* root, int point[], unsigned depth) 
{ 
	// Base cases 
	if (root == NULL) 
		return false; 
	if (arePointsSame(root->point, point)) 
		return true; 

	// Current dimension is computed using current depth 
    // and total dimensions (k) 
	unsigned cd = depth % k; 

	// Compare point with root with respect to 'cd' 
	if (point[cd] < root->point[cd]) 
		return searchRec(root->left, point, depth + 1); 

	return searchRec(root->right, point, depth + 1); 
} 

// Search: Is a Point in the kd-tree? 
// Relies on searchRec() 
bool search(Node* root, int point[]) 
{ 
	// Pass current depth as 0 
	return searchRec(root, point, 0); 
} 

// Test program 
int main() 
{ 
	struct Node *root = NULL; 
	int points[][k] = {{1, 6}, {17, 15}, {13, 15}, {6, 12}, 
					{9, 1}, {2, 7}, {10, 20}}; 

	int n = sizeof(points)/sizeof(points[0]); 

	for (int i=0; i<n; i++) 
	root = insert(root, points[i]); 

	int point1[] = {10, 20}; 
	(search(root, point1))? cout << "Point Found in KD-Tree \n": cout << "Point Not Found in KD-Tree\n"; 

	int point2[] = {5, 18}; 
	(search(root, point2))? cout << "Point Found in KD-Tree \n": cout << "Point Not Found in KD-Tree\n"; 

	return 0; 
} 
