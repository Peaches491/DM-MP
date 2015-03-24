#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include "RRTData.h"
#include "RRT.h"

using namespace std;

// Initialize the ID counter for RRTNode objects to zero.
int RRTNode::_currentID = 0;


/*
*  Calculate the Euclidian distance between this sample, and the given
*  sample. Limit the continuious joint distances to PI.
*
*  @param _values Pointer to a vector of joint values (doubles)
*  @param cfg Pointer to the RRTConfig object describing this RRT.
*  @return A double representing the Euclidian distance.
*/
double RRTNode::dist_to(vector<double>* _values, RRTConfig* cfg) {
    double sum = 0;
    for(uint i = 0; i < _config.size(); i++) {
        if(cfg->ident.at(i)) {
//            sum += pow(fmod((_config.at(i) - _values->at(i)), M_PI), 2.0);
            sum += pow(_config.at(i) - _values->at(i), 2.0);
        } else {
            sum += pow(_config.at(i) - _values->at(i), 2.0);
        }
    }
    return sqrt(sum);
}


/*
*  Print the value at this RRTNode
*/
void RRTNode::print() {
    if(_parent_id != 0){
        cout << _parent_id << "<-" << _id << ": ";
    } else {
        cout << _id << ": ";
    }
    for(auto j : this->_config) {
        cout << j << ", ";
    }
    cout << endl;
}


/*
*  Add a node to the node tree, with the given parent ID.
*
*  @param _values Pointer to a vector of joint values (doubles)
*  @param _parent_id ID of the parent node
*  @return The ID of the newly constructed node
*/
int NodeTree::add_node(vector<double> _values, int _parent_id) {
    RRTNode new_node = RRTNode(_values, _parent_id);
    _nodes.push_back(new_node);
//    cout << "Inserting " << _parent_id << "<-" << new_node._id << " ... ";
//    cout.flush();
//    cout << _values.size() << "  ";
//    cout.flush();
//    for(auto val : new_node.get_config()) {
//        cout << val << ", ";
//    }
//    kd_tree.insert(triplet(new_node.get_config(), new_node._id));
//    cout << "DONE." << endl;
    return new_node._id;
}


/*
*  Add a node to the node tree, with no parent.
*
*  @param _values Pointer to a vector of joint values (doubles)
*  @return The ID of the newly constructed node
*/
int NodeTree::add_node(vector<double> _values) {
    return add_node(_values, 0);
}


/*
*  Currently un-implemented. Not sure why it's requested in project assignment
*
*  @param _node ID of the node to be deleted
*  @return True if a node with the given ID existed. False otherwise.
*/
bool NodeTree::delete_node(int _node) {
    return false;
}


/*
*  Find the nearest neighbor to a given sample.
*  TODO: Currently uses naive search. Update to KD tree for (much) faster neighbor searches.
*
*  @param _values Pointer to a vector of joint values (doubles)
*  @return The ID of the nearest node.
*/
int NodeTree::nearest_node(vector<double> _values) {
    RRTNode* best = &_nodes.at(0);
    double best_dist = _nodes.at(0).dist_to(&_values, cfg);

    for(uint i = 1; i < _nodes.size(); i++) {
        double test_dist = _nodes.at(i).dist_to(&_values, cfg);
        if(test_dist < best_dist) {
            best = &_nodes.at(i);
            best_dist = test_dist;
        }
    }
    return best->_id;
}


/*
*  Print out the tree.
*  TODO: Implement print depth limitation.
*  NOTE: Is this possible with only parent pointers?
*
*  @param depth The max tree depth to be printed
*/
void NodeTree::print_tree(int depth) {
    for(auto node : _nodes){
        node.print();
    }
}