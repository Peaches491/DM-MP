//
// Created by peaches on 3/14/15.
//

#include <iostream>
#include <cmath>
#include <limits>
#include <vector>
#include <boost/smart_ptr/shared_ptr.hpp>
#include "RRTData.h"
#include "RRT.h"

using namespace std;

int RRTNode::_currentID = 0;

double RRTNode::dist_to(std::vector<double>* _values, RRTConfig* cfg) {
    double sum = 0;
    for(uint i = 0; i < _config.size(); i++) {
        if(cfg->ident.at(i)) {
            sum += pow(fmod((_config.at(i) - _values->at(i)), M_PI), 2.0);
//            sum += pow(_config.at(i) - _values->at(i), 2.0);
        } else {
            sum += pow(_config.at(i) - _values->at(i), 2.0);
        }
    }
    return sqrt(sum);
}

void RRTNode::print(ostream& sout) {
    if(_parent_id != 0){
        sout << _parent_id << "<-" << _id << ": ";
    } else {
        sout << _id << ": ";
    }
    for(auto j : this->_config) {
        sout << j << ", ";
    }
    sout << endl;
}

int NodeTree::add_node(std::vector<double> _values, int _parent_id) {
    RRTNode new_node = RRTNode(_values, _parent_id);
    _nodes.push_back(new_node);
    return new_node._id;
}

int NodeTree::add_node(std::vector<double> _values) {
    return add_node(_values, 0);
}

bool NodeTree::delete_node(int _node) {
    return false;
}

int NodeTree::nearest_node(std::vector<double> _values) {
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

void NodeTree::print_tree(std::ostream& sout, int depth) {
    for(auto node : _nodes){
        node.print(sout);
    }
}