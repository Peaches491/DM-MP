//
// Created by peaches on 3/14/15.
//

#include <iostream>
#include <cmath>
#include <limits>
#include "RRTData.h"

using namespace std;

int RRTNode::_currentID = 0;

double RRTNode::dist_to(std::vector<double>* _values) {
    double sum = 0;
    for(uint i = 0; i < _config.size(); i++) {
//        cout << "c: " << _config.at(i) << "  v: " << _values->at(i) << endl;
        sum += pow(_config.at(i) - _values->at(i), 2.0);
    }
    return sqrt(sum);
}

int NodeTree::add_node(std::vector<double> _values, RRTNodePtr _parent) {
    RRTNode new_node = RRTNode(_values, _parent);
    this->_nodes.push_back(new_node);
    return new_node._id;
}

int NodeTree::add_node(std::vector<double> _values) {
    RRTNode new_node = RRTNode(_values, NULL);
    this->_nodes.push_back(new_node);
    return new_node._id;
}

bool NodeTree::delete_node(RRTNodePtr _node) {
    return false;
}

RRTNode* NodeTree::nearest_node(std::vector<double> _values) {
    if(this->_nodes.size() == 0) return NULL;

    RRTNode* best = &this->_nodes.at(0);
    double best_dist = std::numeric_limits<double>::max();
    for(uint i = 1; i < _nodes.size(); i++) {
        double test_dist = best->dist_to(_nodes.at(i).get_config());
        std::cout << "dist: " << test_dist << std::endl;
        if(best_dist > test_dist) {
            best = &_nodes.at(i);
            best_dist = test_dist;
        }
    }

    return best;
}
