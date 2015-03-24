#ifndef _DM_MP_RRTDATA_H_
#define _DM_MP_RRTDATA_H_

#include <vector>
#include <memory>
#include "RRTConfig.h"

class RRTNode {
private:
    std::vector<double> _config;
public:
    static int _currentID;
    const int _id;
    int _parent_id;

    double dist_to(std::vector<double>* _values, RRTConfig* cfg, bool use_weights=false);
    void print();

    RRTNode(std::vector<double> _values, int _parent_id)
        : _id(_currentID++)
    {
        this->_parent_id = _parent_id;
        this->_config = _values;
    };
    std::vector<double> get_config(){
        return _config;
    }
    double get_joint(int joint_idx) {
        return _config[joint_idx];
    }
};



class NodeTree {
private:

    RRTConfig* cfg;
    std::vector<RRTNode> _nodes;

public:
    NodeTree(RRTConfig* cfg) : cfg(cfg) {
        _nodes.reserve(10000);
    };

    int add_node(std::vector<double> _values, int _parent);
    int add_node(std::vector<double> _values);
    bool delete_node(int _node);
    void print_tree(int depth=5);
    int nearest_node(std::vector<double> _values);
    std::vector<RRTNode>* get_nodes() {
        return &_nodes;
    }

};

#endif //_DM_MP_RRTDATA_H_
