//
// Created by peaches on 3/14/15.
//

#ifndef _DM_MP_RRTDATA_H_
#define _DM_MP_RRTDATA_H_

#include <vector>
#include <memory>
#include "RRTConfig.h"

class RRTNode {
private:
    int _parent_id;
    std::vector<double> _config;
public:
    static int _currentID;
    const int _id;

    double dist_to(std::vector<double>* _values, RRTConfig* cfg);
    void print(std::ostream& sout);

    RRTNode(std::vector<double> _values, int _parent_id)
//        : _parent(_parent)
        : _id(++_currentID)
    {
        this->_parent_id = _parent_id;
        this->_config = _values;
    };
    std::vector<double>* get_config(){
        return &_config;
    }
};


class NodeTree {
private:
    RRTConfig* cfg;
    std::vector<RRTNode> _nodes;
//    std::unordered_map _node_map;

public:
//    typedef pair<const Key, T> value_type;

    NodeTree(RRTConfig* cfg) : cfg(cfg){};

    int add_node(std::vector<double> _values, int _parent);
    int add_node(std::vector<double> _values);
    bool delete_node(int _node);
    void print_tree(std::ostream& sout, int depth=5);
    int nearest_node(std::vector<double> _values);
    std::vector<RRTNode> get_nodes() {
        return _nodes;
    }
};

#endif //_DM_MP_RRTDATA_H_
