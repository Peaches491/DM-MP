//
// Created by peaches on 3/14/15.
//

#ifndef _DM_MP_RRTDATA_H_
#define _DM_MP_RRTDATA_H_

#include <vector>
#include <memory>

class RRTNode {
private:
    std::shared_ptr<RRTNode> _parent;
    std::vector<double> _config;
public:
    static int _currentID;
    const int _id;
    RRTNode(std::vector<double> _values, std::shared_ptr<RRTNode> _parent)
        : _parent(_parent)
        , _id(++_currentID)
    {
        _config = _values;
    };
    double dist_to(std::vector<double>* _values);
    std::vector<double>* get_config(){
        return &_config;
    }
};
typedef std::shared_ptr<RRTNode> RRTNodePtr;


class NodeTree {
private:
    std::vector<RRTNode> _nodes;
//    std::unordered_map _node_map;

public:
//    typedef pair<const Key, T> value_type;

    int add_node(std::vector<double> _values, RRTNodePtr _parent);
    int add_node(std::vector<double> _values);
    bool delete_node(RRTNodePtr _node);
    RRTNode* nearest_node(std::vector<double> _values);
};

#endif //_DM_MP_RRTDATA_H_
