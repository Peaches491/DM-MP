#include <random>
#include "RRT.h"

using namespace std;

RRT::RRT(RRTConfig* cfg)
    : sout(cfg->sout)
    , cfg(cfg)
    , root(NodeTree(cfg)){
}

vector<RRTNode*> RRT::do_search(vector<double> start_config, vector<double> _goal_cfg,
        double step_size, double goal_freq) {
    std::vector<RRTNode*> path;

    root.add_node(start_config);
    vector<double> smp;
    for(int i = 0; i < 100; i++) {
        smp = sample();
        int nn = root.nearest_node(smp);
        sout << "  " << root.add_node(smp, nn) << "-NN: " << nn;

    }

    sout << endl;
    root.print_tree(sout, 5);

//    sout << endl << "Generating random samples... " << endl;

//    for(int i = 0; i < 5; i++){
//        vector<double> smp = sample();
//        sout << rrt.add_node(smp) << ": ";
//        for(auto j : smp) {
//            sout << j << ", ";
//        }
//        sout << endl;
//    }
//
//    sout << "Checking Distances" << endl;
//    for(int i = 0; i < 5; i++){
//        vector<double> smp = sample();
//        sout << rrt.nearest_node(smp)->_id << ": ";
//        for(auto j : smp) {
//            sout << j << ", ";
//        }
//        sout << endl;
//        for(auto node : rrt.get_nodes()) {
//            sout << "  " << node._id << ": " << node.dist_to(&smp, cfg) << endl;
//        }
//    }

    return path;
}

vector<double> RRT::sample() {
    vector<double> sample;
    for(uint i = 0; i < cfg->c_dim; i++){
        if(cfg->ident.at(i)) {
//            sample.push_back(fmod(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)), M_PI));
            sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
        } else {
            sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
        }
    }
    return sample;
}