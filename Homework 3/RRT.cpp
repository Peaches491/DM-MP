#include <random>
#include "RRT.h"

using namespace std;

RRT::RRT(std::vector<double> _j_min, std::vector<double> _j_max,
        std::vector<int> ident, ostream &sout)
    : j_min(_j_min), j_max(_j_max), ident(ident), c_dim(_j_max.size()), sout(sout) {
}

vector<RRTNodePtr> RRT::do_search(vector<double> start_config, vector<double> _goal_cfg,
        double step_size, double goal_freq) {
    std::vector<RRTNodePtr> path;

    sout << endl << "Generating random samples... " << endl;

    for(int i = 0; i < 10; i++){
        vector<double> smp = sample();
        sout << rrt.add_node(smp) << ": ";
        for(auto j : smp) {
            sout << j << ", ";
        }
        sout << endl;
    }

    sout << "Checking Distances" << endl;
    for(int i = 0; i < 10; i++){
        vector<double> smp = sample();
        sout << rrt.nearest_node(smp)->_id << ": ";
        for(auto j : smp) {
            sout << j << ", ";
        }
        sout << endl;
    }

    return path;
}

vector<double> RRT::sample() {
    vector<double> sample;
    for(uint i = 0; i < c_dim; i++){
        sample.push_back(this->dRand(j_min.at(i), j_max.at(i)));
    }
    return sample;
}