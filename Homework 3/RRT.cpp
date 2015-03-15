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

    return path;
}

vector<double> RRT::sample(bool non_collide) {
//    cfg->sout << "Taking Sample" << endl;
    vector<double> sample;
    int samples = 0;
    do{
        sample.clear();
        for(uint i = 0; i < cfg->c_dim; i++){
            sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
//            if(cfg->ident.at(i)) {
//                sample.push_back(fmod(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)), M_PI));
//                sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
//            } else {
//                sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
//            }
        }
        samples++;
    } while(non_collide && collides(sample));
//    sout << "Samples taken: " << samples << endl;
    return sample;
}

bool RRT::collides(vector<double> _joints) {
//    cfg->sout << "Checking" << endl;
    cfg->robot->SetActiveDOFValues(_joints);
    // TODO Always collides? =[
    return cfg->env->CheckCollision(cfg->robot);
}