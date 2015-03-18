#include <random>
#include "RRT.h"

using namespace std;

RRT::RRT(RRTConfig* cfg)
    : sout(cfg->sout)
    , cfg(cfg)
    , root(NodeTree(cfg)){
}

vector<RRTNode*> RRT::do_search(vector<double> start_config, vector<double> _goal_cfg,
        double step_size, double goal_freq){
    std::vector<RRTNode*> path;
    goal_config = _goal_cfg;

    root.add_node(start_config);
    vector<double> smp;

    RRTNode* latest;
    int iter = 0;
    for(iter = 0; iter < cfg->max_iter; iter++) {
        std::pair<bool, vector<double> > smp_pair = sample(goal_freq, true);
        smp = smp_pair.second;
        int nn = root.nearest_node(smp);
        pair<bool, RRTNode> connect_result = connect(nn, smp, step_size);
        latest = &(connect_result.second);
        cout << "Iter: " << iter << " Goal? " << smp_pair.first << " Connected? " << connect_result.first << endl;
        if(smp_pair.first && connect_result.first) {
            break;
        }
    }

    if(iter == cfg->max_iter) {
        cout << "Maximum number of Iterations reached. Goal not found." << endl;
    } else {
        cout << "Goal Reached!" << endl;
        cout << "Path:" << endl;
        while(true) {
            cout << latest->_id << endl;
            path.push_back(latest);
            if(latest->_parent_id == 0) {
                break;
            }
            latest = &root.get_nodes().at(latest->_parent_id);
        }
    }
    cout << endl;
//    root.print_tree(5);

    return path;
}

std::pair<bool, vector<double> > RRT::sample(double goal_freq, bool non_collide) {
//    cfg->sout << "Taking Sample" << endl;
    vector<double> sample;
    int samples = 0;

    if(goal_rand(_rand) < goal_freq) {
        return std::pair<bool, vector<double> >(true, goal_config);
    }

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
    return std::pair<bool, vector<double> >(false, sample);
}

pair<bool, RRTNode> RRT::connect(int nn_idx, vector<double> smp, double step) {

//    cout << "CONNECTING " << nn_idx << " from " << root.get_nodes().size() << endl;
    int latest = root.get_nodes().at(nn_idx)._id;

    double axis_dist = step/root.get_nodes().at(latest).dist_to(&smp, cfg);
    int step_count = (int)floor(1.0/axis_dist);

    vector<double> step_vec;
    for(int i = 0; i < cfg->c_dim; i++) {
//        cout << "Axis " << i << endl;
//        cout << "  (" << smp.size() << ") Sample " << smp.at(i) << endl;
//        cout << "  (" << root.get_nodes().at(latest).get_config().size();
//        cout.flush();
//        cout << ") Latest " << root.get_nodes().at(latest).get_config().at(i) << endl;
        step_vec.push_back((root.get_nodes().at(latest).get_config().at(i) - smp.at(i))*axis_dist);
    }

    vector<double> current_step = smp;

    int steps = 0;
    for(steps = 0; steps < step_count; steps++) {
        for(int i = 0; i < cfg->c_dim; i++) {
            current_step.at(i) = current_step.at(i) + step_vec.at(i);
        }
        if(!collides(current_step)) {
            int latest_id = root.add_node(current_step, root.get_nodes().at(latest)._id);
            latest = root.get_nodes().at(latest_id)._id;
        } else {
            break;
        }
    }

    // Last step should never collide
    if(steps == step_count) {
        int latest_id = root.add_node(current_step, root.get_nodes().at(latest)._id);
        latest = root.get_nodes().at(latest_id)._id;
        return pair<bool, RRTNode>(true, root.get_nodes().at(latest));
    } else {
        return pair<bool, RRTNode>(false, root.get_nodes().at(latest));
    }

}

bool RRT::collides(vector<double> _joints) {
    cfg->robot->SetActiveDOFValues(_joints);
    return cfg->env->CheckCollision(cfg->robot);
}