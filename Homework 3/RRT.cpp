#include <random>
#include <unistd.h>

#include "RRT.h"
#include "RRTConfig.h"
#include "RRTData.h"

#include <openrave/viewer.h>

using namespace std;

vector<OpenRAVE::GraphHandlePtr> lineHandles;
vector<OpenRAVE::GraphHandlePtr> handles;
OpenRAVE::GraphHandlePtr goal_handle;
vector<float> points;

/*
*  Simple constructor for a new RRT object. Provides RRT-Connect searching for
*  N-DOF serial manipulators.
*
*  @param cfg A pointer to an RRTConfig object, providing the numerous parameters
*       for an RRT search.
*/
RRT::RRT(RRTConfig *cfg)
        : sout(cfg->sout), cfg(cfg), root(NodeTree(cfg)) {
    time_t seed = time(NULL);
    _rand = std::default_random_engine(1427255970);
    cout << "Random Seed: " << seed;
}


/*
*  Perform the RRT-Connect search for given start and goal configurations.
*  Connect functions will use the given step_size, and sample the goal
*  configuration with the given frequency, goal_freq.
*
*  @param _start_cfg A vector of doubles, the starting joint values for the
*       robots' active DOF
*  @param _goal_cfg A vector of doubles, the goal joint values for the
*       robots' active DOF
*  @param step_size The size (in joint space) for the RRT-Connect steps
*  @param goal_freq Percent of random samples which will sample the goal.
*       Should be 0 < goal_freq < 1 to ensure probabilistic completeness
*  @return A vector of RRTNode objects, in the desired order.
*/
pair<vector<RRTNode>, vector<RRTNode> > RRT::do_search(vector<double> _start_cfg, vector<double> _goal_cfg,
        double step_size, double goal_freq) {
    std::vector<RRTNode> path;
    start_config = _start_cfg;
    goal_config = _goal_cfg;

    // Start from the starting configuration
    root.add_node(start_config);

    int latest_id; // The most recently added node
    vector<double> smp;
    std::pair<bool, vector<double> > smp_pair = pair<bool, vector<double>>(true, _goal_cfg);
    int iter = 1;
    for (iter = 1; iter < cfg->max_iter; iter++) {
        // Sample the space for non-colliding point= sample(goal_freq, true);
        smp = smp_pair.second;

        // Find the ID of the nearest node to the sample.
        int nn = root.nearest_node(smp);

        cfg->robot->SetActiveDOFValues(root.get_nodes()->at(root.nearest_node(goal_config)).get_config());
        OpenRAVE::RobotBase::ManipulatorPtr manip = cfg->robot->GetActiveManipulator();
        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
        vector<float> pt = {(float)trans.x, (float)trans.y, (float)trans.z};
        goal_handle = cfg->env->plot3( &pt[0], 1, sizeof(float)*3, 8.0, OpenRAVE::RaveVector<float>(0.5,1,0.5,1) );

        // Run RRT-Connect, trying to connect the tree to this new sample
        pair<bool, RRTNode> connect_result = connect(nn, smp, step_size);

        // Update the last added node
        latest_id = connect_result.second._id;

        cfg->robot->SetActiveDOFValues(connect_result.second.get_config());
        manip = cfg->robot->GetActiveManipulator();
        trans = manip->GetEndEffectorTransform().trans;
        points.push_back(trans.x);
        points.push_back(trans.y);
        points.push_back(trans.z);
        if(iter%50 == 0 || false){
            cout << "Iter: " << iter << " Nodes: " << root.get_nodes()->size()
                    << " Goal? " << smp_pair.first << " Connected? " << connect_result.first << endl;
            handles.push_back(cfg->env->plot3(&points[0], points.size()/3, sizeof(float)*3, 2.0));
            points.clear();
        }


        // If we connected, and the sample was the goal, break! We found a path!
        if (smp_pair.first && connect_result.first) {
            break;
        } // Else repeat until max_iter is reached

        smp_pair = sample(goal_freq, true);
    }


    if (iter == cfg->max_iter) {
        cout << "Maximum number of Iterations reached. Goal not found." << endl;
    } else {
        cout << "Goal Reached!" << endl;
        cout << "Path:" << endl;
        // Build the path, in reverse at first. Goal --> start
        while (true) {
//            cout << latest_id->_id << ": ";
            path.push_back(root.get_nodes()->at(latest_id));
//            for(auto joint_val : latest_id->get_config()) {
//                cout << joint_val << ", ";
//            }
//            cout << endl;
            if (latest_id == 0) {
                break;
            }
            latest_id = root.get_nodes()->at(root.get_nodes()->at(latest_id)._parent_id)._id;
        }
    }
    cout << endl;

    cout << "Start " << root.get_nodes()->at(0)._id << ": ";
    for(auto node : root.get_nodes()->at(0).get_config()){
        cout << node << ", ";
    }
    cout << endl;

//    for(auto node : root.get_nodes()) {
//        cout << node._id << " -> " << node._parent_id << " : ";
//        for(auto joint_val : node.get_config()) {
//            cout << joint_val << ", ";
//        }
//        cout << endl;
//    }

    // Reverse the path. Now start --> goal.
    vector<RRTNode> path_fwd;
    vector<RRTNode> path_smooth;
    for(long i = path.size(); i-- > 0;) {
        path_fwd.push_back(path.at(i));
        path_smooth.push_back(path.at(i));
    }
    for(auto h : handles) {
        h->SetShow(false);
    }
    goal_handle->SetShow(false);
    handles.empty();

    smooth(path_smooth, step_size);

    return pair<vector<RRTNode>, vector<RRTNode> >(path_fwd, path_smooth);
}


/*
*  Sample the robot's reachable C-space, considering collisions, and joint
*  limitations. This function will uniformly select new samples until a
*  non-colliding sample is found. This process may take several iterations
*  (optional)
*
*  @param goal_freq Probability that this function will immediately return
*       the goal configuration.
*  @param non_collide Boolean indicating wether or not the function should
*       re-sample upon collision
*  @return A std::pair, with a boolean and vector of doubles. The boolean
*       (first) indicates wether or not the goal was sampled, while the
*       vector (second) contains the sampled point.
*/
std::pair<bool, vector<double> > RRT::sample(double goal_freq, bool non_collide) {
    vector<double> sample;
    int samples = 0;

    if (goal_rand(_rand) < goal_freq) {
        return std::pair<bool, vector<double> >(true, goal_config);
    }

    do {
        sample.clear();
        for (uint i = 0; i < cfg->c_dim; i++) {
            sample.push_back(this->dRand(cfg->j_min.at(i), cfg->j_max.at(i)));
        }
        samples++;
    } while (non_collide && collides(sample));
    return std::pair<bool, vector<double> >(false, sample);
}


/*
*  Provides the RRT "connect" functionality. Samples points (in joint
*  space) in a line from the start node with the given ID, nn_id, to the
*  sampled point, smp. Steps are taken with the given step size, step_size
*
*  @param goal_freq Probability that this function will immediately return
*       the goal configuration.
*  @param non_collide Boolean indicating wether or not the function should
*       re-sample upon collision
*  @return A std::pair, with a boolean and vector of doubles. The boolean
*       (first) indicates wether or not the goal was sampled, while the
*       vector (second) contains the sampled point.
*/
pair<bool, RRTNode> RRT::connect(int nn_id, vector<double> smp, double step_size, bool connect) {

    vector<double> start = root.get_nodes()->at(nn_id).get_config();
    int latest = root.get_nodes()->at(nn_id)._id;

    double axis_dist = step_size / root.get_nodes()->at(latest).dist_to(&smp, cfg);
    int step_count = (int) floor(1.0 / axis_dist);

    vector<double> step_vec;
    for (uint i = 0; i < cfg->c_dim; i++) {
        step_vec.push_back((smp.at(i) - root.get_nodes()->at(latest).get_joint(i)) * axis_dist);
    }

    vector<double> current_step = root.get_nodes()->at(nn_id).get_config();

    int steps;
    for (steps = 1; steps < step_count; steps++) {
        current_step.clear();

        for (uint i = 0; i < cfg->c_dim; i++) {
            current_step.push_back(start.at(i) + step_vec.at(i)*steps);
        }

        if (!collides(current_step)) {
            if(connect){
                int latest_id = root.add_node(current_step, root.get_nodes()->at(latest)._id);

                cfg->robot->SetActiveDOFValues(current_step);
                OpenRAVE::RobotBase::ManipulatorPtr manip = cfg->robot->GetActiveManipulator();
                OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
                points.push_back(trans.x);
                points.push_back(trans.y);
                points.push_back(trans.z);
                latest = root.get_nodes()->at(latest_id)._id;
            }
        } else {
            break;
        }
    }

//    cout << "  Connecting steps: " << steps << " / " << step_count << endl;

    // Last step_size should never collide
    bool success;
    if (steps == step_count) {
        if(connect) {
            latest = root.add_node(smp, root.get_nodes()->at(latest)._id);
//            latest = root.get_nodes()->at(latest_id)._id;
        }
        success = true;
    } else {
        success = false;
    }

    if(connect) {
        return pair<bool, RRTNode>(success, root.get_nodes()->at(latest));
    } else {
        return pair<bool, RRTNode>(success, root.get_nodes()->at(0));
    }
}

/*
*  Sets the robot's active DOF values to the given joint values, and
*  returns a boolean where TRUE indicates a collision. NOTE: This
*  function does NOT return the active DOF to their original position
*  (for speed)
*
*  @param _joints The vector of joint values to be tested.
*  @return A boolean, TRUE if given configuration collides.
*/
bool RRT::collides(vector<double> _joints) {

    // TODO: DELETE THIS
//    return false;
    cfg->robot->SetActiveDOFValues(_joints);
    return cfg->env->CheckCollision(cfg->robot) || cfg->robot->CheckSelfCollision();
}

pair<bool, vector<RRTNode> > RRT::connect_points(vector<double> start, vector<double> end, double step_size, RRTConfig* cfg, bool check=false){
    double axis_dist = step_size / RRTNode(start, -1).dist_to(&end, cfg);
    int step_count = (int) floor(1.0 / axis_dist);

    vector<double> step_vec;
    for (uint i = 0; i < cfg->c_dim; i++) {
        step_vec.push_back((end.at(i) - start.at(i)) * axis_dist);
    }

    vector<double> current_step = start;

    vector<RRTNode> points;
    int steps;
    for (steps = 1; steps < step_count; steps++) {
        current_step.clear();

        for (uint i = 0; i < cfg->c_dim; i++) {
            current_step.push_back(start.at(i) + step_vec.at(i)*steps);
        }


        if (check && collides(current_step)) {
            return pair<bool, vector<RRTNode> >(false, vector<RRTNode>());
        }

        RRTNode new_node(current_step, -1);
        points.push_back(new_node);
        root.get_nodes()->push_back(new_node);
    }
    return pair<bool, vector<RRTNode> >(true, points);
}

typedef std::uniform_int_distribution<int> int_dist;
void RRT::smooth(std::vector<RRTNode>& path, double step_size){
    for(int i = 0; i < 200; i++) {
        int_dist goal_rand = int_dist(0.0, path.size()-1);

        int idx_a = goal_rand(_rand);
        int idx_b = goal_rand(_rand);
        if(idx_a > idx_b){
            int tmp = idx_a;
            idx_a = idx_b;
            idx_b = tmp;
        }

//        cout << "Checking: " << idx_a << " " << idx_b << endl;
        vector<double> start = path.at(idx_a).get_config();
        vector<double> end = path.at(idx_b).get_config();

        vector<float> pline;
        if(connect_points(start, end, step_size, cfg, true).first) {
//            cout << "Can connect! " << idx_a << " " << idx_b << endl;

//            cfg->robot->SetActiveDOFValues(start);
//            OpenRAVE::RobotBase::ManipulatorPtr manip = cfg->robot->GetActiveManipulator();
//            OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
//            pline.push_back(trans.x);
//            pline.push_back(trans.y);
//            pline.push_back(trans.z);
//            cfg->robot->SetActiveDOFValues(end);
//            trans = manip->GetEndEffectorTransform().trans;
//            pline.push_back(trans.x);
//            pline.push_back(trans.y);
//            pline.push_back(trans.z);
//            lineHandles.push_back(cfg->env->drawlinestrip(&pline[0], 2, sizeof(float)*3, 2.0, OpenRAVE::RaveVector<float>(0.0, 0.0, 1.0, 1)));
//            pline.clear();

            pair<bool, vector<RRTNode> > segment = connect_points(start, end, step_size, cfg);
//            cout << "  Build segment of length " << segment.second.size() << endl;

            path.erase(path.begin()+idx_a, path.begin()+idx_b);
//            cout << "  Erased old!" << endl;

            path.insert(path.begin()+idx_a, segment.second.begin(), segment.second.end());
//            cout << "  Inserted new!" << endl;
//            cout << "  New Length: " << path.size() << endl;

        } else {
//            cout << "Can't connect!" << endl;
        }
    }
}