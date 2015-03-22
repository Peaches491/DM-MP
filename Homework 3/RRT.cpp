#include <random>
#include <unistd.h>
#include "RRT.h"
#include <openrave/viewer.h>

using namespace std;

vector<OpenRAVE::GraphHandlePtr> handles;
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
vector<RRTNode> RRT::do_search(vector<double> _start_cfg, vector<double> _goal_cfg,
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

        // Run RRT-Connect, trying to connect the tree to this new sample
        pair<bool, RRTNode> connect_result = connect(nn, smp, step_size);

        // Update the last added node
        latest_id = connect_result.second._id;
        cout << "Iter: " << iter << " Nodes: " << root.get_nodes().size() << " Goal? " << smp_pair.first << " Connected? " << connect_result.first << endl;

//        std::vector< double > v;
//        cfg->robot->SetActiveDOFValues(connect_result.second.get_config());
//        cfg->robot->GetDOFValues(v);
//        cfg->robot->GetController()->SetDesired(v);
//        while(!cfg->robot->GetController()->IsDone()){
//            usleep(1000);
//        }


//        cfg->robot->SetActiveDOFValues(connect_result.second.get_config());
//        OpenRAVE::RobotBase::ManipulatorPtr manip = cfg->robot->GetActiveManipulator();
//        OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
//        points.push_back(trans.x);
//        points.push_back(trans.y);
//        points.push_back(trans.z);
        if(iter%20 == 0 || false){
            handles.clear();
            handles.push_back(cfg->env->plot3(&points[0], points.size()/3, sizeof(float)*3, 2.0));
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
            path.push_back(root.get_nodes().at(latest_id));
//            for(auto joint_val : latest_id->get_config()) {
//                cout << joint_val << ", ";
//            }
//            cout << endl;
            if (latest_id == 0) {
                break;
            }
            latest_id = root.get_nodes().at(root.get_nodes().at(latest_id)._parent_id)._id;
        }
    }
    cout << endl;

    cout << "Start " << root.get_nodes().at(0)._id << ": ";
    for(auto node : root.get_nodes().at(0).get_config()){
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
    for(ulong i = path.size(); i-- > 0; i) {
        path_fwd.push_back(path.at(i));
        cout << i << " : " << path.at(i)._id << ": ";
        for(auto joint_val : path.at(i).get_config()) {
            cout << joint_val << ", ";
        }
        cout << endl;

        usleep(1000 * 100);
    }
    return path_fwd;
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
pair<bool, RRTNode> RRT::connect(int nn_id, vector<double> smp, double step_size) {
//    cout << "CONNECTING " << nn_id << " from " << root.get_nodes().size() << endl;
    vector<double> start = root.get_nodes().at(nn_id).get_config();
    int latest = root.get_nodes().at(nn_id)._id;

    double axis_dist = step_size / root.get_nodes().at(latest).dist_to(&smp, cfg);
    int step_count = (int) floor(1.0 / axis_dist);

    vector<double> step_vec;
    for (int i = 0; i < cfg->c_dim; i++) {
        step_vec.push_back((smp.at(i) - root.get_nodes().at(latest).get_config().at(i)) * axis_dist);
    }

    vector<double> current_step = root.get_nodes().at(nn_id).get_config();

    int steps;
    for (steps = 1; steps < step_count; steps++) {
        current_step.clear();

//        cout << "  Step " << steps << ": " ;
        for (int i = 0; i < cfg->c_dim; i++) {
            current_step.push_back(start.at(i) + step_vec.at(i)*steps);
//            cout << current_step.back() << ", ";
        }
//        cout << endl;
        if (!collides(current_step)) {
            int latest_id = root.add_node(current_step, root.get_nodes().at(latest)._id);

            cfg->robot->SetActiveDOFValues(current_step);
            OpenRAVE::RobotBase::ManipulatorPtr manip = cfg->robot->GetActiveManipulator();
            OpenRAVE::RaveVector<OpenRAVE::dReal> trans = manip->GetEndEffectorTransform().trans;
            points.push_back(trans.x);
            points.push_back(trans.y);
            points.push_back(trans.z);
            latest = root.get_nodes().at(latest_id)._id;
        } else {
            break;
        }
    }


    cout << "  Connecting steps: " << steps << " / " << step_count << endl;

    // Last step_size should never collide
    if (steps == step_count) {
        int latest_id = root.add_node(smp, root.get_nodes().at(latest)._id);
        latest = root.get_nodes().at(latest_id)._id;
        return pair<bool, RRTNode>(true, root.get_nodes().at(latest));
    } else {
        return pair<bool, RRTNode>(false, root.get_nodes().at(latest));
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