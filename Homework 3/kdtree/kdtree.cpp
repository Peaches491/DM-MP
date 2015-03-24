#include "kdtree.h"
#include <iostream>
#include <random>
#include <ctime>
#include <queue>
#include <chrono>
#include <vector>
#include <assert.h>
#include <bits/stl_queue.h>

using namespace std;
using namespace KDTree;

time_t seed = time(NULL);
//long seed = 1427171576;
std::default_random_engine _rand = std::default_random_engine(seed);

double dRand(double fMin, double fMax) {
    std::uniform_real_distribution<double> unif(fMin, fMax);
    return unif(_rand);
}

std::vector<double> sample() {
    std::vector<double> sample;
    for (uint i = 0; i < 2; i++) {
        sample.push_back((int)dRand(0, 100));
    }
    return sample;
}

//////////////////////////////////////////////////////////

template<typename T>
void KDTreeRoot<T>::print() {
    cout << "A Tree!" << endl;
    root.print(0);
}

//////////////////////////////////////////////////////////


template<typename T>
void KDTreeNode<T>::add(std::vector<double> smp, T data) {
    int new_axis = (axis + 1) % (int) smp.size();
    if (smp[axis] < this->pos[axis]) {
        if (!left) {
            left = new KDTreeNode<T>(smp, data, new_axis);
        } else {
            left->add(smp, data);
        }
    } else {
        if (!right) {
            right = new KDTreeNode<T>(smp, data, new_axis);
        } else {
            right->add(smp, data);
        }
    }
}


template<typename T>
pair<double, KDTreeNode<T> *> KDTreeNode<T>::NN(vector<double> smp) {
    class mycomparison {
    public:
        mycomparison(){};
        bool operator() (const pair<KDTreeNode<T>*, double>& lhs, const pair<KDTreeNode<T>*, double>&rhs) const
        {
            return (lhs.second<rhs.second);
        }
    };

    double best_dist = numeric_limits<double>::max(); // smallest distance seen so far
    KDTreeNode<T>* best_node; // nearest neighbor so far
    priority_queue<pair<KDTreeNode<T>*, double>, vector<pair<KDTreeNode<T>*, double> >, mycomparison> PQ; // minimizing queue
    PQ.push(pair<KDTreeNode<T>*, double>(this, 0));
    int pops = 0;
    while (!PQ.empty()) {
        pair<KDTreeNode<T>*, double> top = PQ.top();
        PQ.pop();
        KDTreeNode<T>* node = top.first;
        double bound = top.second;
        pops++;
        if (bound >= best_dist) {
            cout << "Popped: " << pops;
            return pair<double, KDTreeNode<T> *>(best_dist, best_node);
        }
        double dist = node->dist_sq(&smp);

        if (dist < best_dist) {
            best_dist = dist;
            best_node = node;
        }
        if (smp[axis] < pos[axis]) {
            if(node->left) PQ.push(pair<KDTreeNode<T>*, double>(node->left, pos[axis] - node->pos[axis]));
            if(node->right) PQ.push(pair<KDTreeNode<T>*, double>(node->right, 0));
        } else {
            if(node->left) PQ.push(pair<KDTreeNode<T>*, double>(node->left, 0));
            if(node->right) PQ.push(pair<KDTreeNode<T>*, double>(node->right, pos[axis] - node->pos[axis]));
        }
    } // while
    cout << "Popped: " << pops;
    return pair<double, KDTreeNode<T> *>(best_dist, best_node);
}


template<typename T>
pair<double, KDTreeNode<T> *> KDTreeNode<T>::NN_naive(vector<double> smp) {
    double best_dist = numeric_limits<double>::max();
    KDTreeNode<T> *best_node;

    queue<KDTreeNode<T>* > q;
    q.push(this);
    while(!q.empty()) {
        KDTreeNode<T>* curr = q.front();
        double NN_dist = curr->dist_sq(&smp);
        if(best_dist > NN_dist) {
            best_dist = NN_dist;
            best_node = q.front();
        }
        if (curr->left) {
            q.push(curr->left);
        }
        if (curr->right) {
            q.push(curr->right);
        }
        q.pop();
    }
    return pair<double, KDTreeNode<T> *>(best_dist, best_node);
}

template<typename T>
void KDTreeNode<T>::print(int depth) {

    cout << this->axis << " ";
    for (auto val : this->pos) {
        cout << val << ", ";
    }
    cout << "-> " << this->data << endl;
    if (this->left) {
        for (int i = 0; i < depth; i++) {
            cout << "-";
        }
        cout << "L-";
        this->left->print(depth + 1);
    }
    if (this->right) {
        for (int i = 0; i < depth; i++) {
            cout << "-";
        }
        cout << "R-";
        this->right->print(depth + 1);
    }
}

//////////////////////////////////////////////////////////

int main(int argc, char *argv[]) {
    KDTree::KDTreeRoot<int> tree(sample(), 0);

    cout << "Random seed: " << seed << endl;

    for (int i = 1; i <= 10; i++) {
        tree.add(sample(), i);
    }

    tree.print();

    pair<double, KDTreeNode<int> *> naive;
    pair<double, KDTreeNode<int> *> NN;
    bool match;

    int nn_time = 0;
    int naive_time = 0;

    for(int i = 0; i < 100; i++){
        vector<double> smp = sample();

        if(i % 50 == 0) cout << "Iteration: " << i << endl;

        auto t3 = std::chrono::high_resolution_clock::now();
        naive = tree.NN_naive(smp);
        auto t4 = std::chrono::high_resolution_clock::now();
        naive_time += std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count();

        auto t1 = std::chrono::high_resolution_clock::now();
        NN = tree.NN(smp);
        auto t2 = std::chrono::high_resolution_clock::now();
        nn_time += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();

        cout << endl;

        assert(naive.first == NN.first);
        assert(naive.second == NN.second);

    }
    std::cout << "NN() took "
            << nn_time
            << " microseconds\n";
    std::cout << "Naive() took "
            << naive_time
            << " microseconds\n";

}
