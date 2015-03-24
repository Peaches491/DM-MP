#ifndef _DM_MP_KDTREE_H_
#define _DM_MP_KDTREE_H_

#include <cstdlib>
#include <vector>
#include <cmath>

namespace KDTree{

    template <class T>
    class KDTreeNode {
    private:
        std::vector<double> pos;
        T data;
        int axis;
    public:
        KDTreeNode* left = NULL;
        KDTreeNode* right = NULL;

        T getData() {return data;};
        std::vector<double> getPos() {return pos;};

        KDTreeNode(std::vector<double> pos, T data, int axis)
                : pos(pos)
                , data(data)
                , axis(axis) {};

        double dist_sq(std::vector<double> *that) {
            double sum = 0;
            for(int i = 0; i < pos.size(); i++) {
                sum += (pos.at(i)-that->at(i))*(pos.at(i)-that->at(i));
            }
//            return sqrt(sum);
            return sum;
        };


        void add(std::vector<double> smp, T data);
        void print(int depth);
        std::pair<double, KDTreeNode<T>*> NN(std::vector<double> smp);
        std::pair<double, KDTreeNode<T>*> NN_naive(std::vector<double> smp);
    };


    template <class T>
    class KDTreeRoot {
    private:
        KDTreeNode<T> root;

    protected:
        int dimension;

    public:
        KDTreeRoot(std::vector<double> root_pos, T root_data)
            : root(KDTreeNode<T>(root_pos, root_data, 0))
            , dimension((int)root_pos.size()) {
        }
        void add(std::vector<double> pos, T data){
            root.add(pos, data);
        }

        void print();
        std::pair<double, KDTreeNode<T> *> NN(std::vector<double> smp){
            return root.NN(smp);
        }
        std::pair<double, KDTreeNode<T> *> NN_naive(std::vector<double> smp){
            return root.NN_naive(smp);
        }

    };


    template<typename T>
    using KDTreeNodePtr = KDTreeNode<T>*;


}

#endif //_DM_MP_KDTREE_H_
