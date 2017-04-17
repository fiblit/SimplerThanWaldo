#ifndef KD_TREE_H
#define KD_TREE_H

#include <opencv2/opencv.hpp>
#include <vector>

class kd_tree {
public:
    typedef cv::Mat Unit;
    typedef double(*UnitCmp)(Unit, Unit);

    // or double *? double[]? std::vector<double>?

    kd_tree(int k, UnitCmp cmp);
    kd_tree(std::vector<Unit> origin, int k, UnitCmp cmp);
    void build(std::vector<Unit> list, int axis);
    Unit nn_search(Unit p);
private:
    int getPivot(std::vector<Unit> list, int axis);
    bool leq(Unit a, Unit b, int dim);

    std::pair<kd_tree *, double> kd_tree::nn_search_recurse_to_bot(kd_tree::Unit p, int axis, kd_tree * t);

    //should be low if similar (like a distance function)
    UnitCmp compare;
    Unit data;
    int k;//dimensions
    kd_tree * left;
    kd_tree * right;
};

#endif//KD_TREE_H