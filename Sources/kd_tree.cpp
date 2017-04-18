#include "kd_tree.h"
#include <random>

using namespace std;

kd_tree::kd_tree(int k, UnitCmp cmp) {
    this->k = k;
    //this->data = data;
    this->compare = cmp;
    this->left = nullptr;
    this->right = nullptr;
}

kd_tree::kd_tree(std::vector<Unit> origin, int k, UnitCmp cmp) {
    this->k = k;
    this->compare = cmp;
    build(origin, 0);
}

//needs to be redefined if Unit is redefined
bool kd_tree::leq(Unit a, Unit b, int dim) {
    return a.at<double>(dim, 0) <= b.at<double>(dim, 0);
}

int kd_tree::getPivot(std::vector<Unit> list, int axis) {
    //wikipedia suggested a random sampling's median wasn't half bad
    //even median of medians isn't perfect, so this is fine. :P
    const int samplesize = 11;
    std::vector<std::pair<Unit,int>> subsample;
    if (list.size() > samplesize) {
        //get 10 random points
        std::random_device rd;
        std::default_random_engine gen(rd());
        std::vector<int> selection;
        for (int i = 0; i < samplesize; i++) {
            std::uniform_int_distribution<int> dis(i, list.size() - 1);
            int r = dis(gen);
            for (int j = 0; j < i; j++) {//faster with a hash but IDGAF anymore
                if (selection[j] == r) {
                    r = j;
                    break;
                }
            }
            selection.push_back(r);
            subsample.push_back(pair<Unit,int>(list[r], r));
        }
    }
    else
        for (int i = 0; i < list.size(); i++)
            subsample.push_back(pair<Unit, int>(list[i], i));

    //get median of subsample (based on the axis-th dimension of Unit)
    std::sort(subsample.begin(), subsample.end(),
    [axis](const pair<Unit, int> & a, const pair<Unit, int> & b) -> bool {
        //this needs to be updated if Unit is updated
        return a.first.at<double>(axis, 0) > b.first.at<double>(axis, 0);
    });

    return subsample[subsample.size() / 2].second;
}

//pretty sure this is n log (n) as it will recurse log(n) times as the pivot ideally splits it
void kd_tree::build(std::vector<Unit> list, int axis) {
    //select pivot from list
    int pivot = getPivot(list, axis);

    this->data = list[pivot];

    vector<Unit> before, after;

    //O(n)
    for (Unit p : list)
        if (leq(p, this->data, axis))
            before.push_back(p);
        else
            after.push_back(p);

    this->left = new kd_tree(this->k, this->compare);
    this->left->build(before, (axis + 1) % this->k);

    this->right = new kd_tree(this->k, this->compare);
    this->right->build(after, (axis + 1) % this->k);
}

//https://en.wikipedia.org/wiki/K-d_tree#Nearest_neighbour_search
pair<kd_tree *, double> kd_tree::nn_search_(kd_tree::Unit p, int axis, kd_tree * t) {
    kd_tree * cur_best;
    double dist;

    //entry * find leaf node
    if (t->leq(p, t->data, axis)) {
        if (t->left == nullptr) {
            return pair<kd_tree *, double>(t, this->compare(p, t->data));
        }
        else {
            pair<kd_tree *, double> d = nn_search_(p, (axis + 1) % this->k, t->left);
            cur_best = d.first;
            dist = d.second;
        }
    }
    else {
        if (t->right == nullptr) {
            return pair<kd_tree *, double>(t, this->compare(p, t->data));
        }
        else {
            pair<kd_tree *, double> d = nn_search_(p, (axis + 1) % this->k, t->right);
            cur_best = d.first;
            dist = d.second;
        }
    }

    //exit & unwind step
    double d = this->compare(p, t->data);
    if (d < dist) {
        dist = d;
        cur_best = t;
    }

    double split_dist = t->data.at<double>(axis, 0) - p.at<double>(axis, 0);
    if (abs(split_dist) < dist) {
        kd_tree * next;
        if (split_dist < 0)
            next = t->left;//invert from where we came from
        else
            next = t->right;

        pair<kd_tree *, double> other_side =  nn_search_(p, (axis + 1) % this->k, next);
        if (other_side.second < dist) {
            dist = other_side.second;
            cur_best = other_side.first;
        }
    }

    return pair<kd_tree *, double>(cur_best, dist);
}

pair<kd_tree *, double> kd_tree::nn_search(kd_tree::Unit p) {
    return nn_search_(p, 0, this); 
}
