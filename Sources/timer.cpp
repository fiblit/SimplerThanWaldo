#include "timer.h"

using namespace std;

namespace timer {
    std::vector<std::pair<Time, std::string>> time_layers;

    void init_layers(int amount) {
        time_layers = vector<pair<Time, string>>(amount);
        for (int i = 0; i < amount; i++)
            time_layers[i] = make_pair(Clock::now(), "");
    }

    void start(int layer_i, std::string name) {
        if ((layer_i >= 0) && (layer_i < time_layers.size()) && (time_layers[layer_i].second.empty())) {
            for (int i = 0; i < layer_i; i++)
                cout << "\t";
            cout << name << "\n";
            time_layers[layer_i] = make_pair(Clock::now(), name);
        }
        else {
            cout << "ERROR: Failed to create timer " << name << " :: Layer " << layer_i << " does not exist.\n";
        }
    }

    void stop(int layer_i) {
        if ((layer_i >= 0) && (layer_i < time_layers.size()) && (!time_layers[layer_i].second.empty())) {
            Time end_time = Clock::now();
            for (int i = 0; i < layer_i; i++)
                cout << "\t";
            long long dt = chrono::duration_cast<chrono::nanoseconds>(end_time - time_layers[layer_i].first).count();
            cout << " in "
                << dt / 1000000.
                << " ms [" + time_layers[layer_i].second + "]\n";
            time_layers[layer_i].second = "";
        }
    }
}