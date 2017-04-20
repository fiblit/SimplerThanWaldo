#ifndef TIMER_H
#define TIMER_H
#include <string>
#include <vector>
#include <chrono>
#include <iostream>


//this should really be an actual stack, but I'll just manually control it so I know I'm ending the right layer
namespace timer {
    typedef std::chrono::high_resolution_clock Clock;
    typedef std::chrono::steady_clock::time_point Time;

    static std::vector<std::pair<Time, std::string>> time_layers;
    static void init_layers(int amount) {
        using namespace std;

        time_layers = vector<pair<Time, string>>(amount);
        for (int i = 0; i < amount; i++)
            time_layers[i] = make_pair(Clock::now(), static_cast<string>(""));
    }

    static void start(int layer_i, std::string name) {
        using namespace std;

        if (layer_i >= 0 && layer_i < time_layers.size() && (time_layers[layer_i].second == static_cast<string>(""))) {
            for (int i = 0; i < layer_i; i++)
                cout << "\t";
            cout << name << "\n";
            time_layers[layer_i] = make_pair(Clock::now(), name);
        }
        else
            cout << "ERROR: Failed to create timer " << name << " :: Layer " << layer_i << " does not exist.";
    }

    static void stop(int layer_i) {
        using namespace std;

        if (layer_i >= 0 && layer_i < time_layers.size() && (time_layers[layer_i].second != (std::string)"")) {
            Time end_time = Clock::now();
            for (int i = 0; i < layer_i; i++)
                cout << "\t";
            cout << " [" + time_layers[layer_i].second + "] in "
                << chrono::duration_cast<chrono::nanoseconds>(end_time - time_layers[layer_i].first).count() / 1000000.
                << " ms" << endl;
            time_layers[layer_i].second = "";
        }
    }
}

#endif//TIMER_H