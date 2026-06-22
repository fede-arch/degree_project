#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>
using namespace std;

struct DroneResult {
    string tag  = "";
    bool        done = false;
};

struct EpisodeSession {
    mutex               mtx;
    condition_variable  cv;
    int                 n_drones = 0;
    int                 n_done   = 0;
    vector<DroneResult> results;
};

static EpisodeSession g_session; 

static void parse_result(const string &data, DroneResult &r) {
    for (const auto &tag : {"arrived", "collision", "timeout"}) {
        if (data.find(tag) != string::npos) {
            r.tag = tag;
            return;
        }
    }
    r.tag = "UNKNOWN";
}

// MAIN
int main(int argc, char **argv) {
    if (argc < 2) {
        cerr << "Usage: MultiLrauv <n_drones>" << endl;
        return 1;
    }

    g_session.n_drones = stoi(argv[1]);
    g_session.results.resize(g_session.n_drones);

    gz::transport::Node node;

    for (int i = 0; i < g_session.n_drones; i++) {
        string topic = "/tethys_" + to_string(i) + "/es/episode_result";
        cout << "[MULTI] Listening on " << topic << endl;

        node.Subscribe<gz::msgs::StringMsg>(topic,
            [i](const gz::msgs::StringMsg &msg) {
                lock_guard<mutex> lock(g_session.mtx);
                if (g_session.results[i].done) return;
                parse_result(msg.data(), g_session.results[i]);
                g_session.results[i].done = true;
                g_session.n_done++;
                cout << "[MULTI] drone_" << i
                          << " -> " << g_session.results[i].tag << endl;
                if (g_session.n_done == g_session.n_drones)
                    g_session.cv.notify_all();
            });
    }

    cout << "[MULTI] Waiting for " << g_session.n_drones << " drones..." << endl;

    unique_lock<mutex> lock(g_session.mtx);
    g_session.cv.wait(lock, [] { return g_session.n_done == g_session.n_drones; });

    int arrived = 0, collision = 0, timeout = 0;
    for (auto &r : g_session.results) {
        if      (r.tag == "arrived")   arrived++;
        else if (r.tag == "collision") collision++;
        else                           timeout++;
    }

    cout << "\n========== RISULTATI ==========" << endl;
    cout << "Droni totali : " << g_session.n_drones << endl;
    cout << "ARRIVED      : " << arrived    << endl;
    cout << "COLLISION    : " << collision  << endl;
    cout << "TIMEOUT      : " << timeout    << endl;
    cout << "Success rate : "
              << (100.0 * arrived / g_session.n_drones) << "%" << endl;

    return 0;
}