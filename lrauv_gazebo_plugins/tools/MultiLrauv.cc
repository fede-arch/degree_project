#include <iostream>
#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>
#include <gz/transport/Node.hh>
#include <gz/msgs/stringmsg.pb.h>

static std::mutex              g_mtx;
static std::condition_variable g_cv;
static int                     g_n_drones = 0;
static int                     g_n_done   = 0;

struct DroneResult {
    std::string tag  = "";
    bool        done = false;
};

static std::vector<DroneResult> g_results;
static void parse_result(const std::string &data, DroneResult &r) {
    for (const auto &tag : {"ARRIVED", "COLLISION", "TIMEOUT"}) {
        if (data.find(tag) != std::string::npos) {
            r.tag = tag;
            return;
        }
    }
    r.tag = "UNKNOWN";
}

// MAIN
int main(int argc, char **argv) {
    if (argc < 2) {
        std::cerr << "Usage: MultiLrauv <n_drones>" << std::endl;
        return 1;
    }

    g_n_drones = std::stoi(argv[1]);
    g_results.resize(g_n_drones);

    gz::transport::Node node;

    for (int i = 0; i < g_n_drones; i++) {
        std::string topic = "/tethys_" + std::to_string(i) + "/es/episode_result";
        std::cout << "[MULTI] Listening on " << topic << std::endl;

        node.Subscribe<gz::msgs::StringMsg>(topic,
            [i](const gz::msgs::StringMsg &msg) {
                std::lock_guard<std::mutex> lock(g_mtx);
                if (g_results[i].done) return;
                parse_result(msg.data(), g_results[i]);
                g_results[i].done = true;
                g_n_done++;
                std::cout << "[MULTI] drone_" << i
                          << " -> " << g_results[i].tag << std::endl;
                if (g_n_done == g_n_drones)
                    g_cv.notify_all();
            });
    }

    std::cout << "[MULTI] Waiting for " << g_n_drones << " drones..." << std::endl;

    std::unique_lock<std::mutex> lock(g_mtx);
    g_cv.wait(lock, [] { return g_n_done == g_n_drones; });

    int arrived = 0, collision = 0, timeout = 0;
    for (auto &r : g_results) {
        if      (r.tag == "ARRIVED")   arrived++;
        else if (r.tag == "COLLISION") collision++;
        else                           timeout++;
    }

    std::cout << "\n========== RISULTATI ==========" << std::endl;
    std::cout << "Droni totali : " << g_n_drones << std::endl;
    std::cout << "ARRIVED      : " << arrived    << std::endl;
    std::cout << "COLLISION    : " << collision  << std::endl;
    std::cout << "TIMEOUT      : " << timeout    << std::endl;
    std::cout << "Success rate : "
              << (100.0 * arrived / g_n_drones) << "%" << std::endl;

    return 0;
}