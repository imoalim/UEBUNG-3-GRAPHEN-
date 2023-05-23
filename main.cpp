#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <unordered_map>
#include <queue>
#include <limits>
#include <iomanip>
#include <algorithm>

struct Station {
    std::string name;
    int cost;

    Station(const std::string& name, int cost) : name(name), cost(cost) {}
};

using Graph = std::unordered_map<std::string, std::vector<Station>>;

void readGraphFromFile(const std::string& filename, Graph& graph) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error opening file " << filename << std::endl;
        return;
    }
    std::string line;

    while (std::getline(file, line)) {
        std::istringstream iss(line);
        std::string lineName;
        std::getline(iss, lineName, ':');

        std::vector<Station> stations;

        std::string stationName;
        int cost;
        char delimiter;

        while (iss >> std::quoted(stationName) >> cost >> delimiter) {
            // Remove quotes from station name
            stationName = stationName.substr(1, stationName.size() - 2);
            stations.emplace_back(stationName, cost);
        }

        graph[lineName] = stations;
    }
    file.close();
}



std::vector<std::string> findShortestPath(const Graph& graph, const std::string& start, const std::string& goal) {
    std::unordered_map<std::string, int> distances;
    std::unordered_map<std::string, std::string> previous;
    std::priority_queue<std::pair<int, std::string>, std::vector<std::pair<int, std::string>>, std::greater<>> pq;

    // Check that start and goal stations exist in the graph
    if (graph.find(start) == graph.end() || graph.find(goal) == graph.end()) {
        std::cerr << "Start or goal station not found in graph." << std::endl;
        return {};
    }

    // Initialize distances
    for (const auto& entry : graph) {
        const std::string& station = entry.first;
        distances[station] = std::numeric_limits<int>::max();
        previous[station] = "";
    }

    distances[start] = 0;
    pq.emplace(0, start);

    while (!pq.empty()) {
        const std::string currentStation = pq.top().second;
        pq.pop();

        if (currentStation == goal) {
            // Destination reached, build and return the path
            std::vector<std::string> path;
            std::string station = goal;

            while (!station.empty()) {
                path.insert(path.begin(), station);
                station = previous[station];
            }

            return path;
        }

        const int currentDistance = distances[currentStation];

        for (const auto& station : graph.at(currentStation)) {
            const int newDistance = currentDistance + station.cost;

            if (newDistance < distances[station.name]) {
                distances[station.name] = newDistance;
                previous[station.name] = currentStation;
                pq.emplace(newDistance, station.name);
            }
        }
    }

    std::cerr << "No path found between " << start << " and " << goal << std::endl;
    return {}; // No path found
}

void printPath(const std::vector<std::string>& path, const Graph& graph){
    if (path.empty()) {
        std::cout << "No path found." << std::endl;
        return;
    }

    std::string currentLine;
    std::string prevLine;
    int totalCost = 0;

    for (size_t i = 1; i < path.size(); ++i) {
        const std::string &currentStation = path[i];
        const std::string &prevStation = path[i - 1];

        // Check that current and previous stations exist in the graph
        if (graph.find(currentStation) == graph.end() || graph.find(prevStation) == graph.end()) {
            std::cerr << "Invalid station in path: " << prevStation << " -> " << currentStation << std::endl;
            return;
        }

        for (const auto &entry: graph) {
            const std::string &lineName = entry.first;
            const std::vector<Station> &stations = entry.second;

            auto currentStationIt = std::find_if(stations.begin(), stations.end(),
                                                 [&currentStation](const Station &station) {
                                                     return station.name == currentStation;
                                                 });

            auto prevStationIt = std::find_if(stations.begin(), stations.end(),
                                              [&prevStation](const Station &station) {
                                                  return station.name == prevStation;
                                              });

            if (currentStationIt != stations.end() && prevStationIt != stations.end()) {
                if (lineName != currentLine) {
                    if (!currentLine.empty()) {
                        std::cout << "Line " << lineName << ":" << std::endl;
                        currentLine = lineName;
                    }

                    totalCost += currentStationIt->cost;
                    std::cout << currentStation << std::endl;
                }
            }
        }

        // Print the total cost and the closing code block
        std::cout << "Total cost: " << totalCost << std::endl;
        std::cout << " " << std::endl;
    }

}


int main(int argc, char* argv[]) {
    if(argc != 4) {
        std::cerr << "Usage: " << argv[0] << " <filename> <start_station> <goal_station>" << std::endl;
        return 1;
    }

    const std::string filename = argv[1];
    const std::string start = argv[2];
    const std::string goal = argv[3];

    Graph graph;
    readGraphFromFile(filename, graph);

    // Print out the contents of the graph
    /*for (auto const& [lineName, stations] : graph) {
        std::cout << lineName << ": ";
        for (auto const& station : stations) {
            std::cout << station.name << " " << station.cost << ", ";
        }
        std::cout << std::endl;
    }*/

    std::vector<std::string> path = findShortestPath(graph, start, goal);

    printPath(path, graph);

    return 0;
}
