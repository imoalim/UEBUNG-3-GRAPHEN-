#include <iostream>
#include <fstream>
#include <queue>
#include <map>
#include <vector>
#include <deque>
#include <limits>
#include <string>
#include <sstream>

using namespace std;

struct Edge {
    string station;
    int time;
    string line_name;
};

struct Node {
    string station;
    int distance;
};

bool operator<(const Node &a, const Node &b) {
    return a.distance > b.distance;
}

using Graph = map<string, vector<Edge>>;

std::vector<std::string> splitAndStrip(const std::string &line, char delimiter) {
    std::vector<std::string> parts;
    std::istringstream iss(line);
    std::string part;

    while (std::getline(iss, part, delimiter)) {
        // Remove leading and trailing whitespaces
        part.erase(part.begin(), std::find_if(part.begin(), part.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
        part.erase(std::find_if(part.rbegin(), part.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), part.end());
        // Add the part to the vector only if it is not empty
        if (!part.empty()) {
            parts.push_back(part);
        }
    }
    return parts;
}

Graph read_file(const string &filename) {
    Graph graph;
    ifstream file(filename);

    string line;
    while (getline(file, line)) {
        auto parts = splitAndStrip(line, '\"');
        string line_name = parts[0].substr(0, parts[0].size() - 1);
        for (size_t i = 1; i < parts.size() - 2; i += 2) {
            string station1 = parts[i];
            int time = stoi(parts[i + 1]);
            string station2 = parts[i + 2];

            graph[station1].push_back({station2, time, line_name});
            graph[station2].push_back({station1, time, line_name});
        }
    }
    return graph;
}

pair<deque<pair<string, string>>, int> find_path(const Graph &graph, const string &start, const string &end) {
    map<string, int> distances;
    for (const auto &[station, _]: graph) {
        distances[station] = numeric_limits<int>::max();
    }
    distances[start] = 0;
    map<string, string> previous_nodes;
    map<string, string> previous_lines;

    priority_queue<Node> queue;
    queue.push({start, 0});

    while (!queue.empty()) {
        Node current = queue.top();
        queue.pop();

        if (current.station == end) {
            break;
        }

        if (current.distance > distances[current.station]) {
            continue;
        }

        for (const auto &neighbor: graph.at(current.station)) {
            int distance = current.distance + neighbor.time;
            if (distance < distances[neighbor.station]) {
                distances[neighbor.station] = distance;
                previous_nodes[neighbor.station] = current.station;
                previous_lines[neighbor.station] = neighbor.line_name;
                queue.push({neighbor.station, distance});
            }
        }
    }

    deque<pair<string, string>> path;
    string current_node = end;
    while (!previous_nodes[current_node].empty()) {
        path.emplace_front(current_node, previous_lines[current_node]);
        current_node = previous_nodes[current_node];
    }
    path.emplace_front(start, "");

    int total_cost = distances[end];
    return {path, total_cost};
}

void pretty_print(const deque<pair<string, string>> &path, int total_cost) {
    if (path.size() == 1) {
        cout << "No path found" << endl;
        return;
    }
    string previous_line;
    for (const auto &[station, line]: path) {
        if (!line.empty() && line != previous_line) {
            cout << "Change to " << line << endl;
        }
        cout << station << endl;
        previous_line = line;
    }
    cout << "Total time: " << total_cost << endl;
}

int main(int argc, char *argv[]) {
    string filename_graph = "paths.txt";
    string start_station = argv[1];
    string end_station = argv[2];

    Graph graph = read_file(filename_graph);
    auto [path, total_cost] = find_path(graph, start_station, end_station);
    pretty_print(path, total_cost);

    return 0;
}
