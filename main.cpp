#include <iostream>
#include <fstream>
#include <queue>
#include <map>
#include <vector>
#include <deque>
#include <limits>
#include <string>
#include <sstream>
#include <cmath>

using namespace std;


struct Edge {
    string station;
    int time;
    string line_name;
};

struct Node {
    string station;
    int distance;
    int heuristic; // Heuristic value for A* eine art Annahme
};

bool operator<(const Node &a, const Node &b) {
    return (a.distance + a.heuristic) > (b.distance + b.heuristic);
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


int calculate_heuristic(const string &station, const string &end) {
    // Simple heuristic value: number of character differences between the stations
    int diff = abs(static_cast<int>(station.size()) - static_cast<int>(end.size()));
    return diff;
}


pair<deque<pair<string, string>>, int> find_path_dijkstra(const Graph &graph, const string &start, const string &end) {
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


pair<deque<pair<string, string>>, int> find_path_astar(const Graph &graph, const string &start, const string &end) {
    map<string, int> distances;
    for (const auto &[station, _]: graph) {
        //jede distance wird mit dem größtmöglichen Wert für den Datentyp int initializert
        distances[station] = numeric_limits<int>::max();
    }
    // Initialisiere den Start mit 0
    distances[start] = 0;
    map<string, string> previous_nodes;
    map<string, string> previous_lines;

    priority_queue<Node> queue;
    // Füge die Startstation zur priority queue hinzu
    // Die Priorität wird durch die Summe aus der bisher zurückgelegten Strecke (distance)
    // und der geschätzten Reststrecke (heuristic) bestimmt.
    queue.push({start, 0, calculate_heuristic(start, end)});

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
                // Aktualisiere die Distanz zum Nachbarn und speichere den Vorgänger und die Linie
                distances[neighbor.station] = distance;
                previous_nodes[neighbor.station] = current.station;
                previous_lines[neighbor.station] = neighbor.line_name;
                // Füge den Nachbarn mit aktualisierter Distanz und Heuristik zur Prioritätswarteschlange hinzu
                queue.push({neighbor.station, distance, calculate_heuristic(neighbor.station, end)});
            }
        }
    }

    deque<pair<string, string>> path;
    string current_node = end;
    while (!previous_nodes[current_node].empty()) {
        // Baue den Pfad von hinten auf, indem jeder Knoten und die zugehörige Linie hinzugefügt werden
        path.emplace_front(current_node, previous_lines[current_node]);
        current_node = previous_nodes[current_node];
    }
    // Füge die Startstation als erstes Element zum Pfad hinzu
    path.emplace_front(start, "");

    int total_cost = distances[end];
    // Gib den Pfad und die Gesamtkosten zurück
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
            cout << "Transfer to Line " << line << endl;
        }
        cout << station << endl;
        previous_line = line;
    }
    cout << "Total travel time: " << total_cost << endl;
}

int main(int argc, char *argv[]) {
    string filename_graph = "test.txt";
    string start_station = argv[1];
    string end_station = argv[2];

    Graph graph = read_file(filename_graph);

    cout << "Dijkstra's Algorithm:" << endl;
    auto [dijkstra_path, dijkstra_total_cost] = find_path_dijkstra(graph, start_station, end_station);
    pretty_print(dijkstra_path, dijkstra_total_cost);

    cout << endl;

    cout << "A* Algorithm:" << endl;
    auto [astar_path, astar_total_cost] = find_path_astar(graph, start_station, end_station);
    pretty_print(astar_path, astar_total_cost);

    return 0;
}
