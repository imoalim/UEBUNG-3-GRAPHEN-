#include <iostream>
#include <fstream>
#include <queue>
#include <map>
#include <vector>
#include <deque>
#include <limits>
#include <string>
#include <sstream>
#include<windows.h>
#include <chrono>

HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);

//Structs
struct Edge {
    std::string station;
    int time;
    std::string line_name;
};

struct Node {
    std::string station;
    int distance;
    int heuristic;
};

using Graph = std::map<std::string, std::vector<Edge>>;


/***                           ***  Function declarations  ***                                              ***/

//Overload < operator for priority queue
bool operator<(const Node &a, const Node &b) {
    return (a.distance + a.heuristic) > (b.distance + b.heuristic);
}


//Function to split the line into parts and remove whitespaces from the parts
std::vector<std::string> splitAndStrip(const std::string &line, char delimiter) {
    std::vector<std::string> parts;
    std::istringstream iss(line);
    std::string part;

    while (std::getline(iss, part, delimiter)) {
        part.erase(part.begin(), std::find_if(part.begin(), part.end(), [](unsigned char ch) {
            return !std::isspace(ch);
        }));
        part.erase(std::find_if(part.rbegin(), part.rend(), [](unsigned char ch) {
            return !std::isspace(ch);
        }).base(), part.end());

        if (!part.empty()) {
            parts.push_back(part);
        }
    }
    return parts;
}


// Function to read the file and create the graph
Graph read_file(const std::string &filename) {
    Graph graph;
    std::ifstream file(filename);
    std::string line;

    while (getline(file, line)) {
        auto parts = splitAndStrip(line, '\"');
        std::string line_name = parts[0].substr(0, parts[0].size() - 1);
        for (size_t i = 1; i < parts.size() - 2; i += 2) {
            std::string station1 = parts[i];
            int time = stoi(parts[i + 1]);
            std::string station2 = parts[i + 2];

            graph[station1].push_back({station2, time, line_name});
            graph[station2].push_back({station1, time, line_name});
        }
    }
    return graph;
}

//  Calculates the heuristic value for the A* algorithm
int calculate_heuristic(const std::string &station, const std::string &end) {
    // Simple heuristic value: number of character differences between the stations
    int diff = abs(static_cast<int>(station.size()) - static_cast<int>(end.size()));
    return diff;
}


//Finds the shortest path between two stations using the Dijkstra algorithm
std::pair<std::deque<std::pair<std::string, std::string>>, int> find_path_dijkstra(const Graph &graph, const std::string &start, const std::string &end) {
    std::map<std::string, int> distances;
    for (const auto &[station, _]: graph) {
        distances[station] = std::numeric_limits<int>::max();
    }
    distances[start] = 0;
    std::map<std::string, std::string> previous_nodes;
    std::map<std::string, std::string> previous_lines;

    std::priority_queue<Node> queue;
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

    std::deque<std::pair<std::string, std::string>> path;
    std::string current_node = end;
    while (!previous_nodes[current_node].empty()) {
        path.emplace_front(current_node, previous_lines[current_node]);
        current_node = previous_nodes[current_node];
    }
    path.emplace_front(start, "");

    int total_cost = distances[end];
    return {path, total_cost};
}


//Finds the shortest path between two stations using the A* algorithm
std::pair<std::deque<std::pair<std::string, std::string>>, int> find_path_astar(const Graph &graph, const std::string &start, const std::string &end) {
    std::map<std::string, int> distances;
    for (const auto &[station, _]: graph) {
        distances[station] = std::numeric_limits<int>::max();
    }

    distances[start] = 0;
    std::map<std::string, std::string> previous_nodes;
    std::map<std::string, std::string> previous_lines;

    std::priority_queue<Node> queue;
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
                distances[neighbor.station] = distance;
                previous_nodes[neighbor.station] = current.station;
                previous_lines[neighbor.station] = neighbor.line_name;
                queue.push({neighbor.station, distance, calculate_heuristic(neighbor.station, end)});
            }
        }
    }

    std::deque<std::pair<std::string, std::string>> path;
    std::string current_node = end;
    while (!previous_nodes[current_node].empty()) {
        path.emplace_front(current_node, previous_lines[current_node]);
        current_node = previous_nodes[current_node];
    }
    path.emplace_front(start, "");
    int total_cost = distances[end];

    return {path, total_cost};
}


// Prints the path and the total cost of the path to the console in a pretty way using colors and stuff :)
void pretty_print(const std::deque<std::pair<std::string, std::string>> &path, int total_cost) {
    if (path.size() == 1) {
        std::cout << "No path found" << std::endl;
        return;
    }
    std::string previous_line;
    int stations{0};
    for (const auto &[station, line]: path) {
        if (!line.empty() && line != previous_line) {
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED);
            std::cout << " --> Transfer to Line " << line << std::endl;
            SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);

        }
        std::cout << " - " << station << std::endl;
        ++stations;
        previous_line = line;
    }

    SetConsoleTextAttribute(hConsole, FOREGROUND_RED);
    std::cout << " \n --> Total travel time: " << total_cost << std::endl;
    SetConsoleTextAttribute(hConsole, FOREGROUND_BLUE);
    std::cout << " --> Number of stations: " << stations << std::endl;
    SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);
}



/***                                  ***  MAIN   ***                                                              ***/

int main(int argc, char *argv[]) {
    std::string filename_graph = "paths.txt";
    std::string start_station = argv[1];
    std::string end_station = argv[2];

    //  Reads the graph from the file
    Graph graph = read_file(filename_graph);

    SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN);
    std::cout << "\n +++   Dijkstra's Algorithm    +++" << std::endl;
    SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);

    auto [dijkstra_path, dijkstra_total_cost] = find_path_dijkstra(graph, start_station, end_station);
    pretty_print(dijkstra_path, dijkstra_total_cost);

    SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN);
    std::cout << "\n ***       A* Algorithm        ***" << std::endl;
    SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);

    auto [astar_path, astar_total_cost] = find_path_astar(graph, start_station, end_station);
    pretty_print(astar_path, astar_total_cost);



    SetConsoleTextAttribute(hConsole, FOREGROUND_GREEN);
    std::cout << "\n\n+++       Test cases         +++" << std::endl;
    SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);



    //  Test cases for the algorithms (Dijkstra and A*)
    std::vector<std::pair<std::string, std::string>> test_cases = {
            {"Handelskai", "Praterstern"},
            {"Floridsdorf", "Schottenring"},
            {"Schwedenplatz", "Westbahnhof"}
    };

    for (const auto& [start_station, end_station] : test_cases) {
        // Dijkstra algorithm
        auto start_dijkstra = std::chrono::high_resolution_clock::now();
        auto result_dijkstra = find_path_dijkstra(graph, start_station, end_station);
        auto end_dijkstra = std::chrono::high_resolution_clock::now();
        auto elapsed_time_dijkstra = std::chrono::duration_cast<std::chrono::microseconds>(end_dijkstra - start_dijkstra).count();

        //  A* algorithm
        auto start_astar = std::chrono::high_resolution_clock::now();
        auto result_astar = find_path_astar(graph, start_station, end_station);
        auto end_astar = std::chrono::high_resolution_clock::now();
        auto elapsed_time_astar = std::chrono::duration_cast<std::chrono::microseconds>(end_astar - start_astar).count();

        SetConsoleTextAttribute(hConsole, FOREGROUND_BLUE);
        std::cout << "Test case: " << start_station << " -> " << end_station << std::endl;
        SetConsoleTextAttribute(hConsole, FOREGROUND_RED | FOREGROUND_GREEN | FOREGROUND_BLUE);

        std::cout << "Dijkstra elapsed time: " << elapsed_time_dijkstra << " microseconds" << std::endl;
        std::cout << "A* elapsed time: " << elapsed_time_astar << " microseconds" << std::endl;
    }


    return 0;
}
