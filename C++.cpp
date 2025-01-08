#include <bits/stdc++.h>

using namespace std;
queue<pair<string, string>> complaintQueue;

// Structure to store house information
struct House {
    string id;
    double x, y;
    double pressure = 100.0; // Default pressure for each house
    vector<pair<string, double>> pipelines; // (connected house, cost)
};

// Function to calculate the Euclidean distance between two houses
double calculateDistance(const House& house1, const House& house2) {
    return sqrt(pow(house2.x - house1.x, 2) + pow(house2.y - house1.y, 2));
}

// Function to adjust the water pressure within safe limits
void adjustPressure(vector<House>& houses) {
    for (auto& house : houses) {
        double totalPipelineCost = 0;
        for (const auto& pipeline : house.pipelines) {
            totalPipelineCost += pipeline.second;
        }
        if (totalPipelineCost > 100.0) {
            house.pressure -= (totalPipelineCost - 100.0) / 10.0;
        } else {
            house.pressure += (100.0 - totalPipelineCost) / 20.0;
        }

        if (house.pressure < 80) {
            house.pressure = 80;
            cout << "Warning: House " << house.id << " had low pressure. Adjusted to 80 units." << endl;
        } else if (house.pressure > 120) {
            house.pressure = 120;
            cout << "Warning: House " << house.id << " had excessive pressure. Adjusted to 120 units." << endl;
        }
    }
}

// Function to add a new house and update the network
void addNewHouse(vector<House>& houses) {
    string newHouseId;
    double newX, newY;

    cout << "Enter the name of the new house: ";
    cin >> newHouseId;
    cout << "Enter the coordinates (x, y) for the new house: ";
    cin >> newX >> newY;

    houses.push_back({newHouseId, newX, newY});

    for (size_t i = 0; i < houses.size() - 1; i++) {
        double distance = calculateDistance(houses[i], houses.back());
        houses[i].pipelines.push_back({houses.back().id, distance});
        houses.back().pipelines.push_back({houses[i].id, distance});
    }

    cout << "New house " << newHouseId << " added successfully!" << endl;
}

// Kruskal's algorithm to find the minimum spanning tree (MST)
void calculateMST(vector<House>& houses) {
    vector<pair<double, pair<string, string>>> edges;

    for (const auto& house : houses) {
        for (const auto& pipeline : house.pipelines) {
            if (house.id < pipeline.first) {
                edges.push_back({pipeline.second, {house.id, pipeline.first}});
            }
        }
    }

    sort(edges.begin(), edges.end());

    unordered_map<string, string> parent;
    for (const auto& house : houses) {
        parent[house.id] = house.id;
    }

    function<string(const string&)> findParent = [&](const string& x) -> string {
        if (parent[x] != x) {
            parent[x] = findParent(parent[x]);
        }
        return parent[x];
    };

    function<void(const string&, const string&)> unionSets = [&](const string& x, const string& y) {
        string parentX = findParent(x);
        string parentY = findParent(y);
        if (parentX != parentY) {
            parent[parentX] = parentY;
        }
    };

    double totalCost = 0;
    cout << "--- Water Flow Pipeline Network (Minimum Spanning Tree) ---" << endl;
    for (const auto& edge : edges) {
        string house1 = edge.second.first;
        string house2 = edge.second.second;
        double distance = edge.first;

        if (findParent(house1) != findParent(house2)) {
            cout << "Pipeline between " << house1 << " and " << house2 << " (Cost: " << distance << ")" << endl;
            totalCost += distance;
            unionSets(house1, house2);
        }
    }

    cout << "--- Water Flow Optimization ---" << endl;
    cout << "Total pipeline length (cost) for efficient water flow: " << totalCost << " units." << endl;
    adjustPressure(houses);

    cout << "--- Water Flow Status ---" << endl;
    for (const auto& house : houses) {
        cout << "House " << house.id << " has a water pressure of " << house.pressure << " units." << endl;
    }
}

// Function to compute prefix table for KMP algorithm
vector<int> computePrefixTable(const string& pattern) {
    int m = pattern.length();
    vector<int> prefix(m, 0);
    int j = 0;

    for (int i = 1; i < m; i++) {
        while (j > 0 && pattern[i] != pattern[j]) {
            j = prefix[j - 1];
        }
        if (pattern[i] == pattern[j]) {
            j++;
        }
        prefix[i] = j;
    }

    return prefix;
}

// KMP string search function
bool kmpSearch(const string& text, const string& pattern) {
    int n = text.length();
    int m = pattern.length();
    vector<int> prefix = computePrefixTable(pattern);

    int j = 0; // Pointer for pattern
    for (int i = 0; i < n; i++) {
        while (j > 0 && text[i] != pattern[j]) {
            j = prefix[j - 1];
        }
        if (text[i] == pattern[j]) {
            j++;
        }
        if (j == m) {
            return true; // Pattern found
        }
    }
    return false; // Pattern not found
}

// Register a complaint with KMP string search for house ID matching

void registerComplaint(const vector<string>& keywords, const vector<House>& houses) {
    string partialHouseId, complaint;

    cout << "Enter part of the house ID for registering a complaint: ";
    cin >> partialHouseId;

    // Use KMP to search for the partial house ID in the list of house IDs
    bool houseFound = false;
    string fullHouseId;

    for (const auto& house : houses) {
        if (kmpSearch(house.id, partialHouseId)) {
            houseFound = true;
            fullHouseId = house.id;
            break;
        }
    }

    if (!houseFound) {
        cout << "Error: No matching house ID found!" << endl;
        return;
    }

    cout << "House ID found: " << fullHouseId << endl;

    cout << "Describe your complaint: ";
    cin.ignore(); // Clear input buffer
    getline(cin, complaint);

    string category = "General"; // Default category
    for (const auto& keyword : keywords) {
        if (kmpSearch(complaint, keyword)) {
            category = keyword;
            break;
        }
    }
    // Get the current time and format it
    time_t now = time(0); // Current time
    tm* localTime = localtime(&now); // Convert to local time
    char timestamp[20];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", localTime); // Format as "YYYY-MM-DD HH:MM:SS"


    // Push complaint into the queue
    complaintQueue.push({fullHouseId, "Category: " + category + ", Complaint: " + complaint+ ", Time: " + timestamp});
    cout << "Complaint registered under category: " << category << endl;
}

// Display the current network of houses and pipelines
void displayNetwork(const vector<House>& houses) {
    cout << "--- Current Network of Houses and Pipelines ---" << endl;
    for (const auto& house : houses) {
        cout << house.id << " -> ";
        for (const auto& pipeline : house.pipelines) {
            cout << "(" << pipeline.first << ", Cost: " << pipeline.second << ") ";
        }
        cout << endl;
    }
}
// Function to find the shortest path using Dijkstra's Algorithm
void findShortestPath(const vector<House>& houses, const string& startHouseId, const string& endHouseId) {
    unordered_map<string, double> distances; // Store the shortest distance from start to each house
    unordered_map<string, string> previous; // Store the previous house in the shortest path
    unordered_map<string, bool> visited;    // Keep track of visited houses
    priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> pq; // Min-heap priority queue

    // Initialize distances to infinity and visited to false
    for (const auto& house : houses) {
        distances[house.id] = numeric_limits<double>::infinity();
        visited[house.id] = false;
    }

    // Distance to the start house is 0
    distances[startHouseId] = 0.0;
    pq.push({0.0, startHouseId});

    // Main Dijkstra's Algorithm loop
    while (!pq.empty()) {
        string currentHouse = pq.top().second;
        double currentDistance = pq.top().first;
        pq.pop();

        // Skip if the house is already visited
        if (visited[currentHouse]) continue;

        visited[currentHouse] = true;

        // Find the current house in the houses vector
        const auto& house = *find_if(houses.begin(), houses.end(), [&](const House& h) { return h.id == currentHouse; });

        // Update distances to neighboring houses
        for (const auto& pipeline : house.pipelines) {
            string neighborHouse = pipeline.first;
            double pipelineCost = pipeline.second;

            if (!visited[neighborHouse] && currentDistance + pipelineCost < distances[neighborHouse]) {
                distances[neighborHouse] = currentDistance + pipelineCost;
                previous[neighborHouse] = currentHouse;
                pq.push({distances[neighborHouse], neighborHouse});
            }
        }
    }

    // Print the shortest path
    if (distances[endHouseId] == numeric_limits<double>::infinity()) {
        cout << "No path exists between " << startHouseId << " and " << endHouseId << "." << endl;
    } else {
        cout << "Shortest path from " << startHouseId << " to " << endHouseId << " is " << distances[endHouseId] << " units." << endl;

        // Backtrack to find the path
        vector<string> path;
        for (string at = endHouseId; at != ""; at = previous[at]) {
            path.push_back(at);
        }
        reverse(path.begin(), path.end());

        cout << "Path: ";
        for (const string& house : path) {
            cout << house;
            if (house != endHouseId) cout << " -> ";
        }
        cout << endl;
    }
}
void displayComplaints() {
    if (complaintQueue.empty()) {
        cout << "No complaints registered yet." << endl;
        return;
    }

    cout << "--- Registered Complaints ---" << endl;
    while (!complaintQueue.empty()) {
        auto complaint = complaintQueue.front();
        cout << "House ID: " << complaint.first << " -> " << complaint.second << endl;
        complaintQueue.pop(); // Remove the complaint from the queue
    }
}
// Function to calculate the all-pairs shortest paths using Floyd-Warshall Algorithm
vector<vector<double>> floydWarshall(const vector<House>& houses) {
    int n = houses.size();
    vector<vector<double>> distances(n, vector<double>(n, numeric_limits<double>::infinity()));

    // Initialize distances with pipeline costs
    for (int i = 0; i < n; i++) {
        distances[i][i] = 0; // Distance to itself is 0
        for (const auto& pipeline : houses[i].pipelines) {
            int neighborIndex = find_if(houses.begin(), houses.end(), [&](const House& h) {
                return h.id == pipeline.first;
            }) - houses.begin();
            distances[i][neighborIndex] = pipeline.second;
        }
    }

    // Floyd-Warshall Algorithm
    for (int k = 0; k < n; k++) {
        for (int i = 0; i < n; i++) {
            for (int j = 0; j < n; j++) {
                if (distances[i][k] < numeric_limits<double>::infinity() &&
                    distances[k][j] < numeric_limits<double>::infinity()) {
                    distances[i][j] = min(distances[i][j], distances[i][k] + distances[k][j]);
                }
            }
        }
    }

    return distances;
}

// Function to find the optimal pump location
void findOptimalPumpPlacement(const vector<House>& houses) {
    vector<vector<double>> distances = floydWarshall(houses);

    int n = houses.size();
    vector<double> maxDistances(n, 0);

    // Calculate the maximum distance from each house to any other house
    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            if (distances[i][j] != numeric_limits<double>::infinity()) {
                maxDistances[i] = max(maxDistances[i], distances[i][j]);
            }
        }
    }

    // Find the house with the minimum of the maximum distances (Graph Center)
    double minMaxDistance = numeric_limits<double>::infinity();
    int optimalPumpIndex = -1;
    for (int i = 0; i < n; i++) {
        if (maxDistances[i] < minMaxDistance) {
            minMaxDistance = maxDistances[i];
            optimalPumpIndex = i;
        }
    }

    // Display the result
    if (optimalPumpIndex != -1) {
        cout << "--- Optimal Pump Placement ---" << endl;
        cout << "The optimal location for placing the water pump is at House: " 
             << houses[optimalPumpIndex].id << endl;
        cout << "Maximum distance from any house to this pump: " 
             << minMaxDistance << " units." << endl;
    } else {
        cout << "Error: Unable to determine the optimal pump placement." << endl;
    }
}





// Main function
int main() {
    vector<House> houses;
    vector<string> keywords = {"leakage", "low pressure", "no water","blockage","low speed flow"};

    int numHouses;

    cout << "Welcome to the Water Pipeline Management System." << endl;
    cout << "-----------------------------------------------" << endl;
    
    // Introduction
    cout << "This system helps us efficiently plan and manage water pipeline connections for the city's infrastructure." << endl;
    cout << "To begin the process, we need to first assess the number of households that will require access to the water supply." << endl;
    
    // Asking for input
    cout << "Please provide the total number of houses to be connected to the system." << endl;
    
    cin >> numHouses;

    for (int i = 0; i < numHouses; i++) {
        string houseId;
        double x, y;

        cout << "Enter the name for house " << i + 1 << ": ";
        cin >> houseId;
        cout << "Enter the coordinates (x, y) for house " << i + 1 << ": ";
        cin >> x >> y;

        houses.push_back({houseId, x, y});
    }

    for (size_t i = 0; i < houses.size(); i++) {
        for (size_t j = i + 1; j < houses.size(); j++) {
            double distance = calculateDistance(houses[i], houses[j]);
            houses[i].pipelines.push_back({houses[j].id, distance});
            houses[j].pipelines.push_back({houses[i].id, distance});
        }
    }

    while (true) {
        cout << "--- Menu ---" << endl;
        cout << "1. Display current network" << endl;
        cout << "2. Add a new house" << endl;
        cout << "3. Register a complaint" << endl;
        cout << "4. Display complaints" << endl;
        cout << "5. Calculate minimum spanning tree for water flow" << endl;
        cout << "6. Shortest path during maintenance" << endl;
        cout << "7. Find Optimal Pump Placement" << endl;
        cout << "8. Exit" << endl;
        

        cout << "Enter your choice: ";
        int choice;
        cin >> choice;

        switch (choice) {
            case 1:
                displayNetwork(houses);
                break;
            case 2:
                addNewHouse(houses);
                break;
            case 3:
                registerComplaint(keywords,houses);
                break;
           case 4:
               displayComplaints();
               break;
            case 5:
                calculateMST(houses);
                break;
            case 8:
                cout << "Exiting the program. Goodbye!" << endl;
                return 0;
                case 7:
                findOptimalPumpPlacement(houses);
                break;

            case 6: {
                  string startHouseId, endHouseId;
                  cout << "Enter the starting house ID: ";
                  cin >> startHouseId;
                  cout << "Enter the ending house ID: ";
                  cin >> endHouseId;
                  findShortestPath(houses, startHouseId, endHouseId);
                  break;
}

            default:
                cout << "Invalid choice, please try again." << endl;
        }
    }

    return 0;
}
