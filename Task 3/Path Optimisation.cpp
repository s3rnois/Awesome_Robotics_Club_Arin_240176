#include <iostream>
#include <vector>
#include <queue>
#include <climits>
#include <unordered_map>
#include <utility>

#define ROWS 10
#define COLS 10

using namespace std;

// Just basic 4-way movement: up, down, left, right
int dx[4] = {-1, 1, 0, 0};  
int dy[4] = {0, 0, -1, 1};  

// Structure to represent a position on the grid
struct Step {
    int x, y;
    long long cost;
    vector<pair<int, int>> trail; // Keeping track of how we got here

    // So we can use Step in a min-heap based on cost
    bool operator>(const Step& other) const {
        return cost > other.cost;
    }
};

// A little helper for debugging or quick prints
void printPath(const vector<pair<int, int>>& path) {
    for (auto p : path) {
        cout << "(" << p.first << ", " << p.second << ") ";
    }
    cout << endl;
}

void findPath(char grid[ROWS][COLS]) {
    // We'll use a priority queue for Dijkstra's - min-heap version
    priority_queue<Step, vector<Step>, greater<Step>> toVisit;

    long long minDist[ROWS][COLS];  // This tracks shortest distance found to each cell
    bool seen[ROWS][COLS];          // Mark if we've processed a cell

    // Set everything to infinite distance initially
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            minDist[r][c] = LLONG_MAX;
            seen[r][c] = false;
        }
    }

    // Assume we always start from top-left (0, 0) — the 'S'
    minDist[0][0] = 0;
    Step start;
    start.x = 0;
    start.y = 0;
    start.cost = 0;
    start.trail.push_back({0, 0});
    toVisit.push(start);

    while (!toVisit.empty()) {
        Step curr = toVisit.top();
        toVisit.pop();

        int x = curr.x;
        int y = curr.y;

        if (seen[x][y]) continue;
        seen[x][y] = true;

        // We found the goal
        if (grid[x][y] == 'G') {
            cout << "Minimum Path Cost: " << curr.cost << endl;
            cout << "Path Taken (Coordinates):" << endl;
            printPath(curr.trail);
            return;
        }

        // Check all directions (up/down/left/right)
        for (int dir = 0; dir < 4; ++dir) {
            int nextX = x + dx[dir];
            int nextY = y + dy[dir];

            // Stay inside the grid and skip walls
            if (nextX < 0 || nextX >= ROWS || nextY < 0 || nextY >= COLS) continue;
            if (grid[nextX][nextY] == '#') continue;  // '#' is a wall

            // Compute cost to move to the neighbor
            long long stepCost = 0;

            char terrain = grid[nextX][nextY];
            if (terrain == 'S') stepCost = 0;
            else if (terrain == '.') stepCost = 1;
            else if (terrain == '~') stepCost = 3;
            else if (terrain == '^') stepCost = 5;
            else if (terrain == 'G') stepCost = 0;
            else stepCost = 1; // Fallback in case of weird input

            long long totalCost = curr.cost + stepCost;

            // If this new path is cheaper, go ahead
            if (totalCost < minDist[nextX][nextY]) {
                minDist[nextX][nextY] = totalCost;
                Step nextStep;
                nextStep.x = nextX;
                nextStep.y = nextY;
                nextStep.cost = totalCost;
                nextStep.trail = curr.trail;  // Copy the trail so far
                nextStep.trail.push_back({nextX, nextY});
                toVisit.push(nextStep);
            }
        }
    }

    // If we get here, no path was found
    cout << "-1 (No path exists)" << endl;
}

int main() {
    char map[ROWS][COLS];

    // Reading grid from input
    for (int r = 0; r < ROWS; ++r) {
        for (int c = 0; c < COLS; ++c) {
            cin >> map[r][c];
        }
    }

    findPath(map);  // Start the pathfinding

    return 0;
}
