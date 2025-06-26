#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std;

const int GRID_SIZE = 50;
const int VOXEL_SIZE = 1;

// string mapPointsFilename = "/home/map-points.txt";
// string newestPoseFilename = "/home/newest-pose.txt";
// string allPosesFilename = "/home/all-poses.txt";

//for testing without docker
string mapPointsFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/map-points.txt";
string newestPoseFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/newest-pose.txt";
string allPosesFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/all-poses.txt";

vector<vector<double>> loadMapPoints() {
    vector<vector<double>> points;
    ifstream f(mapPointsFilename);

    string line;
    double x, y, z;

    while (getline(f, line)) {
        stringstream ss(line);
        if (ss >> x >> y >> z) {
            vector<double> point = {x, y, z};
            points.push_back(point);
        }
    }

    f.close();
    return points;
}

vector<vector<double>> loadNewestPose() {
    ifstream f(newestPoseFilename);
    vector<vector<double>> pose;
    string line;
    int row = 0;

    for (int i = 0; i < 4; i++) {
        vector<double> row;
        for (int j = 0; j < 4; j++) {
            row.push_back(0);
        }
        pose.push_back(row);
    }

    while (getline(f, line) && row < 4) {
        stringstream ss(line);
        double value;
        int col = 0;
        while (ss >> value && col < 4) {
            pose[row][col] = value;
            col++;
        }
        row++;
    }

    f.close();
    
    return pose;
}

vector<vector<vector<double>>> loadAllPoses() {
    vector<vector<vector<double>>> poses;
    ifstream f(allPosesFilename);
    string line;
    vector<vector<double>> pose;

    while (getline(f, line)) {
        if (!line.empty()) {
            vector<double> row;
            stringstream ss(line);
            double value;

            while (ss >> value) {
                row.push_back(value);
            }

            if (!row.empty()) {
                pose.push_back(row);
            }
        }
        else {
            if (pose.size() == 4) {
                poses.push_back(pose);
                pose.clear();
            }
        }
    }

    if (pose.size() == 4) {
        poses.push_back(pose);
    }

    f.close();

    return poses;
}

void resetOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid) {
    occupancyGrid.clear();
    for (int i = 0; i < GRID_SIZE; i++) {
        vector<vector<char>> matrix;
        for (int j = 0; i < GRID_SIZE; i++) {
            vector<char> row;
            for (int k = 0; i < GRID_SIZE; i++) {
                row.push_back(0);
            }
            matrix.push_back(row);
        }
        occupancyGrid.push_back(matrix);
    }
}

void addOccupiedVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<double>>& mapPoints) {
    for (vector<double> p : mapPoints) {
        occupancyGrid[p[0] / VOXEL_SIZE][p[1] / VOXEL_SIZE][p[2] / VOXEL_SIZE] = 1;
    }
}

void addFreeVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<vector<double>>> allPoses) {
    for (vector<vector<double>> p : allPoses) {
        //calculate frustum
        //calculate overlapping voxels
    }
    
}

int main() {
    vector<vector<vector<char>>> occupancyGrid;

    while (1) {
        vector<vector<double>> mapPoints = loadMapPoints();
        vector<vector<double>> newestPose = loadNewestPose();
        vector<vector<vector<double>>> allPoses = loadAllPoses();

        resetOccupancyGrid(occupancyGrid);
        addOccupiedVoxelsToOccupancyGrid(occupancyGrid, mapPoints);
        addFreeVoxelsToOccupancyGrid(occupancyGrid, allPoses);
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return 0;
}