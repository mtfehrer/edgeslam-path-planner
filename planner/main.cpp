#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>

using namespace std;

string mapPointsFilename = "/home/map_points.txt";
string newestPoseFilename = "/home/newest_pose.txt";
string allPosesFilename = "/home/all_poses.txt";

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

int main() {
    while (1) {
        vector<vector<double>> mapPoints = loadMapPoints();
        vector<vector<double>> newestPose = loadNewestPose();
        vector<vector<vector<double>>> allPoses = loadAllPoses();
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }
    return 0;
}