#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>

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

struct Vec3 {
    double x = 0.0f, y = 0.0f, z = 0.0f;
    Vec3 operator+(const Vec3& other) const {
        return {x + other.x, y + other.y, z + other.z};
    }
    Vec3 operator-(const Vec3& other) const {
        return {x - other.x, y - other.y, z - other.z};
    }
    Vec3 operator*(double scalar) const {
        return {x * scalar, y * scalar, z * scalar};
    }
    Vec3 operator-() const {
        return {-x, -y, -z};
    }
};

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

double getExtreme(vector<Vec3> vectors, char direction, bool findMax) {
    if (findMax) {
        double max = -std::numeric_limits<double>::infinity();
        if (direction == 'x') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].x > max) {
                    max = vectors[i].x;
                }
            }
        }
        else if (direction == 'y') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].y > max) {
                    max = vectors[i].y;
                }
            }
        }
        else if (direction == 'z') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].z > max) {
                    max = vectors[i].z;
                }
            }
        }
        return max;
    }
    else {
        double min = std::numeric_limits<double>::infinity();
        if (direction == 'x') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].x < min) {
                    min = vectors[i].x;
                }
            }
        }
        if (direction == 'y') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].y < min) {
                    min = vectors[i].y;
                }
            }

        }
        if (direction == 'z') {
            for (int i = 0; i < vectors.size(); i++) {
                if (vectors[i].y < min) {
                    min = vectors[i].y;
                }
            }

        }
        return min;
    }
}

void addFreeVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<vector<double>>> allPoses) {
    double fovY = 0;
    double aspectRatio = 0;
    double farDist = 0;

    for (vector<vector<double>> p : allPoses) {
        Vec3 cameraPos = {p[0][3], p[1][3], p[2][3]};
        Vec3 right = {p[0][0], p[1][0], p[2][0]};
        Vec3 up    = {p[0][1], p[1][1], p[2][1]};
        Vec3 forward = -Vec3{p[0][2], p[1][2], p[2][2]};

        const float tanHalfFovY = tan(fovY / 2.0);
        const float farHeight = 2.0 * tanHalfFovY * farDist;
        const float farWidth = farHeight * aspectRatio;

        const Vec3 farCenter = cameraPos + (forward * farDist);
        const Vec3 halfUp = up * (farHeight / 2.0);
        const Vec3 halfRight = right * (farWidth / 2.0);

        Vec3 frustumFtl = farCenter + halfUp - halfRight;
        Vec3 frustumFtr = farCenter + halfUp + halfRight;
        Vec3 frustumFbl = farCenter - halfUp - halfRight;
        Vec3 frustumFbr = farCenter - halfUp + halfRight;
        vector<Vec3> vectors = {frustumFtl, frustumFtr, frustumFbl, frustumFbr};

        double minX = getExtreme(vectors, 'x', false);
        double minY = getExtreme(vectors, 'y', false);
        double minZ = getExtreme(vectors, 'z', false);
        double maxX = getExtreme(vectors, 'x', true);
        double maxY = getExtreme(vectors, 'y', true);
        double maxZ = getExtreme(vectors, 'z', true);

        for (int i = (int) minX; i < (int) maxX; i++) {
            for (int j = (int) minY; j < (int) maxY; j++) {
                for (int k = (int) minZ; k < (int) maxZ; k++) {
                    if (occupancyGrid[i][j][k] == 0) {
                        occupancyGrid[i][j][k] = 1;
                    }
                }
            }
        }
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