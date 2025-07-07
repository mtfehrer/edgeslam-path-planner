#include <fstream>
#include <vector>
#include <sstream>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>

using namespace std;

const int GRID_SIZE = 100;
const double VOXEL_SIZE = 0.1;

// string mapPointsFilename = "/home/map-points.txt";
// string newestPoseFilename = "/home/newest-pose.txt";
// string allPosesFilename = "/home/all-poses.txt";

//for testing without docker
string mapPointsFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/map-points.txt";
string newestPoseFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/newest-pose.txt";
string allPosesFilename = "/home/michael/Projects/edgeslam-path-planner/edgeslam/exported-data/all-poses.txt";
string occupancyGridFilename = "/home/michael/Projects/edgeslam-path-planner/planner/occupancy-grid.txt";

struct GridCoord {
    int i, j, k;
};

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

const Vec3 gridOrigin = {-5.0, -5.0, -1.0};

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

GridCoord worldToGrid(const Vec3& worldPoint) {
    Vec3 relativePos = worldPoint - gridOrigin;
    GridCoord gridCoord;
    gridCoord.i = static_cast<int>(relativePos.x / VOXEL_SIZE);
    gridCoord.j = static_cast<int>(relativePos.y / VOXEL_SIZE);
    gridCoord.k = static_cast<int>(relativePos.z / VOXEL_SIZE);
    return gridCoord;
}

void resetOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid) {
    occupancyGrid.clear();
    for (int i = 0; i < GRID_SIZE; i++) {
        vector<vector<char>> matrix;
        for (int j = 0; j < GRID_SIZE; j++) {
            vector<char> row;
            for (int k = 0; k < GRID_SIZE; k++) {
                row.push_back(0);
            }
            matrix.push_back(row);
        }
        occupancyGrid.push_back(matrix);
    }
}

void addOccupiedVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<double>>& mapPoints) {
    for (const auto& p : mapPoints) {
        Vec3 worldPoint = {p[0], p[1], p[2]};
        GridCoord coord = worldToGrid(worldPoint);

        if (coord.i >= 0 && coord.i < GRID_SIZE &&
            coord.j >= 0 && coord.j < GRID_SIZE &&
            coord.k >= 0 && coord.k < GRID_SIZE) {
            occupancyGrid[coord.i][coord.j][coord.k] = 1;
        }
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
                if (vectors[i].z < min) {
                    min = vectors[i].z;
                }
            }

        }
        return min;
    }
}

void addFreeVoxelsToOccupancyGrid(vector<vector<vector<char>>>& occupancyGrid, vector<vector<vector<double>>> allPoses) {
    double fovY = 0.75;
    double aspectRatio = 1.333;
    double farDist = 10;

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
                    if (i >= 0 && i < GRID_SIZE &&
                        j >= 0 && j < GRID_SIZE &&
                        k >= 0 && k < GRID_SIZE) {
                        if (occupancyGrid[i][j][k] == 0) {
                            occupancyGrid[i][j][k] = 1;
                        }
                    }
                }
            }
        }
    }
}

void exportOccupancyGrid(vector<vector<vector<char>>> occupancyGrid) {
    ofstream f;
    f.open(occupancyGridFilename.c_str());

    for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
            for (int k = 0; k < GRID_SIZE; k++) {
                f << (int) occupancyGrid[i][j][k] << " ";
            }
            f << "\n";
        }
        f << "\n";
    }

    f.close();
}

int main() {
    vector<vector<vector<char>>> occupancyGrid;

    vector<vector<double>> mapPoints = loadMapPoints();
    vector<vector<double>> newestPose = loadNewestPose();
    vector<vector<vector<double>>> allPoses = loadAllPoses();

    resetOccupancyGrid(occupancyGrid);
    addOccupiedVoxelsToOccupancyGrid(occupancyGrid, mapPoints);
    addFreeVoxelsToOccupancyGrid(occupancyGrid, allPoses);

    exportOccupancyGrid(occupancyGrid);

    return 0;
}