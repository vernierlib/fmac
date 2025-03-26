#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {

    // Defining cloud limits
    const double yawMin = -M_PI;
    const double yawMax = -yawMin;
    const double pitchMin = -M_PI / 4.0;
    const double pitchMax = -pitchMin;
    const double rollMin = -M_PI / 4.0;
    const double rollMax = -rollMin;
    const double xMin = -0.06;
    const double xMax = -xMin;
    const double yMin = -0.04;
    const double yMax = -yMin;
    const double zMin = 0.1;
    const double zMax = 0.4;

    // Defining the seed and the number of poses in the cloud
    const int seed = 5784;
    const int poseCount = 100;

    // Generating the pose cloud
    PoseCloud cloud(seed, poseCount, yawMin, yawMax, pitchMin, pitchMax, rollMin, rollMax, xMin, xMax, yMin, yMax, zMin, zMax);
    cout << "Cloud: " << cloud << endl;
    cout << "Generated poses: " << endl;
    cloud.printPoses();

    // Writing the cloud in a YAML file
    cloud.write("data/aruco/cloud.yml");

    return 0;
}
