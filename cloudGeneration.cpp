#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {
    
    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/left_camera.yml", "data/aruco/aruco.png");
    cout << camera << endl;

    // Defining cloud limits
    PoseBox box;
    box.yawMin = -M_PI;
    box.yawMax = -box.yawMin;
    box.pitchMin = -M_PI / 4.0;
    box.pitchMax = -box.pitchMin;
    box.rollMin = -M_PI / 4.0;
    box.rollMax = -box.rollMin;
    box.zMin = 0.5;
    box.zMax = 0.1;

    // Generating the pose cloud
    const int seed = 5784;
    const int poseCount = 100;
    PoseCloud cloud(seed, poseCount, box, camera);
    cout << cloud << endl;

    // Writing the cloud in a YAML file
    cout << "Writing cloud file..." << endl;
    cloud.write("data/aruco/cloud.yml");
    cout << "Completed. " << endl;
    
    cout << "Drawing marker locations..." << endl;
    Mat imageRGB(camera.imageHeight, camera.imageWidth, CV_8UC3);
    cloud.draw(imageRGB, camera);
    imshow("Location of markers", imageRGB);
    waitKey(0);

    return 0;
}
