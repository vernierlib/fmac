#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {
    
    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/camera.json", "data/aruco/aruco.png");
    cout << camera << endl;
    PoseBox box;
    box.read("data/aruco/camera.json");

    // Generating the pose cloud
    const int seed = 5784;
    const int poseCount = 100;
    PoseCloud cloud(seed, poseCount, box, camera);
    cout << cloud << endl;

    // Writing the cloud in a file
    cout << "Writing cloud file..." << endl;
    cloud.write("data/aruco/cloud.json");
    cout << "Completed. " << endl;
    
    cout << "Drawing marker locations..." << endl;
    Mat imageRGB(camera.imageHeight, camera.imageWidth, CV_8UC3);
    cloud.draw(imageRGB, camera);
    imshow("Location of markers", imageRGB);
    
    cout << "Start rendering? (ESC to stop)" << endl;
    int keyCode = waitKey(0);

    Mat image;
    int k = 0;
    while (k < cloud.getPoseCount() && keyCode != 27) {
        cout << "Rendering image " << k + 1 << "/" << cloud.getPoseCount() << "..." << endl;
        camera.render(cloud.getRVec(k), cloud.getTVec(k), image);
        imwrite("data/aruco/image" + to_string(k + 1) + ".png", image);
        imshow("Rendered image", image);
        keyCode = waitKey(1);
        k++;
    }
    if (keyCode != 27) {
        cout << "Done!" << endl;
    } else {
        cout << "Stopped by user!" << endl;
    }

    return 0;
}
