#include "ThinLensCamera.hpp"
#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/left_camera.yml", "data/aruco/aruco.png");
    cout << "Camera: " << camera << endl;
    
    cout << "Drawing marker locations..." << endl;
    PoseCloud cloud("data/aruco/cloud.yml");
    cout << "Cloud: " << cloud << endl;
    Mat imageRGB(camera.imageHeight, camera.imageWidth, CV_8UC3);
    for (int k = 0; k < cloud.getPoseCount(); k++) {
        cv::drawFrameAxes(imageRGB, camera.cameraMatrix, camera.distortionCoefficients, cloud.getRVec(k), cloud.getTVec(k), camera.markerWidth, 1);
    }
    cout << "Start rendering? (ESC to stop)" << endl;
    imshow("Location of markers", imageRGB);
    int keyCode = waitKey(0);

    if (keyCode != 27) {
        Mat image;
        for (int k = 0; k < cloud.getPoseCount(); k++) {
            cout << "Rendering image " << k + 1 << "/" << cloud.getPoseCount() << "..." << endl;
            cout << "RVec " << cloud.getRVec(k) << endl;
            cout << "TVec " << cloud.getTVec(k) << endl;            
            camera.render(cloud.getRVec(k), cloud.getTVec(k), image);
            imwrite("data/aruco/image" + to_string(k + 1) + ".png", image);
            imshow("Rendered image", image);
            waitKey(1);
        }
        cout << "Done!" << endl;
    } else {
        cout << "Stopped by user!" << endl;
    }
    return 0;
}
