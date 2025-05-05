#include "ThinLensCamera.hpp"
#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/left_camera.yml", "data/aruco/aruco.png");
    cout << camera << endl;
    PoseCloud cloud("data/aruco/cloud.yml");
    cout << cloud << endl;
    cout << "Drawing marker locations..." << endl;
    Mat imageRGB(camera.imageHeight, camera.imageWidth, CV_8UC3);
    cout << "Start rendering? (ESC to stop)" << endl;
    imshow("Location of markers", imageRGB);
    int keyCode = waitKey(0);

    Mat image;
    int k = 0;
    while (k < cloud.getPoseCount() && keyCode != 27) {
        cout << "Rendering image " << k + 1 << "/" << cloud.getPoseCount() << "..." << endl;
        cout << "RVec " << cloud.getRVec(k) << endl;
        cout << "TVec " << cloud.getTVec(k) << endl;
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
