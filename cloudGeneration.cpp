#include "ThinLensCamera.hpp"
#include "PoseCloud.hpp"

using namespace cv;
using namespace std;

int main() {
    
    string folder = "data/aruco/";
    
    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera(folder + "camera.json", folder + "aruco.png");
    cout << camera << endl;
    PoseBox box;
    box.read(folder + "camera.json");
    
    cout << "Generating the pose cloud..." << endl;
    const int seed = 5784;
    const int poseCount = 100;
    PoseCloud cloud(seed, poseCount, box, camera);
    cout << cloud << endl;

    cout << "Writing the cloud in a file..." << endl;
    cloud.write(folder + "actualPoses.json");
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
        imwrite(folder + "image" + to_string(k + 1) + ".png", image);
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
