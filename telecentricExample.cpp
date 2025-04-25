#include "TelecentricCamera.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    TelecentricCamera camera("data/matlab/telecentricCamera.yml", "data/aruco/aruco.png");
    cout << camera;
    
    cout << "Rendering..." << endl;
    cv::Mat rvec = (Mat_<double>(1, 3) << 0.54395, -0.0622605, -0.137385);
    cv::Mat tvec = (Mat_<double>(1, 3) << -77.55589, 86.6330, 701.891110);
    
    Mat image;
    camera.render(rvec, tvec, image);
    
    imshow("Rendered image", image);
    waitKey(0);

    return 0;
}
