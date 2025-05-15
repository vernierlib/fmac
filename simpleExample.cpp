#include "ThinLensCamera.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/camera.json", "data/aruco/aruco.png");
    cout << camera;

    cout << "Rendering..." << endl;
    cv::Mat rvec = (Mat_<double>(1, 3) << 1.0177665535647002, -1.0177665535647002, 2.4571058169456226);
    cv::Mat tvec = (Mat_<double>(1, 3) << -123.86148808975392, -75.218446537651744, 500.0);

    Mat image;
    camera.render(rvec, tvec, image);

    imshow("Rendered image", image);
    waitKey(0);

    return 0;
}
