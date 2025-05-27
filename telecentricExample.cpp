#include "TelecentricCamera.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    TelecentricCamera camera("data/matlab/telecentricCamera.yml", "data/matlab/checkerboard.png");
    cout << camera;
    
    cout << "Rendering..." << endl;
    cv::Mat rvec = (Mat_<double>(1, 3) << -0.77703046719924662, -0.28307849950874941, -0.10178129979743512);
    cv::Mat tvec = (Mat_<double>(1, 3) << -178.55589705930504, -29.663306026047344, 801.89111083527519);
    
    Mat image;
    camera.render(rvec, tvec, image);
    cout << "Done!" << endl;
    
    imshow("Rendered image", image);
    waitKey(0);

    return 0;
}
