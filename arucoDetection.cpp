#include "ThinLensCamera.hpp"
#include <opencv2/objdetect/aruco_detector.hpp>

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/aruco/left_camera.yml", "data/aruco/aruco.png");
    cout << camera << endl;

    cout << "Rendering..." << endl;
    cv::Mat rvec = (Mat_<double>(1, 3) << 0.54395, -0.0622605, -0.137385);
    cv::Mat tvec = (Mat_<double>(1, 3) << 0.01, 0.02, 0.1);
    
    Mat image;
    camera.render(rvec, tvec, image);
    
    cout << "Detecting..." << endl;

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
    cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
    // detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    // detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_CONTOUR;
    // detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_APRILTAG;

    cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    cv::aruco::ArucoDetector detector(dictionary, detectorParams);
    detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);

    Mat imageRGB;
    merge(std::vector<Mat>({image, image, image}), imageRGB);
    
    if (!markerIds.empty()) {
        cout << "Marker id: " << markerIds[0] << endl;
        cout << "Marker corners: " << markerCorners[0][0] << ", " << markerCorners[0][1] << ", " << markerCorners[0][2] << ", " << markerCorners[0][3] << endl;

        // set coordinate system
        double markerLength = camera.markerWidth;
        cv::Mat objPoints(4, 1, CV_32FC3);
        objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
        objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
        objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);

        Vec3d rvec2, tvec2;
        solvePnP(objPoints, markerCorners[0], camera.cameraMatrix, camera.distortionCoefficients, rvec2, tvec2);

        cout << "Initial rvec:   " << rvec << endl;
        cout << "Estimated rvec: " << rvec2 << endl;
        cout << "Initial tvec:   " << tvec << endl;
        cout << "Estimated tvec: " << tvec2 << endl;

        cv::aruco::drawDetectedMarkers(imageRGB, markerCorners, markerIds);
        cv::drawFrameAxes(imageRGB, camera.cameraMatrix, camera.distortionCoefficients, rvec2, tvec2, camera.markerWidth * 1.5, 2);
    }

    imshow("Rendered image", imageRGB);
    waitKey(0);

    return 0;
}
