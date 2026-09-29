#include "ThinLensCamera.hpp"
#include "PoseCloud.hpp"
#include <opencv2/core/version.hpp>
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
#include <opencv2/objdetect/aruco_detector.hpp>
#else
// ArUco was moved from opencv_contrib to objdetect in OpenCV 4.7
#include <opencv2/aruco.hpp>
#endif

using namespace cv;
using namespace std;

int main() {

    string folder = "data/aruco/";

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera(folder + "camera.json", folder + "aruco.png");
    cout << camera << endl;

    PoseCloud cloud(folder + "actualPoses.json");
    cout << cloud << endl;

    int keyCode = 0;
    int k = 0;
    while (k < cloud.getPoseCount() && keyCode != 27) {

        Mat image = imread(folder + "image" + to_string(k + 1) + ".png", IMREAD_GRAYSCALE);
        cout << "Detecting image " << k + 1 << "/" << cloud.getPoseCount() << "..." << endl;

        std::vector<int> markerIds;
        std::vector<std::vector < cv::Point2f>> markerCorners, rejectedCandidates;
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
        cv::aruco::DetectorParameters detectorParams = cv::aruco::DetectorParameters();
        detectorParams.cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        cv::aruco::Dictionary dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::ArucoDetector detector(dictionary, detectorParams);
        detector.detectMarkers(image, markerCorners, markerIds, rejectedCandidates);
#else
        cv::Ptr<cv::aruco::DetectorParameters> detectorParams = cv::aruco::DetectorParameters::create();
        detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;

        cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
        cv::aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams, rejectedCandidates);
#endif

        Mat imageRGB;
        merge(std::vector<Mat>({image, image, image}), imageRGB);

        cv::Mat rvec = (Mat_<double>(1, 3) << 0.0, 0.0, 0.0);
        cv::Mat tvec = (Mat_<double>(1, 3) << 0.0, 0.0, -1.0);

        if (!markerIds.empty()) {

            // set coordinate system
            double markerLength = camera.markerWidth;
            cv::Mat objPoints(4, 1, CV_32FC3);
            objPoints.ptr<Vec3f>(0)[3] = Vec3f(-markerLength / 2.f, -markerLength / 2.f, 0);
            objPoints.ptr<Vec3f>(0)[2] = Vec3f(markerLength / 2.f, -markerLength / 2.f, 0);
            objPoints.ptr<Vec3f>(0)[1] = Vec3f(markerLength / 2.f, markerLength / 2.f, 0);
            objPoints.ptr<Vec3f>(0)[0] = Vec3f(-markerLength / 2.f, markerLength / 2.f, 0);
            std::reverse(markerCorners[0].begin(),markerCorners[0].end());
            
            solvePnP(objPoints, markerCorners[0], camera.cameraMatrix, camera.distortionCoefficients, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE);

            cv::aruco::drawDetectedMarkers(imageRGB, markerCorners, markerIds);
            cv::drawFrameAxes(imageRGB, camera.cameraMatrix, camera.distortionCoefficients, rvec, tvec, camera.markerWidth * 1.5, 2);
            cloud.setMarkerCorners(k, markerCorners[0]);
        }
        cloud.setTVec(k, tvec);
        cloud.setRVec(k, rvec);

        imshow("Rendered image", imageRGB);
        keyCode = waitKey(1);
        k++;
    }
    
    if (keyCode != 27) {
        cout << "Writing cloud file..." << endl;
        cloud.write(folder + "estimatedPoses.json");
        cout << "Done!" << endl;
    } else {
        cout << "Stopped by user!" << endl;
    }

    return 0;
}
