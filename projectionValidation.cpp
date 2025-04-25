#include "ThinLensCamera.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/opencv/left_camera.yml", "data/opencv/chessboard.png");
    cout << camera;

    cv::FileStorage file("data/opencv/left_camera.yml", cv::FileStorage::READ);
    if (!file.isOpened()) {
        std::cout << "Could not find or read the camera parameter file" << std::endl;
        return 1;
    }
    cv::Mat extrinsicParameters;
    file["extrinsic_parameters"] >> extrinsicParameters;
    file.release();

    for (int frame = 0; frame < extrinsicParameters.rows; frame++) {

        Mat rvec = extrinsicParameters(Rect(0, frame, 3, 1));
        Mat tvec = extrinsicParameters(Rect(3, frame, 3, 1));

        Mat image;
        cout << "Rendering..." << endl;
        camera.render(rvec, tvec, image);

        imwrite("data/opencv/outputImage" + to_string(frame + 1) + ".jpg", image);
        image.convertTo(image, CV_64F);
        normalize(image, image, 1.0, 0, cv::NORM_MINMAX);
        imshow("Rendered image", image);
        moveWindow("Rendered image", image.cols, 0);

        string filename = "data/opencv/left" + to_string(frame + 1) + ".jpg";
        Mat originalImage = imread(filename, IMREAD_GRAYSCALE);
        if (originalImage.empty()) {
            std::cout << "Could not find or read the image: " << filename << std::endl;
            return 1;
        }
        originalImage.convertTo(originalImage, CV_64F);
        normalize(originalImage, originalImage, 1.0, 0, cv::NORM_MINMAX);
        Mat originalImageRGB;
        merge(std::vector<Mat>({originalImage, originalImage, originalImage}), originalImageRGB);
        drawFrameAxes(originalImageRGB, camera.cameraMatrix, camera.distortionCoefficients, rvec, tvec, camera.markerWidth, 2);

        imshow("Original image", originalImageRGB);
        moveWindow("Original image", 0, 0);

        Mat colorImage;
        merge(std::vector<Mat>({image, image, originalImage}), colorImage);
        imshow("Image comparison", colorImage);
        moveWindow("Image comparison", 0, image.rows);

        waitKey(0);
    }

    return 0;
}
