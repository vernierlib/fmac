#include "ThinLensCamera.hpp"

using namespace cv;
using namespace std;

int main() {

    cout << "Loading configuration files..." << endl;
    ThinLensCamera camera("data/matlab/canonCamera.yml", "data/matlab/checkerboard.png");
    cout << camera;

    cv::FileStorage file("data/matlab/canonCamera.yml", cv::FileStorage::READ);
    if (!file.isOpened()) {
        std::cout << "Could not find or read the camera parameter file" << std::endl;
        return 1;
    }
    cv::Mat extrinsicParameters;
    file["extrinsic_parameters"] >> extrinsicParameters;
    file.release();

    int frame = 7;
    cv::Mat rvec = extrinsicParameters(Rect(0, frame, 3, 1));
    cv::Mat tvec = extrinsicParameters(Rect(3, frame, 3, 1));

    Mat image;
    cout << "Rendering... (may take several minutes)" << endl;
    camera.render(rvec, tvec, image);

    cout << "Done!" << endl << "Depth and blur results:" << endl;
    cout << "  | minimum pattern depth: " << camera.minPatternDistance << " " << camera.unit << endl;
    cout << "  | maximum pattern depth: " << camera.maxPatternDistance << " " << camera.unit << endl;
    cout << "  | minimum radius of the circle of confusion: " << camera.circleOfConfusionRadiusInPixels(camera.minPatternDistance) << " px" << endl;
    cout << "  | maximum radius of the circle of confusion: " << camera.circleOfConfusionRadiusInPixels(camera.maxPatternDistance) << " px" << endl;
    
    camera.showMaps();

    image.convertTo(image, CV_32F);
    cv::normalize(image, image, 1.0, 0, cv::NORM_MINMAX);
    imshow("Rendered image", image);
    imwrite("data/matlab/render" + to_string(frame + 1) + ".tiff", image);
   
    string filename = "data/matlab/image" + to_string(frame + 1) + "b.jpg";
    Mat originalImage = imread(filename, IMREAD_GRAYSCALE);
    if (originalImage.empty()) {
        std::cout << "Could not find or read the image: " << filename << std::endl;
        return 1;
    }
    originalImage.convertTo(originalImage, CV_32F);
    cv::normalize(originalImage, originalImage, 1.0, 0, cv::NORM_MINMAX);
    imshow("Original image", originalImage);

    Mat colorImage;
    merge(std::vector<Mat>({image, image, originalImage}), colorImage);
    imwrite("data/matlab/superposition" + to_string(frame + 1) + ".tiff", colorImage);
    imshow("Image comparison", colorImage);
    
    Mat originalZoom1 = originalImage(Rect(1438, 1338, 31, 31));
    resize(originalZoom1, originalZoom1, Size(310, 310),0,0,INTER_NEAREST);
    imwrite("data/matlab/originalZoom1.tiff", originalZoom1);
    imshow("Zoom 1 on the original image", originalZoom1);
    
    Mat imageZoom1 = image(Rect(1437, 1338, 31, 31));
    resize(imageZoom1, imageZoom1, Size(310, 310),0,0,INTER_NEAREST);
    imwrite("data/matlab/imageZoom1.tiff", imageZoom1);
    imshow("Zoom 1 of rendered image", imageZoom1);
    
    Mat originalZoom2 = originalImage(Rect(1413, 358, 31, 31));
    resize(originalZoom2, originalZoom2, Size(310, 310),0,0,INTER_NEAREST);
    imwrite("data/matlab/originalZoom2.tiff", originalZoom2);
    imshow("Zoom 2 on the original image", originalZoom2);
    
    Mat imageZoom2 = image(Rect(1412, 358, 31, 31));
    resize(imageZoom2, imageZoom2, Size(310, 310),0,0,INTER_NEAREST);
    imwrite("data/matlab/imageZoom2.tiff", imageZoom2);
    imshow("Zoom 2 of rendered image", imageZoom2);   

    waitKey(0);

    return 0;
}
