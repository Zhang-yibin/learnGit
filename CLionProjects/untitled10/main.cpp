#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/core.hpp>
#include <opencv4/opencv2/core/mat.hpp>
#include <opencv4/opencv2/core/types.hpp>
#include <opencv4/opencv2/dnn.hpp>
#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/imgcodecs.hpp>
#include <opencv4/opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

int main() {
    // Create a VideoCapture object to access the default camera (0)
    cv::VideoCapture cap(0);

    if (!cap.isOpened()) {
        std::cerr << "Failed to open default camera!" << std::endl;
        return -1;
    }

    while (true) {
        // Read a frame from the camera
