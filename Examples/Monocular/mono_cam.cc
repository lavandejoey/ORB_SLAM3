#include <iostream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <System.h>

using namespace std;

int main(int argc, char **argv) {

    if (argc != 5) {
        cerr << endl
             << "Usage: ./mono_usb path_to_vocabulary path_to_settings (path_to_video | path_to_sequence_folder) trajectory_file_name"
             << endl;
        return 1;
    }

    // Create SLAM system
    ORB_SLAM3::System SLAM(argv[1], // path to vocabulary
                           argv[2], // path to settings
                           ORB_SLAM3::System::MONOCULAR, // Camera mode
                           true // Use viewer
    );
    cv::VideoCapture cap;
    if (argv[3] == string("0")) {
        // Initialize camera
        cap.open(0);
        if (!cap.isOpened()) {
            cerr << "Failed to open camera" << endl;
            return -1;
        }
    } else {
        // Load video
        cap.open(argv[3]);
    }

    cv::Mat frame;
    double timestamp = 0;

    while (!SLAM.isShutDown()) {

        cap >> frame; // get a new frame
        if (frame.empty())
            break;

        timestamp = double(cv::getTickCount()) / cv::getTickFrequency();

        // Process frame
        SLAM.TrackMonocular(frame, timestamp);

        char key = (char) cv::waitKey(1);
        if (key == 'q')
            break;

    }

    // Stop camera and SLAM
    cap.release();
    SLAM.Shutdown();

    return 0;
}