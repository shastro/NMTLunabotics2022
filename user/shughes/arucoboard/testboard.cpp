#include <iostream>
// #include <cv::aruco::boarjkd>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
using namespace cv;

int main(void){

  cv::Mat inputImage;
  // TODO: Get image from somewhere

  VideoCapture inputVideo;
  int waitTime;
  if(!video.empty()) {
    inputVideo.open(video);
    waitTime = 0;
  } else {
    inputVideo.open(camId);
    waitTime = 10;
  }

  cv::Mat cameraMatrix, distCoeffs;
  cv::aruco::readCameraParameters("webcamcali.yml", cameraMatrix, distCoeffs);
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);

  // Dictionary used in calibration board "ARUCO_MIP_36h12"

  cv::Ptr<cv::aruco::Board> board = cv::aruco::Board::create();


    while(inputVideo.grab()) {
        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        Vec3d rvec, tvec;

        // detect markers
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);

        // refind strategy to detect more markers
        if(refindStrategy)
            aruco::refineDetectedMarkers(image, board, corners, ids, rejected, camMatrix,
                                         distCoeffs);

        // estimate board pose
        int markersOfBoardDetected = 0;
        if(ids.size() > 0)
            markersOfBoardDetected =
                aruco::estimatePoseBoard(corners, ids, board, camMatrix, distCoeffs, rvec, tvec);

        double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
        totalTime += currentTime;
        totalIterations++;
        if(totalIterations % 30 == 0) {
            cout << "Detection Time = " << currentTime * 1000 << " ms "
                 << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
        }

        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
        }

        if(showRejected && rejected.size() > 0)
            aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

        if(markersOfBoardDetected > 0)
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);

        imshow("out", imageCopy);
        char key = (char)waitKey(waitTime);
        if(key == 27) break;
    }

  return 0;
}
