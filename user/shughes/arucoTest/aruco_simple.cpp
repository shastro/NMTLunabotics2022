#include <iostream>
#include <aruco.h>
#include <opencv2/highgui.hpp>

using namespace std;
int main(int argc,char **argv)
{
    cv::Mat im;

    cv::VideoCapture cap;

    cap.open(0);
    //int deviceID = 0;             // 0 = open default camera
    //int apiID = cv::CAP_ANY;      // 0 = autodetect default API

    // open selected camera using selected API
    //cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }


   aruco::CameraParameters camera;
   camera.readFromXMLFile(argv[1]);



   aruco::MarkerDetector Detector;
   Detector.setDictionary("ARUCO_MIP_36h12");

   for (;;) {
      cap.read(im);
      auto markers=Detector.detect(im,camera,0.07);//0.05 is the marker size

      for(auto m:markers){
	  aruco::CvDrawingUtils::draw3dCube(im, m, camera);
          aruco::CvDrawingUtils::draw3dAxis(im,m,camera);
	  m.draw(im);
          cout<<m.Rvec<<" "<<m.Tvec<<endl;
      }

      cv::imshow("image",im);

      cv::waitKey(10);
   }
}


