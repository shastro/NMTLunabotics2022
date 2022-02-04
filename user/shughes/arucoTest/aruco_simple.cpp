#include <iostream>
#include <aruco.h>
#include <opencv2/highgui.hpp>

using namespace std;
int main(int argc,char **argv)
{
   cv::Mat im=cv::imread(argv[1]);

   // aruco::CameraParameters camera;
   // camera.readFromXMLFile(argv[2]);
   // aruco::MarkerDetector Detector;
   // Detector.setDictionary("ARUCO_MIP_36h12");
   // auto markers=Detector.detect(im,camera,0.05);//0.05 is the marker size
   // for(auto m:markers){
   //     aruco::CvDrawingUtils::draw3dAxis(im,m,camera);
   //     cout<<m.Rvec<<" "<<m.Tvec<<endl;
   // }
   cv::imshow("image",im);
   //cout << "bruh\n"; 
   //cv::waitKey(0);
}


