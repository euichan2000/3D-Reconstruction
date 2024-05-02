#include <iostream>
#include <vector>
#include <stdio.h>
#include <vector>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv_modules.hpp>
#include <librealsense2/rs.hpp>
#include "universalRobotsKinematics.h"
#include "preprocess.h"
#include <eigen3/Eigen/Eigen>
#include <opencv2/core/eigen.hpp>
using namespace cv;
using namespace std;
using namespace aruco;

int CHECKERBOARD[2]{6, 9};

Mat cameraMatrix = (Mat_<double>(3, 3) << 898.454561024777, 0, 651.946687160551,
                    0, 899.092148378796, 382.332250962326,
                    0, 0, 1);

Mat distCoeffs = (Mat_<double>(1, 5) << 0.008861249213184505, 0.3494378753891278, -0.002162889885546831, 0.006092371833764963, -1.084121168410527);

int main()
{

     vector<String> images;
     string path = "../image";
     glob(path, images); // 파일 목록을 가져오는 glob 함수
     cout << "로드한 이미지 개수 : " << images.size() << endl;
     if (images.size() == 0)
          cout << "이미지가 존재하지 않음! \n"
               << endl;

     Ptr<Dictionary> dictionary = getPredefinedDictionary(DICT_4X4_250);
     Ptr<CharucoBoard> board = CharucoBoard::create(7, 5, 0.035, 0.005, dictionary);
     Ptr<DetectorParameters> params = DetectorParameters::create();
     universalRobots::UR robot; // universalRobots::UR 클래스의 객체 생성
     vector<Mat> R_target2cam, t_target2cam;
     Mat rmatrix_tcp2base, tmatrix_tcp2base;
     Mat rvec_image, tvec_image; // image to camera rotation, transformation vector
     Mat rmatrix_image;
     Mat rvec_tcp2base, rvec_target2cam;

     robot.loadYAML("../calibration.yaml", thetaX, thetaY, X, Y, Z);

     float scene1[6] = {-7.76, -88.08, -110.72, -87.69, 134.65, -19.56};
     const double thetaX = 0 * M_PI / 180;
     const double thetaY = 0 * M_PI / 180;
     const double thetaZ = 180 * M_PI / 180;
     Eigen::Matrix4f matrix_base2tcp, matrix_base2cam, matrix_cam2target, matrix_tcp2cam;
     matrix_tcp2cam << cos(thetaZ) * cos(thetaY), cos(thetaZ) * sin(thetaY) * sin(thetaX) - sin(thetaZ) * cos(thetaX), cos(thetaZ) * sin(thetaY) * cos(thetaX) + sin(thetaZ) * sin(thetaX), 0,
         sin(thetaZ) * cos(thetaY), sin(thetaZ) * sin(thetaY) * sin(thetaX) + cos(thetaZ) * cos(thetaX), sin(thetaZ) * sin(thetaY) * cos(thetaX) - cos(thetaZ) * sin(thetaX), 0,
         -sin(thetaY), cos(thetaY) * sin(thetaX), cos(thetaY) * cos(thetaX), 0.09363,
         0, 0, 0, 1;

     matrix_base2tcp = robot.forwardKinematics(scene1);
     matrix_base2cam = matrix_base2tcp * matrix_tcp2cam;

     for (int i = 0; i < images.size(); i++)
     {

          Mat image;
          char buf[256];
          image = imread(images[i]);

          vector<int> markerIds;
          vector<vector<Point2f>> markerCorners;
          detectMarkers(image, board->dictionary, markerCorners, markerIds, params);
          // if at least one marker detected
          if (markerIds.size() > 0)
          {
               drawDetectedMarkers(image, markerCorners, markerIds);
               vector<Point2f> charucoCorners;
               vector<int> charucoIds;
               interpolateCornersCharuco(markerCorners, markerIds, image, board, charucoCorners, charucoIds, cameraMatrix, distCoeffs);
               // if at least one charuco corner detected
               if (charucoIds.size() > 0)
               {
                    Scalar color = Scalar(255, 0, 0);
                    drawDetectedCornersCharuco(image, charucoCorners, charucoIds, color);

                    bool valid = estimatePoseCharucoBoard(charucoCorners, charucoIds, board, cameraMatrix, distCoeffs, rvec_image, tvec_image);

                    Rodrigues(rvec_image, rmatrix_image); // convert rotation vector to rotation matrix

                    matrix_cam2target = createTransformationMatrix(rmatrix_image, tvec_image);

                    // if charuco pose is valid
                    if (valid)
                    {
                         drawFrameAxes(image, cameraMatrix, distCoeffs, rvec_image, tvec_image, 0.1f);
                    }
               }
          }
          sprintf(buf, "../image/%d.jpg", i);
          imwrite(buf, image);
          imshow("Image", image);
          waitKey(0);
     }
     cout << "base2marker\n"
          << matrix_base2cam * matrix_cam2target << endl;

     return 0;
}