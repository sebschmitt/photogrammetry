#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/opencv.hpp>
//#define OPENCV_VERSION CVAUX_STR(CV_VERSION_MAJOR)"" CVAUX_STR(CV_VERSION_MINOR)"" CVAUX_STR(CV_VERSION_REVISION)
//#if NDEBUG 
//#pragma comment(lib, "opencv_world" OPENCV_VERSION ".lib")
//#else
//#pragma comment(lib, "opencv_world" OPENCV_VERSION "d.lib")
//#endif

using namespace cv;
using namespace std;

int test() {
    int numBoards = 0;
    int numCornersHor;
    int numCornersVer;

    printf("Enter number of corners along width: ");
    scanf("%d", &numCornersHor);

    printf("Enter number of corners along height: ");
    scanf("%d", &numCornersVer);

    printf("Enter number of boards: ");
    scanf("%d", &numBoards);

    int numSquares = numCornersHor * numCornersVer;
    Size board_size = Size(numCornersHor, numCornersVer);

    VideoCapture capture = VideoCapture(0);

    vector<vector<Point3f>> object_points;
    vector<vector<Point2f>> image_points;
    vector<Point2f> corners;
    int successes=0;

    Mat image;
    Mat gray_image;
    capture >> image;

    vector<Point3f> obj;
    for(int j=0;j<numSquares;j++)
        obj.push_back(Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

    while(successes<numBoards) {
        cvtColor(image, gray_image, COLOR_BGR2GRAY);
        bool found = findChessboardCorners(image, board_size, corners, CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if(found)
        {
            printf("Found Corners");
            cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.1));
            drawChessboardCorners(gray_image, board_size, corners, found);
        }

        imshow("win1", image);
        imshow("win2", gray_image);

        capture >> image;
        int key = waitKey(1);

        if(key==27) 
            return 0;

        if(key==' ' && found!=0) {
            image_points.push_back(corners);
            object_points.push_back(obj);

            printf("Snap stored!");

            successes++;

            if(successes>=numBoards)
                break;
        }
    }

    Mat intrinsic = Mat(3, 3, CV_32FC1);
    Mat distCoeffs;
    vector<Mat> rvecs;
    vector<Mat> tvecs;

    intrinsic.ptr<float>(0)[0] = 1;
    intrinsic.ptr<float>(1)[1] = 1;

    calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

    Mat imageUndistorted;
    while(1)
    {
        capture >> image;
        undistort(image, imageUndistorted, intrinsic, distCoeffs);

        imshow("win1", image);
        imshow("win2", imageUndistorted);
        waitKey(1);
    }
    capture.release();

    return 0;
}
