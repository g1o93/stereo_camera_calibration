/**
 *   @file ImageControl.h
 *   @brief this header file will contain all required definitions and functions
 *          for image processing
 */
// Used from 2D Object detection
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include<opencv2/opencv.hpp>

#include <opencv2/calib3d.hpp>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/video/video.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/utility.hpp>


using namespace cv;
using namespace std;

class CameraCalibrationClass
{
public:

    void help();

    /**
     *   This method will be used to take images from one camera.
     *   @param     vector to Mat objects for saving the images
     *              camera index of the camera you want to calibrate
     *              counter for taken images
     *              image resolution for further processing
     */
    void saveImg(vector<Mat>& images,
                 int cameraIndex,
                 int count,
                 Size& imageResolution);

    /**
     *   This method will be used to take images from one camera.
     *   @param     vector to Mat objects for saving the images from default camera
     *              vector to Mat objects for saving the images from not default camera
     *              counter for taken images
     *              image resolution for further processing
     */
    void saveImagesStereo(vector<Mat>& images0,
                          vector<Mat>& images1,
                          int count,
                          Size& imageResolution);

    /**
     *   This method will be used to create known corner points on the chessboard pattern.
     *   @param     number of inner intersections of chessboard
     *              length of one chessboard square
     *              vector for processing the found corners
     */
    void createKnownBoardPosition(Size boardSize,
                                  float squareEdgeLength,
                                  vector<Point3f>& corners);

    void getChessboardCorners(vector<Mat> images,
                              vector<vector<Point2f>>& allFoundCorners,
                              bool showResults = false);

    bool saveCameraCalibration2D(string name,
                                 Mat cameraMatrix,
                                 Mat distanceCoefficients);

    bool cameraCalibration2D(vector<Mat> calibrationImages,
                             Size boardSize,
                             float squareEdgeLength,
                             Mat& cameraMatrix,
                             Mat& distanceCoefficients,
                             vector<vector<Point2f>>& chessboardImageSpacePoints,
                             vector<vector<Point3f>>& worldSpaceCornerPoints);

    void cameraCalibration3D(vector<vector<Point3f>> worldSpaceCornerPoints,
                             vector<vector<Point2f>> chessboardImageSpacePoints0,
                             vector<vector<Point2f>> chessboardImageSpacePoints1,
                             Mat cameraMatrix0,
                             Mat distanceCoefficients0,
                             Mat cameraMatrix1,
                             Mat distanceCoefficients1,
                             Size imageResolution,
                             Mat& rotationMatrix,
                             Mat& translationVector,
                             Mat& essentialMatrix,
                             Mat& fundamentalMatrix,
                             Mat& reprojectionMatrix,
                             Mat& mapx0,
                             Mat& mapy0,
                             Mat& mapx1,
                             Mat& mapy1,
                             Rect& validPixROI0,
                             Rect& validPixROI1,
                             Rect& validDisparityROI);

    // bool saveCameraCalibration3D(string name,
    //                              Mat reprojectionMatrix,
    //                              Mat fundamentalMatrix,
    //                              Mat fundamentalMatrixEpipolar,
    //                              Mat rotationMatrix,
    //                              Mat translationVector);

    // bool saveCameraRectification3D(string nameX,
    //                                string nameX2,
    //                                Mat mapX,
    //                                string nameY,
    //                                string nameY2,
    //                                Mat mapY);

    void optimizeCameraData(const vector<vector<Point3f>> worldSpaceCornerPoints,
                            const vector<vector<Point2f>> chessboardImageSpacePoints0,
                            const vector<vector<Point2f>> chessboardImageSpacePoints1,
                            const Mat cameraMatrix0,
                            const Mat distanceCoefficients0,
                            const Mat cameraMatrix1,
                            const Mat distanceCoefficients1,
                            Rect validPixROI0,
                            Rect validPixROI1,
                            Rect validDisparityROI,
                            const Size imageResolution,
                            Mat& rotationMatrix,
                            Mat& translationVector,
                            Mat& essentialMatrix,
                            Mat& fundamentalMatrix,
                            Mat& reprojectionMatrix,
                            const double alpha,
                            Mat optimalCameraMatrix0,
                            Mat optimalCameraMatrix1,
                            Mat rectificationRotation0,
                            Mat rectificationRotation1,
                            Mat projectionEquations0,
                            Mat projectionEquations1,
                            Mat optimalMapX0,
                            Mat optimalMapY0,
                            Mat optimalMapX1,
                            Mat optimalMapY1);

    void epipolarGeometrie(vector<vector<Point2f>> chessboardImageSpacePoints0,
                           vector<vector<Point2f>> chessboardImageSpacePoints1,
                           Mat& cameraMatrix0,
                           Mat& distanceCoefficients0,
                           Mat& cameraMatrix1,
                           Mat& distanceCoefficients1,
                           Mat& fundamentalMatrixEpipolar,
                           Mat& correspondEpilines0,
                           Mat& correspondEpilines1);

    void computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                   const vector< vector< Point2f > >& imagePoints,
                                   const vector< Mat >& rvecs,
                                   const vector< Mat >& tvecs,
                                   const Mat& cameraMatrix ,
                                   const Mat& distCoeffs);

private:
    /** Threshold for contour extraction */
    double thresholdVal = 160;
};
