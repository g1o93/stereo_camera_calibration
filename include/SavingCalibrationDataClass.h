/**
 *   @file ImageControl.h
 *   @brief this header file will contain all required definitions and functions
 *          for image processing
 */
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

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

class SavingCalibrationDataClass
{
public:

    // void help();
    //
    // /**
    //  *   This method will be used to take images from one camera.
    //  *   @param     vector to Mat objects for saving the images
    //  *              camera index of the camera you want to calibrate
    //  *              counter for taken images
    //  *              image resolution for further processing
    //  */
    // void saveImg(vector<Mat>& images,
    //              int cameraIndex,
    //              int count,
    //              Size& imageResolution);
    //
    // /**
    //  *   This method will be used to take images from one camera.
    //  *   @param     vector to Mat objects for saving the images from default camera
    //  *              vector to Mat objects for saving the images from not default camera
    //  *              counter for taken images
    //  *              image resolution for further processing
    //  */
    // void saveImagesStereo(vector<Mat>& images0,
    //                       vector<Mat>& images1,
    //                       int count,
    //                       Size& imageResolution);

    void saveCameraCalibration2D(Mat cameraMatrix,
                                 Mat distanceCoefficients,
                                 int cameraIndex);

    void saveCameraCalibration3D(Mat reprojectionMatrix,
                                 Mat fundamentalMatrix,
                                 Mat rotationMatrix,
                                 Mat translationVector);

    void saveCameraExtrinsics(Mat rectificationRotation0,
                              Mat rectificationRotation1,
                              Mat projectionEquations0,
                              Mat projectionEquations1);

    void saveCameraRectification3D(Mat mapX0,
                                   Mat mapY0,
                                   Mat mapX1,
                                   Mat mapY1);

    void saveOptimizedData(Mat optimalCameraMatrix0,
                           Mat optimalCameraMatrix1,
                           Mat distanceCoefficients0,
                           Mat distanceCoefficients1,
                           Mat rectificationRotation0,
                           Mat rectificationRotation1,
                           Mat projectionEquations0,
                           Mat projectionEquations1,
                           Mat optimalMapX0,
                           Mat optimalMapY0,
                           Mat optimalMapX1,
                           Mat optimalMapY1,
                           Mat optimalReprojectionMatrix,
                           Mat optimalFundamentalMatrix,
                           Mat optimalRotationMatrix,
                           Mat optimalTranslationVector);

};
