#include <SavingCalibrationDataClass.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

#include <opencv2/features2d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

void SavingCalibrationDataClass::saveCameraCalibration2D(Mat cameraMatrix, Mat distanceCoefficients, int cameraIndex)
{
    if (cameraIndex == 0)
    {
        FileStorage file("../data/Camera_0/cameraParams.txt", FileStorage::WRITE);
        file << "camera_matrix" << cameraMatrix << "distortion_coefficients" << distanceCoefficients;
        file.release();
        FileStorage fsCam0("../data/all_matrices/paramsCamera0.txt", FileStorage::WRITE);
        fsCam0 << "cameraMatrix" << cameraMatrix << "distortionCoefficients" << distanceCoefficients;
        fsCam0.release();
    }
    else if (cameraIndex == 1) {
        FileStorage file("../data/Camera_1/cameraParams.txt", FileStorage::WRITE);
        file << "camera_matrix" << cameraMatrix << "distortion_coefficients" << distanceCoefficients;
        file.release();
        FileStorage fsCam1("../data/all_matrices/paramsCamera1.txt", FileStorage::WRITE);
        fsCam1 << "cameraMatrix" << cameraMatrix << "distortionCoefficients" << distanceCoefficients;
        fsCam1.release();
    }
}

void SavingCalibrationDataClass::saveCameraCalibration3D(Mat reprojectionMatrix, Mat fundamentalMatrix,
    Mat rotationMatrix, Mat translationVector)
{
    FileStorage fileReprojectionMatrix("../data/reprojectionMatrix.txt", FileStorage::WRITE);
    fileReprojectionMatrix << "reprojectionMatrix Q" << reprojectionMatrix;
    fileReprojectionMatrix.release();
    FileStorage fileFundamentalMatrix("../data/fundamentalMatrix.txt", FileStorage::WRITE);
    fileFundamentalMatrix << "fundamentalMatrix F" << fundamentalMatrix;
    fileFundamentalMatrix.release();
    FileStorage file("../data/rotationMatrix_translationVector.txt", FileStorage::WRITE);
    file << "rotationMatrix rMat" << rotationMatrix << "translationVector tVec" << translationVector;
    file.release();
    FileStorage fsRelMat("../data/all_matrices/cameraRelationMatrices.txt", FileStorage::WRITE);
    fsRelMat << "reprojectionMatrix Q" << reprojectionMatrix
             << "fundamentalMatrix F" << fundamentalMatrix
             << "rotationMatrix rMat" << rotationMatrix
             << "translationVector tVec" << translationVector;
    fsRelMat.release();
}

void SavingCalibrationDataClass::saveCameraExtrinsics(Mat rectificationRotation0, Mat rectificationRotation1,
    Mat projectionEquations0, Mat projectionEquations1)
{
    FileStorage fileExtrinsics("../data/all_matrices/cameraExtrinsics.txt", FileStorage::WRITE);
    fileExtrinsics << "rectification matrix R0" << rectificationRotation0
                   << "rectification matrix R1" << rectificationRotation1
                   << "projection matrix P0" << projectionEquations0
                   << "projection matrix P1" << projectionEquations1;
}

void SavingCalibrationDataClass::saveCameraRectification3D(Mat mapX0, Mat mapY0, Mat mapX1, Mat mapY1)
{
    FileStorage file("../data/Camera_0/mapX.txt", FileStorage::WRITE);
    file << "mapX" << mapX0;
    file.release();
    FileStorage file1("../data/Camera_0/mapY.txt", FileStorage::WRITE);
    file1 << "mapY" << mapY0;
    file1.release();
    FileStorage file2("../data/Camera_1/mapX.txt", FileStorage::WRITE);
    file2 << "mapX" << mapX1;
    file2.release();
    FileStorage file3("../data/Camera_1/mapY.txt", FileStorage::WRITE);
    file3 << "mapY" << mapY1;
    file3.release();
    FileStorage fsMap0("../data/all_matrices/mapCamera0.txt", FileStorage::WRITE);
    fsMap0 << "mapX" << mapX0
           << "mapY" << mapY0;
    fsMap0.release();
    FileStorage fsMap1("../data/all_matrices/mapCamera1.txt", FileStorage::WRITE);
    fsMap1 << "mapX" << mapX1
           << "mapY" << mapY1;
    fsMap1.release();
}

void SavingCalibrationDataClass::saveOptimizedData(Mat optimalCameraMatrix0, Mat optimalCameraMatrix1,
    Mat distanceCoefficients0, Mat distanceCoefficients1,
    Mat rectificationRotation0, Mat rectificationRotation1,
    Mat projectionEquations0, Mat projectionEquations1,
    Mat optimalMapX0, Mat optimalMapY0, Mat optimalMapX1, Mat optimalMapY1,
    Mat optimalReprojectionMatrix, Mat optimalFundamentalMatrix,
    Mat optimalRotationMatrix, Mat optimalTranslationVector)
{
    FileStorage fsCamMat("../data/optimized_data/optimalCameraIntrinsics.txt", FileStorage::WRITE);
    fsCamMat << "optimalCameraMatrix 0" << optimalCameraMatrix0
             << "optimalDistanceCoefficients 0" << distanceCoefficients0
             << "optimalCameraMatrix 1" << optimalCameraMatrix1
             << "optimalDistanceCoefficients 1" << distanceCoefficients1;
    fsCamMat.release();
    FileStorage fsRecProMat("../data/optimized_data/optimalRectificationProjection.txt", FileStorage::WRITE);
    fsRecProMat << "rectificationRotation0" << rectificationRotation0
                << "projectionEquations0" << projectionEquations0
                << "rectificationRotation1" << rectificationRotation1
                << "projectionEquations1" << projectionEquations1;
    fsRecProMat.release();
    FileStorage fsMap0("../data/optimized_data/optimalMap0.txt", FileStorage::WRITE);
    fsMap0 << "mapX" << optimalMapX0
           << "mapY" << optimalMapY0;
    fsMap0.release();
    FileStorage fsMap1("../data/optimized_data/optimalMap1.txt", FileStorage::WRITE);
    fsMap1 << "mapX" << optimalMapX1
           << "mapY" << optimalMapY1;
    fsMap1.release();
    FileStorage fsExt("../data/optimized_data/cameraExtrinsics.txt", FileStorage::WRITE);
    fsExt << "reprojectionMatrix Q" << optimalReprojectionMatrix
          << "fundamentalMatrix F" << optimalFundamentalMatrix
          << "rotationMatrix rMat" << optimalRotationMatrix
          << "translationVector tVec" << optimalTranslationVector;
    fsExt.release();
}
