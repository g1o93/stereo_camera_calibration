#include <CameraCalibrationClass.h>
#include <SavingCalibrationDataClass.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/affine.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/core/utility.hpp>
#include <opencv2/calib3d.hpp>
#include<opencv2/opencv.hpp>

#include <sstream>
#include <iostream>
#include <fstream>

using namespace std;
using namespace cv;

const float calibrationSquareDimension = 0.021f; //meters old value 0.0256f
const Size chessboardDimensions = Size(9, 6);   // old value 5,4

int main(int argc, char const** argv)
{
    CameraCalibrationClass cmrclbclss;
    SavingCalibrationDataClass svngclbrtndtclss;
    if (String(argv[1]).compare("-h") == 0) {
        cmrclbclss.help();
        return 0;
    }
    if (argc > 1) {
        cmrclbclss.help();
        return 0;
    }


    int cameraIndex = 0;
    int cameraCount;
    int maxFrames;

    printf("Enter number of pictures for camera calibration: ");
    scanf("%d", &maxFrames);

    printf("Enter number of cameras you want to calibrate: ");
    scanf("%d", &cameraCount);
    int cameraCountTemp = cameraCount;
    if (cameraCount == 1) {
        printf("Enter index of camera you want to calibrate: ");
        scanf("%d", &cameraIndex);
        cameraCountTemp = cameraIndex + 1;
    }
    Mat frame;
    Mat drawToFrame;
    Mat image;

    bool success = false;
    Size imageResolution;

    Mat cameraMatrix = Mat::zeros(3, 3, CV_64F);
    Mat cameraMatrix0 = Mat::zeros(3, 3, CV_64F);
    Mat cameraMatrix1 = Mat::zeros(3, 3, CV_64F);
    Mat distanceCoefficients = Mat::zeros(8, 1, CV_64F);
    Mat distanceCoefficients0 = distanceCoefficients;
    Mat distanceCoefficients1 = distanceCoefficients;
    Mat mapx0, mapy0, mapx1, mapy1;

    vector<vector<Point2f>> markerCorner, rejectedCandidates;
    vector<vector<Point2f>> imagePoints, imagePoints0, imagePoints1;
    vector<vector<Point3f>> objectPoints(1), objectPoints0(1), objectPoints1(1);

    Mat rotationMatrix;
    Mat translationVector;
    Mat essentialMatrix = Mat::eye(3, 3, CV_64F);       // Matrix E
    Mat fundamentalMatrix = Mat::eye(3, 3, CV_64F);     // Matrix F
    // Mat fundamentalMatrixEpipolar = Mat::eye(3, 3, CV_64F);
    Mat reprojectionMatrix = Mat::zeros(4, 4, CV_64F);  // Matrix Q

    Rect validPixROI0;
    Rect validPixROI1;
    Rect validDisparityROI;

    if (!(cameraCount == 2))
    {

        for (; cameraIndex < cameraCountTemp; cameraIndex++)
        {
            vector<Mat> savedImages;
            vector<Mat> savedImages0;
            vector<Mat> savedImages1;
            // vector<vector<Point2f>> imagePoints, imagePoints0, imagePoints1;
            // vector<vector<Point3f>> objectPoints(1);

            for (int count = 0; count < maxFrames; count++)
            {
                cmrclbclss.saveImg(savedImages, cameraIndex, count, imageResolution); //get image from webcam
                waitKey(1000);
            }
            cout << endl << "All images for calibration have been taken!" << endl;
            cout << "saved " << savedImages.size() << " images for camera " << cameraIndex << endl;
            cout << "the images have size: " << imageResolution << endl;
            if (savedImages.empty())
            {
                cout << "failed to save images" << endl;
            }
            success = cmrclbclss.cameraCalibration2D(
                savedImages, chessboardDimensions, calibrationSquareDimension,
                cameraMatrix, distanceCoefficients, imagePoints, objectPoints);
            ostringstream name;
            if (cameraIndex == 0)
            {
                name << "../data/Camera_0/calibrationMatrix_distanceCoefficients.txt";
                imagePoints0 = imagePoints;
                imagePoints.clear();

                Mat cameraMatrix0 = cameraMatrix;
                Mat distanceCoefficients0 = distanceCoefficients;
                objectPoints.resize(imagePoints.size(), objectPoints[0]);
                objectPoints.resize(1);
            }
            if (cameraIndex == 1) {
                name << "../data/Camera_1/calibrationMatrix_distanceCoefficients.txt";
                imagePoints1 = imagePoints;
                imagePoints.clear();
                Mat cameraMatrix1 = cameraMatrix;
                Mat distanceCoefficients1 = distanceCoefficients;
            }

            success = cmrclbclss.saveCameraCalibration2D(name.str(), cameraMatrix, distanceCoefficients);
            if (success)
                cout << "saving camera "<< cameraIndex << " matrix successful" << endl << endl;
            else
                cout << "saving camera "<< cameraIndex << " matrix failed" << endl << endl;
        }

    }

    // stereo calibration
    if (cameraCount == 2)
    {
        vector<Mat> savedImages;
        vector<Mat> savedImages0;
        vector<Mat> savedImages1;

        for (int count = 0; count < maxFrames; count++)
        {
            cmrclbclss.saveImagesStereo(savedImages0, savedImages1, count, imageResolution); //get image from webcam
            // if (savedImages1.empty()) {
            //     cout << endl << "Error: image1 empty" << endl;
            //     return 0;
            // }
        }
        cout << endl << "All images for calibration taken!" << endl;

        cout << "saved " << savedImages0.size() << " images for camera 0" << endl;
        cout << "the images have size: " << imageResolution << endl;
        if (savedImages0.empty())
        {
            cout << "failed to save images for camera 0" << endl;
        }

        cout << "saved " << savedImages1.size() << " images for camera 1" << endl;
        cout << "the images have size: " << imageResolution << endl << endl;
        if (savedImages1.empty())
        {
            cout << "failed to save images for camera 1" << endl;
        }

        success = cmrclbclss.cameraCalibration2D(
            savedImages0, chessboardDimensions, calibrationSquareDimension,
            cameraMatrix0, distanceCoefficients0, imagePoints0, objectPoints0);
        if (success)
            cout << "calibration of camera 0 successful" << endl << endl;
        else
            cout << "calibration of camera 0 failed" << endl << endl;
        success = cmrclbclss.cameraCalibration2D(
            savedImages1, chessboardDimensions, calibrationSquareDimension,
            cameraMatrix1, distanceCoefficients1, imagePoints1, objectPoints1);
        if (success)
            cout << "calibration of camera 1 successful" << endl << endl;
        else
            cout << "calibration of camera 1 failed" << endl << endl;

        svngclbrtndtclss.saveCameraCalibration2D(cameraMatrix0, distanceCoefficients0, 0);
        svngclbrtndtclss.saveCameraCalibration2D(cameraMatrix1, distanceCoefficients1, 1);
        ostringstream name0;
        ostringstream name1;
        name0 << "../data/Camera_0/calibrationMatrix_distanceCoefficients.txt";
        name1 << "../data/Camera_1/calibrationMatrix_distanceCoefficients.txt";

        success = cmrclbclss.saveCameraCalibration2D(name0.str(), cameraMatrix0, distanceCoefficients0);
        if (success)
            cout << "saving camera 0 matrix successful" << endl;
        else
            cout << "saving camera 0 matrix failed" << endl;
        success = cmrclbclss.saveCameraCalibration2D(name1.str(), cameraMatrix1, distanceCoefficients1);
        if (success)
            cout << "saving camera 1 matrix successful" << endl << endl;
        else
            cout << "saving camera 1 matrix failed" << endl << endl;

        if (imagePoints0.empty() || imagePoints1.empty()) {
            cout << "no data for stereo calibration" << endl;
            return 0;
        }

        cmrclbclss.cameraCalibration3D(objectPoints0, imagePoints0, imagePoints1,
            cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1, imageResolution,
            rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
            reprojectionMatrix, mapx0, mapy0, mapx1, mapy1, validPixROI0, validPixROI1, validDisparityROI);

        // svngclbrtndtclss.saveCameraCalibration3D(reprojectionMatrix, fundamentalMatrix, rotationMatrix, translationVector);
        // ostringstream nameNew;
        // nameNew << "../data/reprojectionMatrix.txt";
        // success = cmrclbclss.saveCameraCalibration3D(nameNew.str(), reprojectionMatrix, fundamentalMatrix, fundamentalMatrixEpipolar,
        // rotationMatrix, translationVector);
        // if (success)
        //     cout << "saving reprojection matrix successful" << endl;
        // else
        //     cout << "saving reprojection matrix failed" << endl;

        // ostringstream nameX0;
        // ostringstream nameY0;
        // ostringstream nameX01;
        // ostringstream nameY01;
        // nameX0 << "../data/Camera_0/mapX_lookup_table.txt";
        // nameY0 << "../data/Camera_0/mapY_lookup_table.txt";
        // nameX01 << "../data/Camera_0/mapX_lookup_table_Mat_format.txt";
        // nameY01 << "../data/Camera_0/mapY_lookup_table_Mat_format.txt";
        // success = cmrclbclss.saveCameraRectification3D(nameX0.str(), nameX01.str(), mapx0, nameY0.str(), nameY01.str(), mapy0);
        // if (success)
        // {
        //     cout << "saving lookup-tables for camera 0 successful at" << endl;
        //     cout << nameX0.str() << endl;
        //     cout << nameY0.str() << endl;
        // }
        // else
        //     cout << "saving lookup-tables for camera 0 failed" << endl;

        // ostringstream nameX1;
        // ostringstream nameY1;
        // ostringstream nameX11;
        // ostringstream nameY11;
        // nameX1 << "../data/Camera_1/mapX_lookup_table.txt";
        // nameY1 << "../data/Camera_1/mapY_lookup_table.txt";
        // nameX11 << "../data/Camera_1/mapX_lookup_table_Mat_format.txt";
        // nameY11 << "../data/Camera_1/mapY_lookup_table_Mat_format.txt";
        // success = cmrclbclss.saveCameraRectification3D(nameX1.str(), nameX11.str(), mapx1, nameY1.str(), nameY11.str(), mapy1);
        // if (success)
        // {
        //     cout << "saving lookup-tables for camera 1 successful at" << endl;
        //     cout << nameX1.str() << endl;
        //     cout << nameY1.str() << endl;
        // }
        // else
        //     cout << "saving lookup-tables for camera 1 failed" << endl;
        // svngclbrtndtclss.saveCameraRectification3D(mapx0, mapy0, mapx1, mapy1);
    }

    else
    {
            cout << "all  "<< cameraCount << " have successfully been calibrated" << endl;
    }
    return 0;
}
