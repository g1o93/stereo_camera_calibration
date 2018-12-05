#include <CameraCalibrationClass.h>
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

const float calibrationSquareDimension = 0.021f; //meters large pattern 0.0256f
const Size chessboardDimensions = Size(9, 6);     // large pattern 5, 4

void CameraCalibrationClass::help(){
    cerr << "Program needs two parameters camera and maxFrames" << endl;
    cerr << "camera_index: which camera should take the images?" << endl;
    cerr << "0 (default, left camera) || 1 (other, right camera)" << endl;
    cerr << "maxFrames: how many pictures should be taken?" << endl;
    cerr << "any integer number" << endl;
}

void CameraCalibrationClass::saveImg(vector<Mat>& images, int cameraIndex, int count, Size& imageResolution){
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    Mat frame, drawToFrame;
    vector<Vec2f> foundPoints;
    bool found = false;
    if (cameraIndex == 0)
    {
        VideoCapture cap;
        if(!cap.open(0)) //test if webcam is open
        {
            cout << " Error: Failed to camera 0 (default)!" << endl;
        }
        cap.open(0);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
        ostringstream name;
        name << "../data/Camera_0/test" << count << ".png";
        cap >> frame;

        images.push_back(frame);
        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if (found)
        {
            imshow("Webcam", drawToFrame);
            imwrite(name.str(), frame);
        }
        else
            imshow("Webcam", frame);
        imageResolution = frame.size();
        cap.release(); //evtl.
    }
    if (cameraIndex == 1)
    {
        VideoCapture cap;
        if(!cap.open(0)) //test if webcam is open
        {
            cout << " Error: Failed to camera 1 (not default camera)!" << endl;
        }
        cap.open(1);
        cap.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
        cap.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
        ostringstream name;
        name << "../data/Camera_1/test" << count << ".png";
        cap >> frame;

        found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
        frame.copyTo(drawToFrame);
        drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
        if (found)
        {
            imshow("Webcam", drawToFrame);
            imwrite(name.str(), frame);
            images.push_back(frame);
        }
        else
            imshow("Webcam", frame);
        imageResolution = frame.size();
        cap.release(); //evtl.
    }
}

void CameraCalibrationClass::saveImagesStereo(vector<Mat>& images0, vector<Mat>& images1, int count, Size& imageResolution)
{
    // open the default camera, use something different from 0 otherwise;
    // Check VideoCapture documentation.
    Mat frame0, drawToFrame0;
    Mat frame1, drawToFrame1;
    vector<Vec2f> foundPoints0;
    vector<Vec2f> foundPoints1;
    bool found0 = false;
    bool found1 = false;
    Mat temp0 ,temp1;

    VideoCapture cap0;
    if(!cap0.open(0)) //test if webcam is open
    {
        cout << " Error: Failed to camera 0 (default)!" << endl;
    }
    cap0.open(0);
    cap0.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap0.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    ostringstream name0;
    name0 << "../data/Camera_0/test" << count << ".png";
    cap0 >> frame0;

    found0 = findChessboardCorners(frame0, chessboardDimensions, foundPoints0, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    frame0.copyTo(drawToFrame0);
    drawChessboardCorners(drawToFrame0, chessboardDimensions, foundPoints0, found0);
    if (found0)
    {
        imshow("Webcam", drawToFrame0);
        temp0 = frame0;
    }
    else
        imshow("Webcam", frame0);
    imageResolution = frame0.size();
    cap0.release(); //evtl.
    waitKey(10);

    VideoCapture cap1;
    if(!cap1.open(0)) //test if webcam is open
    {
        cout << " Error: Failed to camera 1 (not default camera)!" << endl;
    }
    cap1.open(1);
    cap1.set(CV_CAP_PROP_FRAME_WIDTH, 1920);
    cap1.set(CV_CAP_PROP_FRAME_HEIGHT, 1080);
    ostringstream name1;
    name1 << "../data/Camera_1/test" << count << ".png";
    cap1 >> frame1;

    found1 = findChessboardCorners(frame1, chessboardDimensions, foundPoints1, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
    frame1.copyTo(drawToFrame1);
    drawChessboardCorners(drawToFrame1, chessboardDimensions, foundPoints1, found1);
    if (found1)
    {
        imshow("Webcam", drawToFrame1);
        temp1 = frame1;

    }
    else
        imshow("Webcam", frame1);
    imageResolution = frame1.size();
    cap1.release(); //evtl.

    if (found0 && found1) {
        imwrite(name0.str(), temp0);
        imwrite(name1.str(), temp1);
        images0.push_back(temp0);
        images1.push_back(temp1);
    }
    waitKey();
}

void CameraCalibrationClass::createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners)
{
    for (int i = 0; i < boardSize.height; i++)
    {
        for (int j = 0; j < boardSize.width; j++)
        {
            corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
        }
    }
}

void CameraCalibrationClass::getChessboardCorners(vector<Mat> images, vector<vector<Point2f>>& allFoundCorners, bool showResults)
{
    for (vector<Mat>::iterator iter = images.begin(); iter != images.end(); iter++)
    {
        vector<Point2f> pointBuf;
        Mat imageGray;
        bool found = findChessboardCorners(*iter, chessboardDimensions, pointBuf,
            CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);

        if (found)
        {
            cvtColor(*iter, imageGray, CV_BGR2GRAY,1);
            // cout << "image taken with " << images[1].channels() << " channels and type " << images[1].type() << endl;
            cornerSubPix(imageGray, pointBuf, Size(5, 5), Size(-1, -1),
                TermCriteria(TermCriteria::COUNT|TermCriteria::EPS,20,0.03));
            allFoundCorners.push_back(pointBuf);
        }

        if (showResults) {
            drawChessboardCorners(*iter, chessboardDimensions, pointBuf, found);
            imshow("Looking for Corners", *iter);
            waitKey();
        }
    }
}

bool CameraCalibrationClass::saveCameraCalibration2D(string name, Mat cameraMatrix, Mat distanceCoefficients)
{
    ofstream outStream(name);
    if (outStream)
    {
        uint16_t rows = cameraMatrix.rows;
        uint16_t columns = cameraMatrix.cols;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double value = cameraMatrix.at<double>(r, c);
                outStream << value << endl;
            }
        }

        rows = distanceCoefficients.rows;
        columns = distanceCoefficients.cols;

        for (int r = 0; r < rows; r++)
        {
            for (int c = 0; c < columns; c++)
            {
                double value = distanceCoefficients.at<double>(r, c);
                outStream << value << endl;
            }
        }

        outStream.close();
        return true;
    }

    return false;
}

bool CameraCalibrationClass::cameraCalibration2D(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength,
    Mat& cameraMatrix, Mat& distanceCoefficients,
    vector<vector<Point2f>>& chessboardImageSpacePoints, vector<vector<Point3f>>& worldSpaceCornerPoints)
{
    // vector<vector<Point2f>> chessboardImageSpacePoints;
    getChessboardCorners(calibrationImages, chessboardImageSpacePoints, false);
    int numberTakenPictures = chessboardImageSpacePoints.size();
    cout << "recognized chessboard pattern on " << numberTakenPictures << " pictures"<< endl;
    // vector<vector<Point3f>> worldSpaceCornerPoints(1);

    createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornerPoints[0]);
    if (!worldSpaceCornerPoints.empty())
    {
        worldSpaceCornerPoints.resize(chessboardImageSpacePoints.size(), worldSpaceCornerPoints[0]);
        int numberTakenPictures = worldSpaceCornerPoints.size();
        cout << "successfully transformed " << numberTakenPictures << " pictures to world coordinates"<< endl;
        vector<Mat> rVectors, tVectors;
        // distanceCoefficients = Mat::zeros(8, 1, CV_64F);
        cout << "calibrating camera ..." << endl << endl;
        double rms = calibrateCamera(worldSpaceCornerPoints, chessboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
        // computeReprojectionErrors(worldSpaceCornerPoints, chessboardImageSpacePoints, rVectors, tVectors, cameraMatrix, distanceCoefficients);
        // undistortPoints(chessboardImageSpacePoints, chessboardImageSpacePoints, cameraMatrix, distanceCoefficients, Mat(), cameraMatrix);

        cout << "cameraMatrix = "<< endl << " "  << cameraMatrix << endl << endl;
        cout << "distanceCoefficients = "<< endl << " "  << distanceCoefficients << endl << endl;
        cout << "RMS re-projection error " << rms << endl << endl;
    }
    else
        cout << "failed to create worldSpaceCornerPoints" << endl;

    if (!cameraMatrix.empty() && !distanceCoefficients.empty())
        return true;
    else
        return false;
}

void CameraCalibrationClass::cameraCalibration3D(vector<vector<Point3f>> worldSpaceCornerPoints,
    vector<vector<Point2f>> chessboardImageSpacePoints0, vector<vector<Point2f>> chessboardImageSpacePoints1,
    Mat cameraMatrix0, Mat distanceCoefficients0, Mat cameraMatrix1, Mat distanceCoefficients1, Size imageResolution,
    Mat& rotationMatrix, Mat& translationVector, Mat& essentialMatrix, Mat& fundamentalMatrix,
    Mat& reprojectionMatrix, Mat& mapx0, Mat& mapy0, Mat& mapx1, Mat& mapy1,
    Rect& validPixROI0, Rect& validPixROI1, Rect& validDisparityROI)
{
    SavingCalibrationDataClass svngclbrtndtclss;

    Mat rectificationRotation0 = Mat::zeros(3, 3, CV_64F);
    Mat rectificationRotation1 = Mat::zeros(3, 3, CV_64F);
    Mat projectionEquations0 = Mat::zeros(3, 4, CV_64F);
    Mat projectionEquations1 = Mat::zeros(3, 4, CV_64F);

    Mat optimalCameraMatrix0, optimalCameraMatrix1;
    Mat optimalMapX0, optimalMapY0;
    Mat optimalMapX1, optimalMapY1;

    double alpha = 1;

    Mat correspondEpilines0, correspondEpilines1;

    cout << "stereo calibration ..." << endl;

    // epipolarGeometrie(chessboardImageSpacePoints0, chessboardImageSpacePoints1,
    //     cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1,
    //     fundamentalMatrixEpipolar, correspondEpilines0, correspondEpilines1);

    double error = stereoCalibrate(worldSpaceCornerPoints, chessboardImageSpacePoints0, chessboardImageSpacePoints1,
        cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1, imageResolution,
        rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
        CV_CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::COUNT, 30, 0));

    stereoRectify(cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1, imageResolution,
        rotationMatrix, translationVector,
        rectificationRotation0, rectificationRotation1, projectionEquations0, projectionEquations1,
        reprojectionMatrix, CV_CALIB_ZERO_DISPARITY, alpha, imageResolution, &validPixROI0, &validPixROI1);

    initUndistortRectifyMap(cameraMatrix0, distanceCoefficients0, rectificationRotation0, projectionEquations0,
        imageResolution, CV_32FC1, mapx0, mapy0);
    initUndistortRectifyMap(cameraMatrix1, distanceCoefficients1, rectificationRotation1, projectionEquations1,
        imageResolution, CV_32FC1, mapx1, mapy1);
    if (mapx0.empty() || mapy0.empty() || mapx1.empty() || mapy1.empty()) {
        cout << "saving not possible because one of tables is empty" << endl;
    }
    svngclbrtndtclss.saveCameraCalibration3D(
        reprojectionMatrix, fundamentalMatrix, rotationMatrix, translationVector);
    svngclbrtndtclss.saveCameraExtrinsics(
        rectificationRotation0, rectificationRotation1, projectionEquations0, projectionEquations1);
    svngclbrtndtclss.saveCameraRectification3D(
        mapx0, mapy0, mapx1, mapy1);

    if (!(error = 0) && !mapx0.empty() && !mapx1.empty()) {
        cout << "stereo calibration successful" << endl;
        cout << "calibration error: " << error << endl << "re-projection error as root mean square (RMS)"<< endl;
        cout << "fundamentalMatrix (F) = "<< endl << " "  << fundamentalMatrix << endl << endl;
        cout << "reprojectionMatrix (Q) = "<< endl << " "  << reprojectionMatrix << endl << endl;
        cout << "saving of all matrices and lookupTables successful" << endl;

    }
    else if(error == 0) {
        cout << "stereo calibration failed: error in cameraCalibration3D, line 205" << endl;
        cout << "failed to calculate stereoCalibrate function" << endl;
    }
    else if(!mapx0.empty() || !mapx1.empty()) {
        cout << "stereo calibration failed: error in cameraCalibration3D, line 215-218" << endl;
        cout << "failed to initUndistortRectifyMap" << endl;
    }

    optimizeCameraData(worldSpaceCornerPoints, chessboardImageSpacePoints0, chessboardImageSpacePoints1,
        cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1,
        validPixROI0, validPixROI1, validDisparityROI, imageResolution,
        rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix, reprojectionMatrix,
        alpha, optimalCameraMatrix0, optimalCameraMatrix1,
        rectificationRotation0, rectificationRotation1, projectionEquations0, projectionEquations1,
        optimalMapX0, optimalMapY0, optimalMapX1, optimalMapY1);
}

void CameraCalibrationClass::optimizeCameraData(const vector<vector<Point3f>> worldSpaceCornerPoints,
    const vector<vector<Point2f>> chessboardImageSpacePoints0, const vector<vector<Point2f>> chessboardImageSpacePoints1,
    const Mat cameraMatrix0, const Mat distanceCoefficients0, const Mat cameraMatrix1, const Mat distanceCoefficients1,
    Rect validPixROI0, Rect validPixROI1, Rect validDisparityROI, const Size imageResolution,
    Mat& rotationMatrix, Mat& translationVector, Mat& essentialMatrix, Mat& fundamentalMatrix,
    Mat& reprojectionMatrix, const double alpha, Mat optimalCameraMatrix0, Mat optimalCameraMatrix1,
    Mat rectificationRotation0, Mat rectificationRotation1, Mat projectionEquations0, Mat projectionEquations1,
    Mat optimalMapX0, Mat optimalMapY0, Mat optimalMapX1, Mat optimalMapY1)
{

    cout << "region of interest size in image 0 " << validPixROI0.size() << endl
    << "region of interest size in image 1 " << validPixROI1.size() << endl;
    optimalCameraMatrix0 = getOptimalNewCameraMatrix(cameraMatrix0, distanceCoefficients0, imageResolution,
        alpha, imageResolution, &validPixROI0);
    optimalCameraMatrix1 = getOptimalNewCameraMatrix(cameraMatrix1, distanceCoefficients1, imageResolution,
        alpha, imageResolution, &validPixROI1);

    double error = stereoCalibrate(worldSpaceCornerPoints, chessboardImageSpacePoints0, chessboardImageSpacePoints1,
        cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1, imageResolution,
        rotationMatrix, translationVector, essentialMatrix, fundamentalMatrix,
        CV_CALIB_USE_INTRINSIC_GUESS, TermCriteria(TermCriteria::COUNT, 30, 0));
    cout << "stereo calibration successful" << endl;
    cout << "calibration error: " << error << endl << "re-projection error as root mean square (RMS)"<< endl;

    stereoRectify(cameraMatrix0, distanceCoefficients0, cameraMatrix1, distanceCoefficients1, imageResolution,
        rotationMatrix, translationVector,
        rectificationRotation0, rectificationRotation1, projectionEquations0, projectionEquations1,
        reprojectionMatrix, CV_CALIB_ZERO_DISPARITY, alpha, imageResolution, &validPixROI0, &validPixROI1);

    validDisparityROI = getValidDisparityROI(validPixROI1, validPixROI1, 0, 128, 7);

    initUndistortRectifyMap(optimalCameraMatrix0, distanceCoefficients0, rectificationRotation0, projectionEquations0,
        imageResolution, CV_32FC2, optimalMapX0, optimalMapY0);
    initUndistortRectifyMap(optimalCameraMatrix1, distanceCoefficients1, rectificationRotation1, projectionEquations1,
        imageResolution, CV_32FC2, optimalMapX1, optimalMapY1);

    SavingCalibrationDataClass svngclbrtndtclss;
    svngclbrtndtclss.saveOptimizedData(optimalCameraMatrix0, optimalCameraMatrix1,
        distanceCoefficients0, distanceCoefficients1,
        rectificationRotation0, rectificationRotation1, projectionEquations0, projectionEquations1,
        optimalMapX0, optimalMapY0, optimalMapX1, optimalMapY1,
        reprojectionMatrix, fundamentalMatrix, rotationMatrix, translationVector);

    Mat cameraDiff = cameraMatrix0 - optimalCameraMatrix0;
    cout
    << "the optimization created cameraMatrix for camera 0 which differs by the following from the original matrix"
    << cameraDiff << endl << endl;
}

void CameraCalibrationClass::epipolarGeometrie(vector<vector<Point2f>> chessboardImageSpacePoints0, vector<vector<Point2f>> chessboardImageSpacePoints1,
    Mat& cameraMatrix0, Mat& distanceCoefficients0, Mat& cameraMatrix1, Mat& distanceCoefficients1,
    Mat& fundamentalMatrixEpipolar, Mat& correspondEpilines0, Mat& correspondEpilines1)
{


    undistortPoints(chessboardImageSpacePoints0[0], chessboardImageSpacePoints0[0], cameraMatrix0, distanceCoefficients0);
    undistortPoints(chessboardImageSpacePoints1[1], chessboardImageSpacePoints1[1], cameraMatrix1, distanceCoefficients1);
    Mat mask;
    fundamentalMatrixEpipolar = findFundamentalMat(chessboardImageSpacePoints0, chessboardImageSpacePoints1, mask,
        CV_FM_RANSAC, 1.0, 0.99);
    cout << "fundamentalMatrix calculated by epipolarGeometrie " << fundamentalMatrixEpipolar << endl << "mask " << mask << endl << endl;
    if (fundamentalMatrixEpipolar.empty())
    {
        cout << "failed to find fundamental matrix" << endl;
    }

    correctMatches(fundamentalMatrixEpipolar,
        chessboardImageSpacePoints0, chessboardImageSpacePoints1,
        chessboardImageSpacePoints0, chessboardImageSpacePoints1);
    Mat correspondEpilines;
    // Mat correspondent_lines2;
    computeCorrespondEpilines(chessboardImageSpacePoints0, 1, fundamentalMatrixEpipolar, correspondEpilines);
    correspondEpilines0 = correspondEpilines;
    if (correspondEpilines0.empty())
        cout << "failed to compute epipolar lines" << endl;
    computeCorrespondEpilines(chessboardImageSpacePoints1, 2, fundamentalMatrixEpipolar, correspondEpilines);
    correspondEpilines1 = correspondEpilines;
    if (correspondEpilines1.empty())
        cout << "failed to compute epipolar lines" << endl;
}

void CameraCalibrationClass::computeReprojectionErrors(const vector< vector< Point3f > >& objectPoints,
                                 const vector< vector< Point2f > >& imagePoints,
                                 const vector< Mat >& rvecs, const vector< Mat >& tvecs,
                                 const Mat& cameraMatrix , const Mat& distCoeffs)
{
    vector< Point2f > imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    vector< float > perViewErrors;
    perViewErrors.resize(objectPoints.size());

    for (i = 0; i < (int)objectPoints.size(); ++i)
    {
        projectPoints(Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePoints2);
        err = norm(Mat(imagePoints[i]), Mat(imagePoints2), CV_L2);
        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) sqrt(err*err/n);
        // totalErr += err;
        totalErr += err*err;
        totalPoints += n;
    }
    // double sqrtError = sqrt(totalErr/objectPoints.size());
    double sqrtError = sqrt(totalErr/totalPoints);
    cout << "the reprojection error of image 0 is " << sqrtError << endl;
}
