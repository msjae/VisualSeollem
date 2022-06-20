/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include<opencv2/opencv.hpp>
#include "Frame.h"

#define USING_EASY_PROFILER
#include "easy/profiler.h"

namespace ORB_SLAM2
{

// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
class Initializer
{
    typedef pair<int,int> Match;

public:

    // Fix the reference frame
    // Initialize with reference frame, this reference frame is the first frame that SLAM officially starts
    Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);

    // Computes in parallel a fundamental matrix and a homography
    // Selects a model and tries to recover the motion and the structure from motion
    // Use the current frame, that is, use the second frame of the SLAM logic to initialize
    // the entire SLAM , get the R, t between the first two frames, and the point cloud
    bool Initialize(const Frame &CurrentFrame, const vector<int> &vMatches12,
                    cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated);

private:

    /* Assuming that the scene is flat, the Homography matrix(current frame 2 to reference frame 1) is obtained
     * through the first two frame, and the score of the model is obtained
     */
    void FindHomography(vector<bool> &vbMatchesInliers, float &score, cv::Mat &H21);
    /*
     * Assuming that the scene is non-planar, the Fundamental matrix(current frame2 to reference frame 1) is obtained
     * through the first two frames, and the score of the model is obtained
     */
    void FindFundamental(vector<bool> &vbInliers, float &score, cv::Mat &F21);

    // Calculate Homography matrix
    cv::Mat ComputeH21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);
    // Calculate Fundamental matrix
    cv::Mat ComputeF21(const vector<cv::Point2f> &vP1, const vector<cv::Point2f> &vP2);

    // calculate the hypothetical use of the Homography model
    float CheckHomography(const cv::Mat &H21, const cv::Mat &H12, vector<bool> &vbMatchesInliers, float sigma);

    // calculate the hypothetical use of the Fundamental model
    float CheckFundamental(const cv::Mat &F21, vector<bool> &vbMatchesInliers, float sigma);

    // Decompose the F matrix and find the appropriate R, t from the multiple solutions after decomposition
    bool ReconstructF(vector<bool> &vbMatchesInliers, cv::Mat &F21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    // Decompose the H matrix and find the appropriate R, t from the multiple solutions after decomposition
    bool ReconstructH(vector<bool> &vbMatchesInliers, cv::Mat &H21, cv::Mat &K,
                      cv::Mat &R21, cv::Mat &t21, vector<cv::Point3f> &vP3D, vector<bool> &vbTriangulated, float minParallax, int minTriangulated);

    // Through the triangulation method, use the back projection matrix to restore the feature points to 3D points
    void Triangulate(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &P1, const cv::Mat &P2, cv::Mat &x3D);

    // Normalized 3D space point and inter-frame displacement
    void Normalize(const vector<cv::KeyPoint> &vKeys, vector<cv::Point2f> &vNormalizedPoints, cv::Mat &T);

    // ReconstructF calls this function to perform a cheirality check to further find the most suitable
    // solution after F decomposition
    int CheckRT(const cv::Mat &R, const cv::Mat &t, const vector<cv::KeyPoint> &vKeys1, const vector<cv::KeyPoint> &vKeys2,
                       const vector<Match> &vMatches12, vector<bool> &vbInliers,
                       const cv::Mat &K, vector<cv::Point3f> &vP3D, float th2, vector<bool> &vbGood, float &parallax);

    // The F matrix can get the Essential matrix by combining the internal parameters,
    // this function is used to decompose the E matrix, and 4 sets of solutions will be obtained
    void DecomposeE(const cv::Mat &E, cv::Mat &R1, cv::Mat &R2, cv::Mat &t);

    // Keypoints from Reference Frame (Frame 1)
    vector<cv::KeyPoint> mvKeys1;

    // Keypoints from Current Frame (Frame 2)
    vector<cv::KeyPoint> mvKeys2;

    // Current Matches from Reference to Current
    // Reference Frame : 1, Reference Frame : 2
    /// the data structure of Match is pair, mvMatches112 only records the feature points paris from Reference to Current
    vector<Match> mvMatches12;
    /// Record whether each feature point of the Reference Frame has a matching feature point in the Current Frame
    vector<bool> mvbMatched1;

    // Calibration
    cv::Mat mK; /// Camera internal parameters

    // Standard Deviation and Variance
    float mSigma, mSigma2; /// measurement error

    // Ransac max iterations
    int mMaxIterations; /// RANSAC iteration number when calculating Fundamental and Homography matrix

    // Ransac sets
    /*
     * 2-dim container, the size of the outer container is the number of iterations,
     * and the size of the inner container is the point needed to calculate the H or F matrix for each iterations
     */
    vector<vector<size_t> > mvSets;   

};

} //namespace ORB_SLAM

#endif // INITIALIZER_H
