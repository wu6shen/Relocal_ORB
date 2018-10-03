
/**
* This file is part of ORB-SLAM2.
* This file is a modified version of EPnP <http://cvlab.epfl.ch/EPnP/index.php>, see FreeBSD license below.
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

/**
* Copyright (c) 2009, V. Lepetit, EPFL
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* 1. Redistributions of source code must retain the above copyright notice, this
*    list of conditions and the following disclaimer.
* 2. Redistributions in binary form must reproduce the above copyright notice,
*    this list of conditions and the following disclaimer in the documentation
*    and/or other materials provided with the distribution.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
* The views and conclusions contained in the software and documentation are those
* of the authors and should not be interpreted as representing official policies,
*   either expressed or implied, of the FreeBSD Project
*/

#include <iostream>

#include "PoseSolver.h"

#include <vector>
#include <cmath>
#include <opencv2/core/core.hpp>
#include "Thirdparty/DBoW2/DUtils/Random.h"
#include <algorithm>

using namespace std;

namespace ORB_SLAM2
{


Posesolver::Posesolver(const Frame &frame1, const Frame &frame2, const vector<pair<int, int> > &matches12) :
    pws(0), us(0), alphas(0), pcs(0), maximum_number_of_correspondences(0), number_of_correspondences(0), mnInliersi(0),
    mnIterations(0), mnBestInliers(0), N(0)
{
    mvpMapPointMatches.reserve(frame1.N);
    mvP2D1.reserve(frame1.N);
    mvP2D2.reserve(frame1.N);
    mvSigma2.reserve(frame1.N);
    mvP3Dw.reserve(frame1.N);
    mvKeyPointIndices.reserve(frame1.N);
    mvAllIndices.reserve(frame1.N);

	for (size_t i = 0; i < matches12.size(); i++) {
		int l = matches12[i].first;
		int r = matches12[i].second;

		MapPoint* pMP = frame1.mvpLastMapPoints[l];

		const cv::KeyPoint &kp1 = frame1.mvKeysUn[l];
		const cv::KeyPoint &kp2 = frame2.mvKeysUn[r];
		mvP2D1.push_back(kp1.pt);
		mvP2D2.push_back(kp2.pt);
		mvSigma2.push_back(frame1.mvLevelSigma2[kp1.octave]);

		mvKeyPointIndices.push_back(i);

		if (pMP) {
			cv::Mat Pos = pMP->GetWorldPos();
			mvP3Dw.push_back(cv::Point3f(Pos.at<float>(0),Pos.at<float>(1), Pos.at<float>(2)));
			mvAllIndices.push_back(i);
		} else {
			mvP3Dw.push_back(cv::Point3f(-1, -1, -1));
		}
	}

    // Set camera calibration parameters
    fu = frame1.fx;
    fv = frame1.fy;
    uc = frame1.cx;
    vc = frame1.cy;

	Kinv = frame1.mK.inv();

    SetRansacParameters();
}

Posesolver::~Posesolver()
{
  delete [] pws;
  delete [] us;
  delete [] alphas;
  delete [] pcs;
}


void Posesolver::SetRansacParameters(double probability, int minInliers, int maxIterations, int minSet, float epsilon, float th2)
{
    mRansacProb = probability;
    mRansacMinInliers = minInliers;
    mRansacMaxIts = maxIterations;
    mRansacEpsilon = epsilon;
    mRansacMinSet = minSet;
	mRansacTh = th2;

    N = mvP2D1.size(); // number of correspondences

    mvbInliersi.resize(N);

    // Adjust Parameters according to number of correspondences
    int nMinInliers = N*mRansacEpsilon;
    if(nMinInliers<mRansacMinInliers)
        nMinInliers=mRansacMinInliers;
    if(nMinInliers<minSet)
        nMinInliers=minSet;
    mRansacMinInliers = nMinInliers;

	std::cout << "All : " << N << std::endl;
	std::cout << "Min Inlier Num : " << mRansacMinInliers << std::endl;

    if(mRansacEpsilon<(float)mRansacMinInliers/N)
        mRansacEpsilon=(float)mRansacMinInliers/N;

    // Set RANSAC iterations according to probability, epsilon, and max iterations
    int nIterations;

    if(mRansacMinInliers==N)
        nIterations=1;
    else
        nIterations = ceil(log(1-mRansacProb)/log(1-pow(mRansacEpsilon,3)));

    mRansacMaxIts = max(1,min(nIterations,mRansacMaxIts));

    mvMaxError.resize(mvSigma2.size());
    for(size_t i=0; i<mvSigma2.size(); i++)
        mvMaxError[i] = th2;
}

void Posesolver::iterate(int nIterations, bool &bNoMore, vector<bool> &vbInliers, int &nInliers, cv::Mat &R1, cv::Mat &t1, cv::Mat &R2, cv::Mat &t2)
{
    bNoMore = false;
    vbInliers.clear();
    nInliers=0;

    set_maximum_number_of_correspondences(mRansacMinSet);

    if(mvAllIndices.size()<(size_t)mRansacMinInliers)
    {
        bNoMore = true;
        return ;
    }

    vector<size_t> vAvailableIndices;

    int nCurrentIterations = 0;
    while(mnIterations<mRansacMaxIts || nCurrentIterations<nIterations)
    {
        nCurrentIterations++;
        mnIterations++;
        // Compute R1, t1
        reset_correspondences();

        vAvailableIndices = mvAllIndices;
		std::vector<size_t> currentSets;

        // Get min set of points
        for(short i = 0; i < mRansacMinSet; ++i)
        {
            int randi = DUtils::Random::RandomInt(0, vAvailableIndices.size()-1);

            int idx = vAvailableIndices[randi];
			currentSets.push_back(idx);

            add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D1[idx].x,mvP2D1[idx].y);

            vAvailableIndices[randi] = vAvailableIndices.back();
            vAvailableIndices.pop_back();
        }

		if (compute_pose(mRi1, mti1) > mRansacTh) continue;

		// Compute R2, t2
		reset_correspondences();
		for (auto idx : currentSets) {
            add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D2[idx].x,mvP2D2[idx].y);
		}

        compute_pose(mRi2, mti2);


        // Check inliers
        CheckInliers();

        //std::cout << mnInliersi << " " << mRansacMinInliers << std::endl;
        if(mnInliersi>=mRansacMinInliers)
        {
            // If it is the best solution so far, save it
            if(mnInliersi>mnBestInliers)
            {
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						std::cout << mRi1[i][j] << " ";
					}
					std::cout << std::endl;
				}
				for (int i = 0; i < 3; i++) {
					for (int j = 0; j < 3; j++) {
						std::cout << mRi2[i][j] << " ";
					}
					std::cout << std::endl;
				}

                mvbBestInliers = mvbInliersi;
                mnBestInliers = mnInliersi;

                cv::Mat Rcw1(3,3,CV_64F,mRi1);
                cv::Mat tcw1(3,1,CV_64F,mti1);
                Rcw1.convertTo(Rcw1,CV_32F);
                tcw1.convertTo(tcw1,CV_32F);
				R1 = Rcw1, t1 = tcw1;

                cv::Mat Rcw2(3,3,CV_64F,mRi2);
                cv::Mat tcw2(3,1,CV_64F,mti2);
                Rcw2.convertTo(Rcw2,CV_32F);
                tcw2.convertTo(tcw2,CV_32F);
				R2 = Rcw2, t2 = tcw2;

            }
        }
    }

	nInliers=mnBestInliers;
	std::cout << mnIterations << " " << mnBestInliers << std::endl;

    if(mnIterations>=mRansacMaxIts)
    {
        bNoMore=true;
        if(mnBestInliers>=mRansacMinInliers)
        {
            vbInliers = vector<bool>(N,false);
            for(int i=0; i<N; i++)
            {
                if(mvbBestInliers[i])
                    vbInliers[mvKeyPointIndices[i]] = true;
            }
        }
    }
}

bool Posesolver::Refine()
{
	/**
    vector<int> vIndices;
    vIndices.reserve(mvbBestInliers.size());

    for(size_t i=0; i<mvbBestInliers.size(); i++)
    {
        if(mvbBestInliers[i])
        {
            vIndices.push_back(i);
        }
    }

    set_maximum_number_of_correspondences(vIndices.size());

    reset_correspondences();

    for(size_t i=0; i<vIndices.size(); i++)
    {
        int idx = vIndices[i];
        add_correspondence(mvP3Dw[idx].x,mvP3Dw[idx].y,mvP3Dw[idx].z,mvP2D[idx].x,mvP2D[idx].y);
    }

    // Compute camera pose
    compute_pose(mRi, mti);

    // Check inliers
    CheckInliers();

    mnRefinedInliers =mnInliersi;
    mvbRefinedInliers = mvbInliersi;

    if(mnInliersi>mRansacMinInliers)
    {
        cv::Mat Rcw(3,3,CV_64F,mRi);
        cv::Mat tcw(3,1,CV_64F,mti);
        Rcw.convertTo(Rcw,CV_32F);
        tcw.convertTo(tcw,CV_32F);
        mRefinedTcw = cv::Mat::eye(4,4,CV_32F);
        Rcw.copyTo(mRefinedTcw.rowRange(0,3).colRange(0,3));
        tcw.copyTo(mRefinedTcw.rowRange(0,3).col(3));
        return true;
    }

	*/
    return false;
}


void Posesolver::CheckInliers()
{
    mnInliersi=0;

	/**
	for (size_t i = 0; i < mvAllIndices.size(); i++) {
		int indx = mvAllIndices[i];
		cv::Point3f P3Dw = mvP3Dw[indx];
		cv::Point2f P2D = mvP2D1[indx];
        float Xc = mRi1[0][0]*P3Dw.x+mRi1[0][1]*P3Dw.y+mRi1[0][2]*P3Dw.z+mti1[0];
        float Yc = mRi1[1][0]*P3Dw.x+mRi1[1][1]*P3Dw.y+mRi1[1][2]*P3Dw.z+mti1[1];
        float invZc = 1/(mRi1[2][0]*P3Dw.x+mRi1[2][1]*P3Dw.y+mRi1[2][2]*P3Dw.z+mti1[2]);

        double ue = uc + fu * Xc * invZc;
        double ve = vc + fv * Yc * invZc;

        float distX = P2D.x-ue;
        float distY = P2D.y-ve;

        float error2 = distX*distX+distY*distY;

        if(error2<mvMaxError[indx])
        {
            mvbInliersi[indx]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[indx]=false;
        }
	}
	return ;
	*/

	double mRi1t[3][3], mti1t[3];
	for (int i = 0; i < 3; i++) for (int j = 0; j < 3; j++) mRi1t[i][j] = mRi1[j][i];
	for (int i = 0; i < 3; i++) {
		mti1t[i] = 0;
		for (int j = 0; j < 3; j++) {
			mti1t[i] -= mRi1t[i][j] * mti1[j];
		}
	}
	cv::Mat mR21(3, 3, CV_32F), mt21(3, 1, CV_32F), mT21(3, 3, CV_32F);
	for (int i = 0; i < 3; i++) {
		mt21.at<float>(i) = (float)mti2[i];
		for (int j = 0; j < 3; j++) {
			mt21.at<float>(i) += (float)(mRi2[i][j] * mti1t[j]);
			mR21.at<float>(i, j) = 0;
			for (int k = 0; k < 3; k++) {
				mR21.at<float>(i, j) += (float)(mRi2[i][k] * mRi1t[k][j]);
			}
		}
	}

	mT21.at<float>(0, 1) = -mt21.at<float>(2);
	mT21.at<float>(1, 0) = mt21.at<float>(2);
	mT21.at<float>(0, 2) = mt21.at<float>(1);
	mT21.at<float>(2, 0) = -mt21.at<float>(1);
	mT21.at<float>(1, 2) = -mt21.at<float>(0);
	mT21.at<float>(2, 1) = mt21.at<float>(0);
	mT21.at<float>(0, 0) = mT21.at<float>(1, 1) = mT21.at<float>(2, 2) = 0;
	cv::Mat F = Kinv.t() * mT21 * mR21 * Kinv;

    for(int i=0; i<N; i++)
    {
		cv::Mat p1(1, 3, CV_32F);
		cv::Mat p2(1, 3, CV_32F);

		p1.at<float>(0) = mvP2D1[i].x;
		p1.at<float>(1) = mvP2D1[i].y;
		p1.at<float>(2) = 1;
		p2.at<float>(0) = mvP2D1[i].x;
		p2.at<float>(1) = mvP2D1[i].y;
		p2.at<float>(2) = 1;

		cv::Mat errorMat = (p1 * F * p2.t());
        float error2 = fabs(errorMat.at<float>(0));

        if(error2<mvMaxError[i])
        {
            mvbInliersi[i]=true;
            mnInliersi++;
        }
        else
        {
            mvbInliersi[i]=false;
        }
    }
}


void Posesolver::set_maximum_number_of_correspondences(int n)
{
  if (maximum_number_of_correspondences < n) {
    if (pws != 0) delete [] pws;
    if (us != 0) delete [] us;
    if (alphas != 0) delete [] alphas;
    if (pcs != 0) delete [] pcs;

    maximum_number_of_correspondences = n;
    pws = new double[3 * maximum_number_of_correspondences];
    us = new double[2 * maximum_number_of_correspondences];
    alphas = new double[4 * maximum_number_of_correspondences];
    pcs = new double[3 * maximum_number_of_correspondences];
  }
}

void Posesolver::reset_correspondences(void)
{
  number_of_correspondences = 0;
}

void Posesolver::add_correspondence(double X, double Y, double Z, double u, double v)
{
  pws[3 * number_of_correspondences    ] = X;
  pws[3 * number_of_correspondences + 1] = Y;
  pws[3 * number_of_correspondences + 2] = Z;

  us[2 * number_of_correspondences    ] = u;
  us[2 * number_of_correspondences + 1] = v;

  number_of_correspondences++;
}

void Posesolver::choose_control_points(void)
{
  // Take C0 as the reference points centroid:
  cws[0][0] = cws[0][1] = cws[0][2] = 0;
  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      cws[0][j] += pws[3 * i + j];

  for(int j = 0; j < 3; j++)
    cws[0][j] /= number_of_correspondences;


  // Take C1, C2, and C3 from PCA on the reference points:
  CvMat * PW0 = cvCreateMat(number_of_correspondences, 3, CV_64F);

  double pw0tpw0[3 * 3], dc[3], uct[3 * 3];
  CvMat PW0tPW0 = cvMat(3, 3, CV_64F, pw0tpw0);
  CvMat DC      = cvMat(3, 1, CV_64F, dc);
  CvMat UCt     = cvMat(3, 3, CV_64F, uct);

  for(int i = 0; i < number_of_correspondences; i++)
    for(int j = 0; j < 3; j++)
      PW0->data.db[3 * i + j] = pws[3 * i + j] - cws[0][j];

  cvMulTransposed(PW0, &PW0tPW0, 1);
  cvSVD(&PW0tPW0, &DC, &UCt, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);

  cvReleaseMat(&PW0);

  for(int i = 1; i < 4; i++) {
    double k = sqrt(dc[i - 1] / number_of_correspondences);
    for(int j = 0; j < 3; j++)
      cws[i][j] = cws[0][j] + k * uct[3 * (i - 1) + j];
  }
}

void Posesolver::compute_barycentric_coordinates(void)
{
  double cc[3 * 3], cc_inv[3 * 3];
  CvMat CC     = cvMat(3, 3, CV_64F, cc);
  CvMat CC_inv = cvMat(3, 3, CV_64F, cc_inv);

  for(int i = 0; i < 3; i++)
    for(int j = 1; j < 4; j++)
      cc[3 * i + j - 1] = cws[j][i] - cws[0][i];

  cvInvert(&CC, &CC_inv, CV_SVD);
  double * ci = cc_inv;
  for(int i = 0; i < number_of_correspondences; i++) {
    double * pi = pws + 3 * i;
    double * a = alphas + 4 * i;

    for(int j = 0; j < 3; j++)
      a[1 + j] =
	ci[3 * j    ] * (pi[0] - cws[0][0]) +
	ci[3 * j + 1] * (pi[1] - cws[0][1]) +
	ci[3 * j + 2] * (pi[2] - cws[0][2]);
    a[0] = 1.0f - a[1] - a[2] - a[3];
  }
}

void Posesolver::fill_M(CvMat * M,
		  const int row, const double * as, const double u, const double v)
{
  double * M1 = M->data.db + row * 12;
  double * M2 = M1 + 12;

  for(int i = 0; i < 4; i++) {
    M1[3 * i    ] = as[i] * fu;
    M1[3 * i + 1] = 0.0;
    M1[3 * i + 2] = as[i] * (uc - u);

    M2[3 * i    ] = 0.0;
    M2[3 * i + 1] = as[i] * fv;
    M2[3 * i + 2] = as[i] * (vc - v);
  }
}

void Posesolver::compute_ccs(const double * betas, const double * ut)
{
  for(int i = 0; i < 4; i++)
    ccs[i][0] = ccs[i][1] = ccs[i][2] = 0.0f;

  for(int i = 0; i < 4; i++) {
    const double * v = ut + 12 * (11 - i);
    for(int j = 0; j < 4; j++)
      for(int k = 0; k < 3; k++)
	ccs[j][k] += betas[i] * v[3 * j + k];
  }
}

void Posesolver::compute_pcs(void)
{
  for(int i = 0; i < number_of_correspondences; i++) {
    double * a = alphas + 4 * i;
    double * pc = pcs + 3 * i;

    for(int j = 0; j < 3; j++)
      pc[j] = a[0] * ccs[0][j] + a[1] * ccs[1][j] + a[2] * ccs[2][j] + a[3] * ccs[3][j];
  }
}

double Posesolver::compute_pose(double R[3][3], double t[3])
{
  choose_control_points();
  compute_barycentric_coordinates();

  CvMat * M = cvCreateMat(2 * number_of_correspondences, 12, CV_64F);

  for(int i = 0; i < number_of_correspondences; i++)
    fill_M(M, 2 * i, alphas + 4 * i, us[2 * i], us[2 * i + 1]);

  double mtm[12 * 12], d[12], ut[12 * 12];
  CvMat MtM = cvMat(12, 12, CV_64F, mtm);
  CvMat D   = cvMat(12,  1, CV_64F, d);
  CvMat Ut  = cvMat(12, 12, CV_64F, ut);

  cvMulTransposed(M, &MtM, 1);
  cvSVD(&MtM, &D, &Ut, 0, CV_SVD_MODIFY_A | CV_SVD_U_T);
  cvReleaseMat(&M);

  double l_6x10[6 * 10], rho[6];
  CvMat L_6x10 = cvMat(6, 10, CV_64F, l_6x10);
  CvMat Rho    = cvMat(6,  1, CV_64F, rho);

  compute_L_6x10(ut, l_6x10);
  compute_rho(rho);

  double Betas[4][4], rep_errors[4];
  double Rs[4][3][3], ts[4][3];

  find_betas_approx_1(&L_6x10, &Rho, Betas[1]);
  gauss_newton(&L_6x10, &Rho, Betas[1]);
  rep_errors[1] = compute_R_and_t(ut, Betas[1], Rs[1], ts[1]);

  find_betas_approx_2(&L_6x10, &Rho, Betas[2]);
  gauss_newton(&L_6x10, &Rho, Betas[2]);
  rep_errors[2] = compute_R_and_t(ut, Betas[2], Rs[2], ts[2]);

  find_betas_approx_3(&L_6x10, &Rho, Betas[3]);
  gauss_newton(&L_6x10, &Rho, Betas[3]);
  rep_errors[3] = compute_R_and_t(ut, Betas[3], Rs[3], ts[3]);

  int N = 1;
  if (rep_errors[2] < rep_errors[1]) N = 2;
  if (rep_errors[3] < rep_errors[N]) N = 3;

  copy_R_and_t(Rs[N], ts[N], R, t);

  return rep_errors[N];
}

void Posesolver::copy_R_and_t(const double R_src[3][3], const double t_src[3],
			double R_dst[3][3], double t_dst[3])
{
  for(int i = 0; i < 3; i++) {
    for(int j = 0; j < 3; j++)
      R_dst[i][j] = R_src[i][j];
    t_dst[i] = t_src[i];
  }
}

double Posesolver::dist2(const double * p1, const double * p2)
{
  return
    (p1[0] - p2[0]) * (p1[0] - p2[0]) +
    (p1[1] - p2[1]) * (p1[1] - p2[1]) +
    (p1[2] - p2[2]) * (p1[2] - p2[2]);
}

double Posesolver::dot(const double * v1, const double * v2)
{
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

double Posesolver::reprojection_error(const double R[3][3], const double t[3])
{
  double sum2 = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    double * pw = pws + 3 * i;
    double Xc = dot(R[0], pw) + t[0];
    double Yc = dot(R[1], pw) + t[1];
    double inv_Zc = 1.0 / (dot(R[2], pw) + t[2]);
    double ue = uc + fu * Xc * inv_Zc;
    double ve = vc + fv * Yc * inv_Zc;
    double u = us[2 * i], v = us[2 * i + 1];

    sum2 += sqrt( (u - ue) * (u - ue) + (v - ve) * (v - ve) );
  }

  return sum2 / number_of_correspondences;
}

void Posesolver::estimate_R_and_t(double R[3][3], double t[3])
{
  double pc0[3], pw0[3];

  pc0[0] = pc0[1] = pc0[2] = 0.0;
  pw0[0] = pw0[1] = pw0[2] = 0.0;

  for(int i = 0; i < number_of_correspondences; i++) {
    const double * pc = pcs + 3 * i;
    const double * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      pc0[j] += pc[j];
      pw0[j] += pw[j];
    }
  }
  for(int j = 0; j < 3; j++) {
    pc0[j] /= number_of_correspondences;
    pw0[j] /= number_of_correspondences;
  }

  double abt[3 * 3], abt_d[3], abt_u[3 * 3], abt_v[3 * 3];
  CvMat ABt   = cvMat(3, 3, CV_64F, abt);
  CvMat ABt_D = cvMat(3, 1, CV_64F, abt_d);
  CvMat ABt_U = cvMat(3, 3, CV_64F, abt_u);
  CvMat ABt_V = cvMat(3, 3, CV_64F, abt_v);

  cvSetZero(&ABt);
  for(int i = 0; i < number_of_correspondences; i++) {
    double * pc = pcs + 3 * i;
    double * pw = pws + 3 * i;

    for(int j = 0; j < 3; j++) {
      abt[3 * j    ] += (pc[j] - pc0[j]) * (pw[0] - pw0[0]);
      abt[3 * j + 1] += (pc[j] - pc0[j]) * (pw[1] - pw0[1]);
      abt[3 * j + 2] += (pc[j] - pc0[j]) * (pw[2] - pw0[2]);
    }
  }

  cvSVD(&ABt, &ABt_D, &ABt_U, &ABt_V, CV_SVD_MODIFY_A);

  for(int i = 0; i < 3; i++)
    for(int j = 0; j < 3; j++)
      R[i][j] = dot(abt_u + 3 * i, abt_v + 3 * j);

  const double det =
    R[0][0] * R[1][1] * R[2][2] + R[0][1] * R[1][2] * R[2][0] + R[0][2] * R[1][0] * R[2][1] -
    R[0][2] * R[1][1] * R[2][0] - R[0][1] * R[1][0] * R[2][2] - R[0][0] * R[1][2] * R[2][1];

  if (det < 0) {
    R[2][0] = -R[2][0];
    R[2][1] = -R[2][1];
    R[2][2] = -R[2][2];
  }

  t[0] = pc0[0] - dot(R[0], pw0);
  t[1] = pc0[1] - dot(R[1], pw0);
  t[2] = pc0[2] - dot(R[2], pw0);
}

void Posesolver::print_pose(const double R[3][3], const double t[3])
{
  cout << R[0][0] << " " << R[0][1] << " " << R[0][2] << " " << t[0] << endl;
  cout << R[1][0] << " " << R[1][1] << " " << R[1][2] << " " << t[1] << endl;
  cout << R[2][0] << " " << R[2][1] << " " << R[2][2] << " " << t[2] << endl;
}

void Posesolver::solve_for_sign(void)
{
  if (pcs[2] < 0.0) {
    for(int i = 0; i < 4; i++)
      for(int j = 0; j < 3; j++)
	ccs[i][j] = -ccs[i][j];

    for(int i = 0; i < number_of_correspondences; i++) {
      pcs[3 * i    ] = -pcs[3 * i];
      pcs[3 * i + 1] = -pcs[3 * i + 1];
      pcs[3 * i + 2] = -pcs[3 * i + 2];
    }
  }
}

double Posesolver::compute_R_and_t(const double * ut, const double * betas,
			     double R[3][3], double t[3])
{
  compute_ccs(betas, ut);
  compute_pcs();

  solve_for_sign();

  estimate_R_and_t(R, t);

  return reprojection_error(R, t);
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_1 = [B11 B12     B13         B14]

void Posesolver::find_betas_approx_1(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x4[6 * 4], b4[4];
  CvMat L_6x4 = cvMat(6, 4, CV_64F, l_6x4);
  CvMat B4    = cvMat(4, 1, CV_64F, b4);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x4, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x4, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x4, i, 2, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x4, i, 3, cvmGet(L_6x10, i, 6));
  }

  cvSolve(&L_6x4, Rho, &B4, CV_SVD);

  if (b4[0] < 0) {
    betas[0] = sqrt(-b4[0]);
    betas[1] = -b4[1] / betas[0];
    betas[2] = -b4[2] / betas[0];
    betas[3] = -b4[3] / betas[0];
  } else {
    betas[0] = sqrt(b4[0]);
    betas[1] = b4[1] / betas[0];
    betas[2] = b4[2] / betas[0];
    betas[3] = b4[3] / betas[0];
  }
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_2 = [B11 B12 B22                            ]

void Posesolver::find_betas_approx_2(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x3[6 * 3], b3[3];
  CvMat L_6x3  = cvMat(6, 3, CV_64F, l_6x3);
  CvMat B3     = cvMat(3, 1, CV_64F, b3);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x3, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x3, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x3, i, 2, cvmGet(L_6x10, i, 2));
  }

  cvSolve(&L_6x3, Rho, &B3, CV_SVD);

  if (b3[0] < 0) {
    betas[0] = sqrt(-b3[0]);
    betas[1] = (b3[2] < 0) ? sqrt(-b3[2]) : 0.0;
  } else {
    betas[0] = sqrt(b3[0]);
    betas[1] = (b3[2] > 0) ? sqrt(b3[2]) : 0.0;
  }

  if (b3[1] < 0) betas[0] = -betas[0];

  betas[2] = 0.0;
  betas[3] = 0.0;
}

// betas10        = [B11 B12 B22 B13 B23 B33 B14 B24 B34 B44]
// betas_approx_3 = [B11 B12 B22 B13 B23                    ]

void Posesolver::find_betas_approx_3(const CvMat * L_6x10, const CvMat * Rho,
			       double * betas)
{
  double l_6x5[6 * 5], b5[5];
  CvMat L_6x5 = cvMat(6, 5, CV_64F, l_6x5);
  CvMat B5    = cvMat(5, 1, CV_64F, b5);

  for(int i = 0; i < 6; i++) {
    cvmSet(&L_6x5, i, 0, cvmGet(L_6x10, i, 0));
    cvmSet(&L_6x5, i, 1, cvmGet(L_6x10, i, 1));
    cvmSet(&L_6x5, i, 2, cvmGet(L_6x10, i, 2));
    cvmSet(&L_6x5, i, 3, cvmGet(L_6x10, i, 3));
    cvmSet(&L_6x5, i, 4, cvmGet(L_6x10, i, 4));
  }

  cvSolve(&L_6x5, Rho, &B5, CV_SVD);

  if (b5[0] < 0) {
    betas[0] = sqrt(-b5[0]);
    betas[1] = (b5[2] < 0) ? sqrt(-b5[2]) : 0.0;
  } else {
    betas[0] = sqrt(b5[0]);
    betas[1] = (b5[2] > 0) ? sqrt(b5[2]) : 0.0;
  }
  if (b5[1] < 0) betas[0] = -betas[0];
  betas[2] = b5[3] / betas[0];
  betas[3] = 0.0;
}

void Posesolver::compute_L_6x10(const double * ut, double * l_6x10)
{
  const double * v[4];

  v[0] = ut + 12 * 11;
  v[1] = ut + 12 * 10;
  v[2] = ut + 12 *  9;
  v[3] = ut + 12 *  8;

  double dv[4][6][3];

  for(int i = 0; i < 4; i++) {
    int a = 0, b = 1;
    for(int j = 0; j < 6; j++) {
      dv[i][j][0] = v[i][3 * a    ] - v[i][3 * b];
      dv[i][j][1] = v[i][3 * a + 1] - v[i][3 * b + 1];
      dv[i][j][2] = v[i][3 * a + 2] - v[i][3 * b + 2];

      b++;
      if (b > 3) {
	a++;
	b = a + 1;
      }
    }
  }

  for(int i = 0; i < 6; i++) {
    double * row = l_6x10 + 10 * i;

    row[0] =        dot(dv[0][i], dv[0][i]);
    row[1] = 2.0f * dot(dv[0][i], dv[1][i]);
    row[2] =        dot(dv[1][i], dv[1][i]);
    row[3] = 2.0f * dot(dv[0][i], dv[2][i]);
    row[4] = 2.0f * dot(dv[1][i], dv[2][i]);
    row[5] =        dot(dv[2][i], dv[2][i]);
    row[6] = 2.0f * dot(dv[0][i], dv[3][i]);
    row[7] = 2.0f * dot(dv[1][i], dv[3][i]);
    row[8] = 2.0f * dot(dv[2][i], dv[3][i]);
    row[9] =        dot(dv[3][i], dv[3][i]);
  }
}

void Posesolver::compute_rho(double * rho)
{
  rho[0] = dist2(cws[0], cws[1]);
  rho[1] = dist2(cws[0], cws[2]);
  rho[2] = dist2(cws[0], cws[3]);
  rho[3] = dist2(cws[1], cws[2]);
  rho[4] = dist2(cws[1], cws[3]);
  rho[5] = dist2(cws[2], cws[3]);
}

void Posesolver::compute_A_and_b_gauss_newton(const double * l_6x10, const double * rho,
					double betas[4], CvMat * A, CvMat * b)
{
  for(int i = 0; i < 6; i++) {
    const double * rowL = l_6x10 + i * 10;
    double * rowA = A->data.db + i * 4;

    rowA[0] = 2 * rowL[0] * betas[0] +     rowL[1] * betas[1] +     rowL[3] * betas[2] +     rowL[6] * betas[3];
    rowA[1] =     rowL[1] * betas[0] + 2 * rowL[2] * betas[1] +     rowL[4] * betas[2] +     rowL[7] * betas[3];
    rowA[2] =     rowL[3] * betas[0] +     rowL[4] * betas[1] + 2 * rowL[5] * betas[2] +     rowL[8] * betas[3];
    rowA[3] =     rowL[6] * betas[0] +     rowL[7] * betas[1] +     rowL[8] * betas[2] + 2 * rowL[9] * betas[3];

    cvmSet(b, i, 0, rho[i] -
	   (
	    rowL[0] * betas[0] * betas[0] +
	    rowL[1] * betas[0] * betas[1] +
	    rowL[2] * betas[1] * betas[1] +
	    rowL[3] * betas[0] * betas[2] +
	    rowL[4] * betas[1] * betas[2] +
	    rowL[5] * betas[2] * betas[2] +
	    rowL[6] * betas[0] * betas[3] +
	    rowL[7] * betas[1] * betas[3] +
	    rowL[8] * betas[2] * betas[3] +
	    rowL[9] * betas[3] * betas[3]
	    ));
  }
}

void Posesolver::gauss_newton(const CvMat * L_6x10, const CvMat * Rho,
			double betas[4])
{
  const int iterations_number = 5;

  double a[6*4], b[6], x[4];
  CvMat A = cvMat(6, 4, CV_64F, a);
  CvMat B = cvMat(6, 1, CV_64F, b);
  CvMat X = cvMat(4, 1, CV_64F, x);

  for(int k = 0; k < iterations_number; k++) {
    compute_A_and_b_gauss_newton(L_6x10->data.db, Rho->data.db,
				 betas, &A, &B);
    qr_solve(&A, &B, &X);

    for(int i = 0; i < 4; i++)
      betas[i] += x[i];
  }
}

void Posesolver::qr_solve(CvMat * A, CvMat * b, CvMat * X)
{
  static int max_nr = 0;
  static double * A1, * A2;

  const int nr = A->rows;
  const int nc = A->cols;

  if (max_nr != 0 && max_nr < nr) {
    delete [] A1;
    delete [] A2;
  }
  if (max_nr < nr) {
    max_nr = nr;
    A1 = new double[nr];
    A2 = new double[nr];
  }

  double * pA = A->data.db, * ppAkk = pA;
  for(int k = 0; k < nc; k++) {
    double * ppAik = ppAkk, eta = fabs(*ppAik);
    for(int i = k + 1; i < nr; i++) {
      double elt = fabs(*ppAik);
      if (eta < elt) eta = elt;
      ppAik += nc;
    }

    if (eta == 0) {
      A1[k] = A2[k] = 0.0;
      cerr << "God damnit, A is singular, this shouldn't happen." << endl;
      return;
    } else {
      double * ppAik = ppAkk, sum = 0.0, inv_eta = 1. / eta;
      for(int i = k; i < nr; i++) {
	*ppAik *= inv_eta;
	sum += *ppAik * *ppAik;
	ppAik += nc;
      }
      double sigma = sqrt(sum);
      if (*ppAkk < 0)
	sigma = -sigma;
      *ppAkk += sigma;
      A1[k] = sigma * *ppAkk;
      A2[k] = -eta * sigma;
      for(int j = k + 1; j < nc; j++) {
	double * ppAik = ppAkk, sum = 0;
	for(int i = k; i < nr; i++) {
	  sum += *ppAik * ppAik[j - k];
	  ppAik += nc;
	}
	double tau = sum / A1[k];
	ppAik = ppAkk;
	for(int i = k; i < nr; i++) {
	  ppAik[j - k] -= tau * *ppAik;
	  ppAik += nc;
	}
      }
    }
    ppAkk += nc + 1;
  }

  // b <- Qt b
  double * ppAjj = pA, * pb = b->data.db;
  for(int j = 0; j < nc; j++) {
    double * ppAij = ppAjj, tau = 0;
    for(int i = j; i < nr; i++)	{
      tau += *ppAij * pb[i];
      ppAij += nc;
    }
    tau /= A1[j];
    ppAij = ppAjj;
    for(int i = j; i < nr; i++) {
      pb[i] -= tau * *ppAij;
      ppAij += nc;
    }
    ppAjj += nc + 1;
  }

  // X = R-1 b
  double * pX = X->data.db;
  pX[nc - 1] = pb[nc - 1] / A2[nc - 1];
  for(int i = nc - 2; i >= 0; i--) {
    double * ppAij = pA + i * nc + (i + 1), sum = 0;

    for(int j = i + 1; j < nc; j++) {
      sum += *ppAij * pX[j];
      ppAij++;
    }
    pX[i] = (pb[i] - sum) / A2[i];
  }
}



void Posesolver::relative_error(double & rot_err, double & transl_err,
			  const double Rtrue[3][3], const double ttrue[3],
			  const double Rest[3][3],  const double test[3])
{
  double qtrue[4], qest[4];

  mat_to_quat(Rtrue, qtrue);
  mat_to_quat(Rest, qest);

  double rot_err1 = sqrt((qtrue[0] - qest[0]) * (qtrue[0] - qest[0]) +
			 (qtrue[1] - qest[1]) * (qtrue[1] - qest[1]) +
			 (qtrue[2] - qest[2]) * (qtrue[2] - qest[2]) +
			 (qtrue[3] - qest[3]) * (qtrue[3] - qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  double rot_err2 = sqrt((qtrue[0] + qest[0]) * (qtrue[0] + qest[0]) +
			 (qtrue[1] + qest[1]) * (qtrue[1] + qest[1]) +
			 (qtrue[2] + qest[2]) * (qtrue[2] + qest[2]) +
			 (qtrue[3] + qest[3]) * (qtrue[3] + qest[3]) ) /
    sqrt(qtrue[0] * qtrue[0] + qtrue[1] * qtrue[1] + qtrue[2] * qtrue[2] + qtrue[3] * qtrue[3]);

  rot_err = min(rot_err1, rot_err2);

  transl_err =
    sqrt((ttrue[0] - test[0]) * (ttrue[0] - test[0]) +
	 (ttrue[1] - test[1]) * (ttrue[1] - test[1]) +
	 (ttrue[2] - test[2]) * (ttrue[2] - test[2])) /
    sqrt(ttrue[0] * ttrue[0] + ttrue[1] * ttrue[1] + ttrue[2] * ttrue[2]);
}

void Posesolver::mat_to_quat(const double R[3][3], double q[4])
{
  double tr = R[0][0] + R[1][1] + R[2][2];
  double n4;

  if (tr > 0.0f) {
    q[0] = R[1][2] - R[2][1];
    q[1] = R[2][0] - R[0][2];
    q[2] = R[0][1] - R[1][0];
    q[3] = tr + 1.0f;
    n4 = q[3];
  } else if ( (R[0][0] > R[1][1]) && (R[0][0] > R[2][2]) ) {
    q[0] = 1.0f + R[0][0] - R[1][1] - R[2][2];
    q[1] = R[1][0] + R[0][1];
    q[2] = R[2][0] + R[0][2];
    q[3] = R[1][2] - R[2][1];
    n4 = q[0];
  } else if (R[1][1] > R[2][2]) {
    q[0] = R[1][0] + R[0][1];
    q[1] = 1.0f + R[1][1] - R[0][0] - R[2][2];
    q[2] = R[2][1] + R[1][2];
    q[3] = R[2][0] - R[0][2];
    n4 = q[1];
  } else {
    q[0] = R[2][0] + R[0][2];
    q[1] = R[2][1] + R[1][2];
    q[2] = 1.0f + R[2][2] - R[0][0] - R[1][1];
    q[3] = R[0][1] - R[1][0];
    n4 = q[2];
  }
  double scale = 0.5f / double(sqrt(n4));

  q[0] *= scale;
  q[1] *= scale;
  q[2] *= scale;
  q[3] *= scale;
}

} //namespace ORB_SLAM
