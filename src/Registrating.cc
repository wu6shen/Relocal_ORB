#include "Registrating.h"
#include "Thirdparty/sparseicp/ICP.h"
#include <Eigen/Dense>

namespace ORB_SLAM2 {
	Registrating::Registrating(int enoughTh) : mStop(false), mSetMap(false), mEnoughTh(enoughTh), mLastPointsNum(0), mCurrentPointsNum(0) {
		return ;
	}
	void Registrating::SetLastMap(Map *lastMap) {
		mvpLastMap = lastMap->GetAllMapPoints();
		mLastPointsNum = (int)mvpLastMap.size();
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
			for (int j = 0; j < 3; j++) {
				mLastPoints[j][i] = mp.at<float>(j);
			}
		}
	}

	void Registrating::SetCurrentMap(Map *currentMap) {
		mvpCurrentMap = currentMap->GetAllMapPoints();
		std::unique_lock<std::mutex> lock(mMutexSetMap);
		mSetMap = true;
	}

	void Registrating::InsertInCurrentMap(MapPoint *mp) {
		std::unique_lock<std::mutex> lock(mMutexNewMap);
		mvpNewMap.push_back(mp);
	}

	void Registrating::Run() {
		while (1) {
			if (CheckEnoughNewMapPoints() || CheckSetCurrentMap()) {
				ICP();
			}
		}
	}

	void Registrating::SetStop() {
		std::unique_lock<std::mutex> lock(mMutexStop);
		mStop = true;
	}

	bool Registrating::IsStopped() {
		std::unique_lock<std::mutex> lock(mMutexStop);
		return mStop;
	}

	bool Registrating::CheckEnoughNewMapPoints() {
		std::unique_lock<std::mutex> lock(mMutexNewMap);
		if ((int)mvpNewMap.size() > mEnoughTh) {
			for (size_t i = 0; i < mvpNewMap.size(); i++) {
				cv::Mat mp = mvpNewMap[i]->GetWorldPos();
				for (int j = 0; j < 3; j++) {
					mCurrentPoints[j][mCurrentPointsNum] = mp.at<float>(j);
				}
				mCurrentPointsNum++;
			}
			mvpNewMap.clear();
			return true;
		}
		return false;
	}

	bool Registrating::CheckSetCurrentMap() {
		std::unique_lock<std::mutex> lock(mMutexSetMap);
		if (mSetMap) {
			mCurrentPointsNum = mvpCurrentMap.size();
			for (size_t i = 0; i < mvpCurrentMap.size(); i++) {
				cv::Mat mp = mvpCurrentMap[i]->GetWorldPos();
				for (int j = 0; j < 3; j++) {
					mCurrentPoints[j][i] = mp.at<float>(j);
				}
			}
			mSetMap = false;
			return true;
		}
		return false;
	}

	void Registrating::ICP() {
		vertices_source.resize(Eigen::NoChange, mLastPointsNum);
		vertices_target.resize(Eigen::NoChange, mCurrentPointsNum);
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < mLastPointsNum; j++) {
				vertices_source(i, j) = mLastPoints[i][j];
			}
			for (int j = 0; j < mCurrentPointsNum; j++) {
				vertices_target(i, j) = mCurrentPoints[i][j];
			}
		}
		auto tic = std::chrono::steady_clock::now();
		SICP::Parameters pars;
		pars.p = .5;
		pars.max_icp = 15;
		pars.print_icpn = true;
		//std::cout << mLastPoints[0][0] << " " << mLastPoints[1][0] << " " << mLastPoints[2][0] << std::endl;
		SICP::point_to_point(vertices_source, vertices_target, pars);
		//std::cout << vertices_source(0, 0) << " " << vertices_source(1, 0) << " " << vertices_source(2, 0) << std::endl;
		auto toc = std::chrono::steady_clock::now();

		double time_ms = std::chrono::duration <double, std::milli> (toc-tic).count();
		std::cout << "sparseicp registered source to target in: " << time_ms << "ms" << std::endl;

	}

	void Registrating::SetNew() { 
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
			for (int j = 0; j < 3; j++) {
				mLastPoints[j][i] = mp.at<float>(j);
			}
		}
		std::cout << mLastPoints[0][0] << " " << vertices_source(0, 0) << std::endl;
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat now(3, 1, CV_32F);
			for (int j = 0; j < 3; j++) now.at<float>(j) = vertices_source(j, i);
			mvpLastMap[i]->SetWorldPos(now);
		}
		for (int i = 0; i < 3; i++) {
			for (int j = 0; j < mLastPointsNum; j++) {
				vertices_source(i, j) = mLastPoints[i][j];
			}
		}
	}

	/**
	void Registrating::ICPUsePCL() {
		std::cout << "----" << std::endl;
		pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputSource(cloud1);
		icp.setInputTarget(cloud2);
	icp.setEuclideanFitnessEpsilon (0.1);


		icp.align(*result);
		for (size_t i = 0; i < result->size(); i++) {
			pcl::PointXYZ mp = result->points[i];
			cv::Mat now(3, 1, CV_32F);
			for (int j = 0; j < 3; j++) now.at<float>(j) = mp.data[j];
			mvpLastMap[i]->SetWorldPos(now);
		}
		std::cout << "ICP information" << icp.hasConverged() << "score" <<
			icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
	}
	*/
}
