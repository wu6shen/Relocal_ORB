#include "Registrating.h"
#include "Thirdparty/sparseicp/ICP.h"
#include <Eigen/Dense>

namespace ORB_SLAM2 {
	Registrating::Registrating(int enoughTh) : mStop(false), mSetMap(false), mEnoughTh(enoughTh), mLastPointsNum(0), mCurrentPointsNum(0), mpMap(NULL) {
		return ;
	}
	void Registrating::SetLastMap(Map *lastMap) {
		mvpLastMap = lastMap->GetAllMapPoints();
		mLastPointsNum = (int)mvpLastMap.size();

		mMatches12.resize(mLastPointsNum);
		for (auto &it : mMatches12) {
			it = NULL;
		}
	}

	void Registrating::SetCurrentMap(Map *currentMap) {
		std::unique_lock<std::mutex> lock(mMutexSetMap);
		mpMap = currentMap;
		mSetMap = true;
	}

	bool Registrating::NeedICP() {
		std::unique_lock<std::mutex> lock(mMutexSetMap);
		if (mpMap == NULL) return false;
		if (mpMap->GetChangeNum() > mEnoughTh) {
			mpMap->SetChangenumZero();
			return true;
		}
		return false;
	}

	void Registrating::Run() {
		while (1) {
			if (NeedICP()) {
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

	void Registrating::ICP() {
		mCurrentPointsNum = 0;
		mvpCurrentMap = mpMap->GetAllMapPoints();
		for (size_t i = 0; i < mvpCurrentMap.size(); i++) {
			cv::Mat mp = mvpCurrentMap[i]->GetWorldPos();
			if (!mvpCurrentMap[i]->isQualified()) continue;
			for (int j = 0; j < 3; j++) {
				mCurrentPoints[j][mCurrentPointsNum] = mp.at<float>(j);
			}
			mCurrentPointsNum++;
		}
		mLastPointsNum = 0;
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
			if (mMatches12[i]) {
				for (int j = 0; j < 3; j++) {
					mLastPoints[j][mLastPointsNum] = mp.at<float>(j);
				}
				mLastPointsNum++;
			}
		}
		std::cout << "ICP Point num : " << mvpCurrentMap.size() << " " << mCurrentPointsNum << " " << mLastPointsNum << std::endl;
		vertices_source.resize(Eigen::NoChange, mLastPointsNum);
		vertices_target.resize(Eigen::NoChange, mCurrentPointsNum);
		for (int i = 0; i < mCurrentPointsNum; i++) {
			for (int j = 0; j < 3; j++) {
				vertices_target(j, i) = mCurrentPoints[j][i];
			}
		}
		for (int i = 0; i < mLastPointsNum; i++) {
			for (int j = 0; j < 3; j++) {
				vertices_source(j, i) = mLastPoints[j][i];
			}
		}
		auto tic = std::chrono::steady_clock::now();
		SICP::Parameters pars;
		pars.p = .5;
		pars.max_icp = 15;
		pars.print_icpn = true;
		//std::cout << mLastPoints[0][0] << " " << mLastPoints[1][0] << " " << mLastPoints[2][0] << std::endl;
		SICP::point_to_point(vertices_source, vertices_target, pars);
		/**
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat now(3, 1, CV_32F);
			for (int j = 0; j < 3; j++) now.at<float>(j) = vertices_source(j, i);
			mvpLastMap[i]->SetWorldPos(now);
		}
		*/
		//std::cout << vertices_source(0, 0) << " " << vertices_source(1, 0) << " " << vertices_source(2, 0) << std::endl;
		auto toc = std::chrono::steady_clock::now();

		double time_ms = std::chrono::duration <double, std::milli> (toc-tic).count();
		std::cout << "sparseicp registered source to target in: " << time_ms << "ms" << std::endl;

	}

	void Registrating::SetNew() { 
		for (int i = 0; i < mLastPointsNum; i++) {
			cv::Mat now = mvpLastMap[i]->GetWorldPos();
			for (int j = 0; j < 3; j++) {
				mLastPoints[j][i] = now.at<float>(j);
			}
		}
		
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat now(3, 1, CV_32F);
			for (int j = 0; j < 3; j++) now.at<float>(j) = vertices_source(j, i);
			mvpLastMap[i]->SetWorldPos(now);
		}

		for (int i = 0; i < mLastPointsNum; i++) {
			for (int j = 0; j < 3; j++)
				vertices_source(j, i) = mLastPoints[j][i];
		}
	}

	int Registrating::GetLastMapID(const MapPoint *mp) {
		auto fit = lower_bound(mvpLastMap.begin(), mvpLastMap.end(), mp);
		if (fit == mvpLastMap.end()) return -1;
		return fit - mvpLastMap.begin();
	}

	void Registrating::PushMatch(MapPoint *lastMp, MapPoint *curMp) {
		int id = GetLastMapID(lastMp);
		if (id != -1) {
			if (mMatches12[id] == NULL) {
				mMatches12[id] = curMp;
			} else {
			//	std::cout << "_____" << std::endl;
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
