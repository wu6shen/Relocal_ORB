#include "Registrating.h"
#include <pcl/registration/icp.h>

namespace ORB_SLAM2 {
	Registrating::Registrating(int enoughTh) : mStop(false), mSetMap(false), mEnoughTh(enoughTh) {
		cloud1 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		cloud2 = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		result = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
		return ;
	}
	void Registrating::SetLastMap(Map *lastMap) {
		mvpLastMap = lastMap->GetAllMapPoints();
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
			cloud1->push_back(pcl::PointXYZ(mp.at<float>(0), mp.at<float>(1), mp.at<float>(2)));
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
				cloud2->push_back(pcl::PointXYZ(mp.at<float>(0), mp.at<float>(1), mp.at<float>(2)));
			}
		}
	}

	bool Registrating::CheckSetCurrentMap() {
		std::unique_lock<std::mutex> lock(mMutexSetMap);
		if (mSetMap) {
			cloud2->clear();
			std::random_shuffle(mvpLastMap.begin(), mvpLastMap.end());
			for (size_t i = 0; i < mvpLastMap.size(); i++) {
				cv::Mat mp = mvpLastMap[i]->GetWorldPos();
				cloud2->push_back(pcl::PointXYZ(mp.at<float>(0), mp.at<float>(1), mp.at<float>(2)));
			}
			mSetMap = false;
			return true;
		}
		return false;
	}

	void Registrating::ICP() {
		std::cout << "----" << std::endl;
		pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
		icp.setInputCloud(cloud1);
		icp.setInputTarget(cloud2);


		icp.align(*result);
		std::cout << "ICP information" << icp.hasConverged() << "score" <<
			icp.getFitnessScore() << std::endl;
		std::cout << icp.getFinalTransformation() << std::endl;
	}
}
