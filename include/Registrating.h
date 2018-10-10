#ifndef _REGISTRATING_H_
#define _REGISTRATING_H_

#include "MapPoint.h"
#include "Map.h"
#include <mutex>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace ORB_SLAM2 {
	class Registrating {
	public:
		Registrating(int enoughTh);
		void SetLastMap(Map *lastMap);
		void SetCurrentMap(Map *currentMap);
		void InsertInCurrentMap(MapPoint *mp);
		void Run();
		void SetStop();
		bool IsStopped();
		
	protected:
		std::vector<MapPoint*> mvpLastMap;
		std::vector<MapPoint*> mvpCurrentMap;
		std::vector<MapPoint*> mvpNewMap;
		std::mutex mMutexNewMap, mMutexStop, mMutexSetMap;
		std::vector<std::pair<int, int> > mMatches12;
		bool mStop, mSetMap;

		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2;
		pcl::PointCloud<pcl::PointXYZ>::Ptr result;

		bool CheckEnoughNewMapPoints();
		bool CheckSetCurrentMap();
		void ICP();

		int mEnoughTh;
		cv::Mat R, t;
		float scale;
	};

}

#endif
