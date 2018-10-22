#ifndef _REGISTRATING_H_
#define _REGISTRATING_H_

#include "MapPoint.h"
#include "Map.h"
#include <mutex>
#include <Eigen/Dense>

namespace ORB_SLAM2 {
	const int MAX_POINT_NUM = 10010;
	class Registrating {
	public:
		Registrating(int enoughTh);
		void SetLastMap(Map *lastMap);
		void SetCurrentMap(Map *currentMap);
		void Run();
		void SetStop();
		bool IsStopped();

		void SetNew();
		void ICP();
		void RunSuper4PCS();

		void PushMatch(MapPoint *lastMp, MapPoint *curMp);
		
	protected:

		bool NeedICP(); 

		int GetLastMapID(const MapPoint *mp);

		Map *mpMap;
		std::vector<MapPoint*> mvpLastMap;
		std::vector<MapPoint*> mvpCurrentMap;
		std::vector<MapPoint*> mvpNewMap;
		std::mutex mMutexNewMap, mMutexStop, mMutexSetMap;
		std::vector<MapPoint*> mMatches12;
		bool mStop, mSetMap;

		float mLastPoints[3][MAX_POINT_NUM];
		int mLastPointsNum;
		float mCurrentPoints[3][MAX_POINT_NUM];
		int mCurrentPointsNum;

		int mEnoughTh;
		cv::Mat R, t;
		float scale;

		Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_source;
		Eigen::Matrix<double, 3, Eigen::Dynamic> vertices_target;
	};

}

#endif
