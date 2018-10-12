#ifndef _REGISTRATING_H_
#define _REGISTRATING_H_

#include "MapPoint.h"
#include "Map.h"
#include <mutex>

namespace ORB_SLAM2 {
	const int MAX_POINT_NUM = 10010;
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

		float mLastPoints[3][MAX_POINT_NUM];
		int mLastPointsNum;
		float mCurrentPoints[3][MAX_POINT_NUM];
		int mCurrentPointsNum;

		bool CheckEnoughNewMapPoints();
		bool CheckSetCurrentMap();
		void ICP();

		int mEnoughTh;
		cv::Mat R, t;
		float scale;
	};

}

#endif
