#ifndef _REGISTRATING_H_
#define _REGISTRATING_H_

#include "MapPoint.h"
#include "Map.h"
#include <mutex>

namespace ORB_SLAM2 {
	class Registrating {
	public:
		Rregistrating();
		void SetLastMap(Map *lastMap);
		void SetCurrentMap(Map *currentMap);
		void InsertInCurrentMap(MapPoint *mp);
		void Run();
		void SetStop();
		bool IsStopped();
		
	protected:
		std::vector<MapPoint*> mvpLastMap;
		std::vector<MapPoint*> mvpCurrrentMap;
		std::vector<MapPoint*> mvpNewMap;
		std::mutex mMutexNewMap, mStop;
		std::vector<std::pair<int, int> > mMatches12;
		bool mStop;

		bool CheckEnoughNewMapPoints();
		void ICP();

		int mEnoughTh;
		cv::Mat R, t;
		float scale;
	}

}

#endif
