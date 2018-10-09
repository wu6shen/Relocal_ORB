#include "Registrating.h"

namespace ORB_SLAM2 {
	void Registrating::SetLastMap(Map *lastMap) {
		mvpLastMap = lastMap->GetAllMapPoints();
	}

	void Registrating::SetCurrentMap(Map *currentMap) {
		mvpCurrentMap = currentMap->GetAllMapPoints();
	}

	void Registrating::InsertInCurrentMap(MapPoint *mp) {
		std::unique_lock<std::mutex> lock(mMutexNewMap);
		mvpNewMap.push_back(mp);
	}

	void Registrating::Run() {
		while (1) {
			if (CheckEnoughNewMapPoints()) {
				ICP();
			}
		}
	}

	void Registrating::SetStop() {
		std::unique_lock<std::mutex> lock(mStop);
		mStop = true;
	}

	bool Registrating::IsStop() {
		std::unique_lock<std::mutex> lock(mStop);
		return mStop;
	}

	bool Registrating::CheckNewMapPoints() {
		std::unique_lock<std::mutex> lock(mMutexNewMap);
		return mvpNewMap.size() > mEnoughTh;
	}

	void Registrating::ICP() {
	}
}
