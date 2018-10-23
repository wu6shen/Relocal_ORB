#include "Registrating.h"
#include "MySparseICP.h"
#include "Converter.h"
#include <Eigen/Dense>
#include <super4pcs/algorithms/4pcs.h>
#include <super4pcs/utils/geometry.h>
#include <super4pcs/io/io.h>
#include <super4pcs/algorithms/super4pcs.h>

using namespace GlobalRegistration;

namespace ORB_SLAM2 {
struct TransformVisitor {
    inline void operator()(
            float fraction,
            float best_LCP,
            Eigen::Ref<Match4PCSBase::MatrixType> /*transformation*/) const {
      if (fraction >= 0)
        {
          printf("done: %d%c best: %f                  \r",
               static_cast<int>(fraction * 100), '%', best_LCP);
          fflush(stdout);
        }
    }
    constexpr bool needsGlobalTransformation() const { return false; }
};

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

	void Registrating::RunSuper4PCS() {
		std::vector<Point3D> set1, set2;

		float angle = acos(-1) / 4;
		Eigen::Quaternionf rotation (cos (angle / 2.f), 0.3, 0.3, sin (angle / 2.f));

		Eigen::Matrix3f Rtest = rotation.toRotationMatrix();
		Eigen::Vector3f ttest(1, 2, 3);

		mvpCurrentMap = mpMap->GetAllMapPoints();
		for (size_t i = 0; i < mvpCurrentMap.size(); i++) {
			if (!mvpCurrentMap[i]->isQualified()) continue;
			cv::Mat mp = mvpCurrentMap[i]->GetWorldPos();
			Eigen::Vector3f Emp(mp.at<float>(0), mp.at<float>(1), mp.at<float>(2));
			//Emp = R * Emp + t;
			set1.emplace_back(Emp(0), Emp(1), Emp(2));
		}
		std::cout << set1.size() << std::endl;

		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
			set2.emplace_back(mp.at<float>(0), mp.at<float>(1), mp.at<float>(2));
		}

		Point3D::Scalar score = 0;

		Match4PCSOptions options;
		bool isok = options.configureOverlap(0.2);
		if (!isok) {
			std::cout << "Invalid overlap configuration. "<< std::endl;
			return ;
		}
		options.sample_size = 200;
		options.max_normal_difference = -1;
		options.max_color_distance = -1;
		options.max_time_seconds = 10;
		options.delta = 0.03;
		Match4PCSBase::MatrixType mat(Match4PCSBase::MatrixType::Identity());

		constexpr Utils::LogLevel loglvl = Utils::Verbose;
		TransformVisitor visitor;
		Utils::Logger logger(loglvl);

		GlobalRegistration::Sampling::UniformDistSampler sampler;
		
		MatchSuper4PCS matcher(options, logger);
		//logger.Log<Utils::Verbose>("Use Super4PCS");
		score = matcher.ComputeTransformation(set1, &set2, mat, sampler, visitor);
		cv::Mat R(3, 3, CV_32F), t(3, 1, CV_32F);
		for (int i = 0; i < 3; i++) {
			for (int j =0 ; j < 3; j++) R.at<float>(i, j) = mat(i, j);
			t.at<float>(i) = mat(i, 3);
		}

		set1 = matcher.getFirstSampled();
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat now = mvpLastMap[i]->GetWorldPos();
			now = R * now + t;
			mvpLastMap[i]->SetWorldPos(now);
		}
		std::cout << mat << std::endl;
	}

	void Registrating::ICP() {
		vector<pair<int, int> > matches12;
		std::map<MapPoint*, int> ptId;
		mCurrentPointsNum = 0;
		mvpCurrentMap = mpMap->GetAllMapPoints();
		for (size_t i = 0; i < mvpCurrentMap.size(); i++) {
			cv::Mat mp = mvpCurrentMap[i]->GetWorldPos();
			if (!mvpCurrentMap[i]->isQualified()) continue;
			ptId[mvpCurrentMap[i]] = mCurrentPointsNum;
			for (int j = 0; j < 3; j++) {
				mCurrentPoints[j][mCurrentPointsNum] = mp.at<float>(j);
			}
			mCurrentPointsNum++;
		}
		mLastPointsNum = 0;
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat mp = mvpLastMap[i]->GetWorldPos();
				for (int j = 0; j < 3; j++) {
					mLastPoints[j][mLastPointsNum] = mp.at<float>(j);
				}
				mLastPointsNum++;
			if (mMatches12[i]) {
				if (ptId.count(mMatches12[i])) {
					int id = ptId[mMatches12[i]];
					matches12.push_back(make_pair(i, id));
				}
			}
		}
		std::cout << "ICP Point num : " << mvpCurrentMap.size() << " " << mCurrentPointsNum << " " << mLastPointsNum << std::endl;
		vertices_source.resize(Eigen::NoChange, mLastPointsNum);
		vertices_target.resize(Eigen::NoChange, mCurrentPointsNum);

		Eigen::Matrix<double, 3, Eigen::Dynamic> vertices1;
		Eigen::Matrix<double, 3, Eigen::Dynamic> vertices2;
		vertices1.resize(Eigen::NoChange, matches12.size());
		vertices2.resize(Eigen::NoChange, matches12.size());
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
		for (size_t i = 0; i < matches12.size(); i++) {
			int lid = matches12[i].first;
			int rid = matches12[i].second;
			for (int j = 0; j < 3; j++) {
				vertices1(j, i) = mLastPoints[j][lid];
			}
			for (int j = 0; j < 3; j++) {
				vertices2(j, i) = mCurrentPoints[j][rid];
			}
		}
		auto tic = std::chrono::steady_clock::now();
		Eigen::Affine3d trans;
		//std::cout << mLastPoints[0][0] << " " << mLastPoints[1][0] << " " << mLastPoints[2][0] << std::endl;

		trans = RigidMotionEstimator::point_to_point_scale(vertices1, vertices2, Eigen::VectorXd::Ones(vertices1.cols()));
		
        nanoflann::KDTreeAdaptor<Eigen::Matrix<double, 3, Eigen::Dynamic>, 3, nanoflann::metric_L2_Simple> kdtree(vertices_target);
		vertices_source = trans * vertices_source;
		for (size_t i = 0; i < matches12.size(); i++) {
			int lid = matches12[i].first;
			int rid = matches12[i].second;
			int id[2];
		    double dist[2];
			kdtree.query(vertices_source.col(lid).data(), 2, id, dist);
			std::cout << (int)(rid == id[0]) << " ";
			//std::cout << id[0] << " " << id[1] << std::endl;
			//std::cout << dist[0] << " " << dist[1] << std::endl;
		}
			puts("");
		/**
		*/
		for (size_t i = 0; i < mvpLastMap.size(); i++) {
			cv::Mat now(3, 1, CV_32F);
			Eigen::Vector3d p = vertices_source.col(i);
			//p = trans * p;
			for (int j = 0; j < 3; j++) now.at<float>(j) = p(j);
			mvpLastMap[i]->SetWorldPos(now);
		}
		//std::cout << vertices_source(0, 0) << " " << vertices_source(1, 0) << " " << vertices_source(2, 0) << std::endl;
		auto toc = std::chrono::steady_clock::now();

		double time_ms = std::chrono::duration <double, std::milli> (toc-tic).count();
		std::cout << "sparseicp registered source to target in: " << time_ms << "ms" << std::endl;

	}

	void Registrating::SetNew() { 
		/**
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
		*/
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
