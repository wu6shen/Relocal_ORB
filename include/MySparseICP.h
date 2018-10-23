#ifndef _MYSPARSEICP_H_
#define _MYSPARSEICP_H_
#include "Thirdparty/sparseicp/ICP.h"

namespace RigidMotionEstimator {
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point_scale(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   const Eigen::MatrixBase<Derived3>& w) {
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean, Y_mean;
        for(int i=0; i<3; ++i) {
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
        }
        X.colwise() -= X_mean;
        Y.colwise() -= Y_mean;
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
            transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        } else {
            transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
        }
		double scale = 1;
		Eigen::Matrix3Xd XR = transformation.linear() * X;
		double nom = 0;
		for (int i = 0; i < Y.cols(); i++) {
			for (int j = 0; j < 3; j++) {
				nom += XR(j, i) * Y(j, i);
			}
		}
		double den = 0;
		for (int i = 0; i < Y.cols(); i++) {
			for (int j = 0; j < 3; j++) den += std::pow(Y(j, i), 2);
		}
		scale = nom / den;
		std::cout << scale << std::endl;
        transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += Y_mean;
        /// Return transformation
        return transformation;
    }
}

namespace SICP {
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
	/// @param Translation
    /// @param Parameters
	// Z : inlier flag, 零值表示内点， 非零值逐渐收敛到X  
    template <typename Derived1, typename Derived2>
    void point_to_point(const Eigen::MatrixBase<Derived1>& Xt,
                        const Eigen::MatrixBase<Derived2>& Yt,
						Eigen::Affine3d &trans, 
                        Parameters par = Parameters()) {
        Eigen::Matrix3Xd X = Xt;
        Eigen::Matrix3Xd Y = Yt;
		trans = Eigen::Affine3d::Identity();
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
            }
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = X-Q+C/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::Matrix3Xd U = Q+Z-C/mu;
                    trans = trans * RigidMotionEstimator::point_to_point(X, U);
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::Matrix3Xd P = X-Q-Z;
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
			int num = 0;
			for (int i = 0; i < Z.cols(); i++) {
				if (abs(Z(0, i)) < 1e-5 && 
						abs(Z(1, i)) < 1e-5 && 
						abs(Z(2, i)) < 1e-5) {
					num++;
				}
			}
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop) break;
        }
    }
    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
	/// @param Translation
	/// @param Matches
    /// @param Parameters
	// Z : inlier flag, 零值表示内点， 非零值逐渐收敛到X  
    template <typename Derived1, typename Derived2>
    void point_to_point(const Eigen::MatrixBase<Derived1>& Xt,
                        const Eigen::MatrixBase<Derived2>& Yt,
						Eigen::Affine3d &trans, 
						std::vector<int> &matches12,
                        Parameters par = Parameters()) {
        Eigen::Matrix3Xd X = Xt;
        Eigen::Matrix3Xd Y = Yt;
		trans = Eigen::Affine3d::Identity();
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree2(X);
		std::vector<double> eps(X.cols());
		for (int i = 0; i < X.cols(); i++) {
			int id[2];
			double dist[2];
			kdtree2.query(X.col(i).data(), 2, id, dist);
			eps[i] = dist[1];
		}
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(i);
            }
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = X-Q+C/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::Matrix3Xd U = Q+Z-C/mu;
                    trans = trans * RigidMotionEstimator::point_to_point(X, U);
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::Matrix3Xd P = X-Q-Z;
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
			int num = 0, out1 = 0, out2 = 0;
			for (int i = 0; i < Z.cols(); i++) {
				if (Z(0, i) < 1e-5 && Z(1, i) < 1e-5 && Z(2, i) < 1e-5) num++;
			}
			std::cout << num << std::endl;
			num = 0;

			matches12.resize(X.cols());
			for (int i = 0; i < X.cols(); i++) {
				matches12[i] = -1;
				int id[2];
				double dist[2];
				kdtree.query(X.col(i).data(), 2, id, dist);
				if (dist[0] < max(1e-3, eps[i]) && dist[0] * 2 < dist[1]) {
					int id2[2];
					double dist2[2];
					kdtree2.query(Y.col(id[0]).data(), 2, id2, dist2);
					if (id2[0] == i) {
						matches12[i] = id[0];
						num++;
					} else {
					}
				}
			}
			std::cout << num << " " << out1 << " " << out2 << std::endl;
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop) break;
        }
    }
}
#endif
