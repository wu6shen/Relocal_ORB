//
// Created by wu6shen on 18-8-12.
//

#ifndef RELOCAL_REGISTRATION_H
#define RELOCAL_REGISTRATION_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <Eigen/Core>
#include "MapPoint.h"

#define DELTA 0.01
#define MIN_DELTA (1 - DELTA/1.5)
#define MAX_DELTA (1 + DELTA/1.5)
using namespace ORB_SLAM2;


namespace pcl {
    namespace FPCS {
        class FPCSRegistration {
            struct Set4 {
                int p1_index_;
                int p2_index_;
                int p3_index_;
                int p4_index_;

                Set4() {}
                Set4(int p1, int p2, int p3, int p4) : p1_index_(p1), p2_index_(p2), p3_index_(p3), p4_index_(p4) {}

                const int &operator[](int index) {
                    switch (index) {
                        case 0:
                            return p1_index_;
                        case 1:
                            return p2_index_;
                        case 2:
                            return p3_index_;
                        case 3:
                            return p4_index_;
                        default:
                            return -1;

                    }
                }
            };

        public:
            FPCSRegistration() : iterations_(5),
                                 delta_(0.1),
                                 overlap_(0.9),
                                 ratio_(1),
                                 input_source_(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>)),
                                 input_target_(PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>)) {}

            ~FPCSRegistration() {
            }

            void SetNumOfIteration(const int num) { iterations_ = num; }

            void SetDelta(const float delta) { delta_ = delta; }

            float GetDelta() { return delta_; }

            void SetOverlap(const float overlap) { overlap_ = overlap; }

            float GetOverlap() { return overlap_; }

            void SetInputSourceCloud(PointCloud<PointXYZ>::Ptr input) {
                *input_source_ = *input;
            }

            void SetInputTargetCloud(PointCloud<PointXYZ>::Ptr output) {
                *input_target_ = *output;
            }

            void SetRotationGT(const Eigen::Matrix3f &r) { rotation_gt_ = r; }

            void SetOffsetGT(const Eigen::Vector3f &o) { offset_gt_ = o; }

            Eigen::Matrix4f GetTransform() { return transform_; }

            Eigen::Matrix3f GetRotation() { return rotation_; }

            Eigen::Vector3f GetOffset() { return offset_; }

            float GetScale() { return scale_; }

            int Align(PointCloud<PointXYZ>::Ptr &output);

            vector<MapPoint*> last;
            vector<MapPoint*> current;
            Map *last_map;
            Map *new_map;

        private:
            int iterations_;
            double delta_;
            double overlap_;
            double ratio_;

            PointCloud<PointXYZ>::Ptr input_source_;
            PointCloud<PointXYZ>::Ptr input_target_;

            PointCloud<PointXYZ>::Ptr copy_source_;
            PointCloud<PointXYZ>::Ptr copy_target_;

            PointCloud<Set4>::Ptr target_candidate_basis_;
            Set4 basis_;

            Eigen::Matrix4f transform_;
            Eigen::Matrix3f rotation_;
            Eigen::Vector3f offset_;
            float scale_;

            Eigen::Matrix4f transform_gt_;
            Eigen::Matrix3f rotation_gt_;
            Eigen::Vector3f offset_gt_;
            float scale_gt_;

            int findBasis(const double dist);

            int findTargetCandidateBasis();

            int registration(const double dist);

            Eigen::Quaternionf estimateRotation(const int corr_index);

            void estimateSim(const int corr_index, float &scale, Eigen::Matrix3f &rotation, Eigen::Vector3f &offset);

            Eigen::Vector3f centering(PointCloud<PointXYZ>::Ptr point_cloud);

            Eigen::Matrix4f getTransform();

            bool isPlane(Set4 basis, const PointXYZ &last_point);

            PointXYZ getCross(const PointXYZ &p1, const PointXYZ &p2, const PointXYZ &p3, const PointXYZ &p4);

            bool check(Set4 ss, float delta);

        };
    }
}

#endif //RELOCAL_REGISTRATION_H
