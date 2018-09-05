//
// Created by wu6shen on 18-8-12.
//

#include <pcl/common/distances.h>
#include <vector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core.hpp>
#include <opencv/cv.hpp>
#include "Registration.h"

namespace pcl {
    namespace FPCS {
        int FPCSRegistration::Align(PointCloud<PointXYZ>::Ptr &output) {
            srand(time(0));
            double max_dist = 0;
            for (size_t i = 0; i < input_source_->size(); i++) {
                for (size_t j = i + 1; j < input_source_->size(); j++) {
                    double dist = euclideanDistance(input_source_->points[i], input_source_->points[j]);
                    if (max_dist < dist) max_dist = dist;
                }
            }

            int max_inlier_num = 0;
            double error = 0;

            for (int i = 0; i < iterations_; i++) {
                std::cout << "Registration stopped. Iteration #" << i << std::endl;
                target_candidate_basis_ = PointCloud<Set4>::Ptr(new PointCloud<Set4>);
                ratio_ = 1;
                bool next = false;
                while (!registration(max_dist * overlap_ * ratio_)) {
                    if (ratio_ < .25) {
                        ratio_ = 1;
                        next = true;
                        break;
                    }
                    ratio_ /= 1.2;
                }

                if (next) continue;

                Eigen::Vector3f c_source(0, 0, 0), c_target(0, 0, 0);

                for (size_t id = 0; id < target_candidate_basis_->size(); id++) {
                    copy_source_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
                    *copy_source_ = *input_source_;
                    copy_target_ = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
                    *copy_target_ = *input_target_;

                    //c_source = centering(copy_source_);
                    //c_target = centering(copy_target_);

                    int inlier_num = 0, inlier_numb = 0;
                    double err= 0;

                    //PointCloud<PointXYZ>::Ptr inlier(new PointCloud<PointXYZ>);
                    float current_scale;
                    std::vector<std::pair<int, int>> pairs;
                    Eigen::Matrix3f current_rotation;
                    Eigen::Vector3f current_offset(0, 0, 0);
                    estimateSim(id, current_scale, current_rotation, current_offset);

                    transformPointCloud(*copy_source_, *copy_source_, Eigen::Vector3f(0, 0, 0), Eigen::Quaternionf(current_rotation));

                    for (size_t p = 0; p < copy_source_->size(); p++) {
                        for (int j = 0; j < 3; j++) {
                            copy_source_->points[p].data[j] = current_scale * copy_source_->points[p].data[j] + current_offset(j);
                        }
                    }

                    pcl::KdTreeFLANN<PointXYZ> kd_tree;
                    kd_tree.setInputCloud(copy_target_);
                    int vis[3000];
                    memset(vis, 0, sizeof(vis));

                    for (size_t p = 0; p < copy_source_->size(); p++) {
                        std::vector<int> inds;
                        std::vector<float> dists;
                        kd_tree.radiusSearch(copy_source_->points[p], DELTA, inds, dists);
                        if (inds.size() > 0) {
                            if (inds.size() == 1 || dists[0] < dists[1] / 1.2) {
                                if (vis[inds[0]]) continue;
                                vis[inds[0]] = 1;
                                inlier_num++;
                                //pairs.push_back(std::make_pair(p, inds[0]));
                                err += dists[0];
                            }
                        } else {
                            kd_tree.radiusSearch(copy_source_->points[p], 5 * DELTA, inds, dists);
                            /**
                            if (current_scale < 0.5) {
                                for (auto id : inds) {
                                    std::cout << id << " ";
                                }
                                std::cout << std::endl;
                            }
                             */
                            if (inds.size() > 0 && vis[inds[0]] == 0) inlier_numb++, vis[inds[0]] = 1;
                        }
                        /**
                        for (size_t q = 0; q < copy_target_->size(); q++) {
                            double dist = euclideanDistance(copy_source_->points[p], copy_target_->points[q]);
                            if (dist <= DELTA * std::sqrt(current_scale)) {
                                pairs.push_back(std::make_pair(p, q));
                                err += dist;
                                inlier_num++;
                                break;
                            }
                            if (dist <= DELTA * 2 * std::sqrt(current_scale)) {
                                inlier_numb++;
                            }
                        }
                         */
                    }
                    /**
                    */

                    if (inlier_num + inlier_numb / 10 > max_inlier_num) {
                        max_inlier_num = inlier_num + inlier_numb / 10;
                        std::cout << "Now LinerNum: " << inlier_num << " " << inlier_numb << std::endl;

                        for (size_t j = 0; j < pairs.size(); j++) {
                            std::cout << pairs[j].first << " " << pairs[j].second << ",";
                        }
                        puts("");
                        error = err;

                        rotation_ = current_rotation;
                        offset_ = current_offset;
                        scale_ = current_scale;
                        std::cout << offset_ << std::endl;
                        std::cout << scale_ << std::endl;
                        if (max_inlier_num > 50) break;
                    }
                }
            }

            transform_ = getTransform();
            transformPointCloud(*input_source_, *output, transform_);

            return 1;
        }

        int FPCSRegistration::findBasis(const double dist) {
            std::cout << "Basis..." << std::endl;
            Set4 basis(-1, -1, -1, -1);
            basis_ = Set4(-1, -1, -1, -1);
            //PointCloud<PointXYZ>::Ptr basis = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
            bool go = false;

            int random_index = rand() % input_source_->size();
            basis.p1_index_ = random_index;
            //basis->push_back(input_source_->points[random_index]);
            //std::cout << random_index << " ";

            KdTreeFLANN<PointXYZ> kdtree;
            kdtree.setInputCloud(input_source_);

            std::vector<int> indexs;
            std::vector<float> dists;

            kdtree.radiusSearch(input_source_->points[basis[0]], dist * MAX_DELTA, indexs, dists);

            /** search second point*/
            for (size_t i = 0; i < indexs.size(); i++) {
                double current_dist = euclideanDistance(input_source_->points[basis[0]], input_source_->points[indexs[i]]);
                if (current_dist > dist * MIN_DELTA / 2) {
                    basis.p2_index_ = indexs[i];
                    //basis->push_back(input_source_->points[indexs[i]]);
                    //std::cout << indexs[i] << " ";
                    go = true;
                    break;
                }
            }

            if (!go) {
                //std::cout << std::endl;
                return 0;
            }

            /** search third point */
            go = false;

            for (size_t i = 0; i < indexs.size(); i++) {
                double current_dist1 = euclideanDistance(input_source_->points[basis[1]], input_source_->points[indexs[i]]);
                double current_dist0 = euclideanDistance(input_source_->points[basis[0]], input_source_->points[indexs[i]]);
                if (current_dist1 > dist * MIN_DELTA / 2 && current_dist0 > dist * MIN_DELTA / 2 && current_dist1 > 1e-3) {
                    //std::cout << indexs[i] << " ";
                    basis.p3_index_ = indexs[i];
                    //basis->push_back(input_source_->points[indexs[i]]);
                    go = true;
                    break;
                }
            }

            if (!go) {
                //std::cout << std::endl;
                return 0;
            }

            /** search forth point */
            go = false;

            float max_dist = 0;

            for (size_t i = 0; i < input_source_->size(); i++) {
                double dist0 = euclideanDistance(input_source_->points[basis[0]], input_source_->points[i]);
                double dist1 = euclideanDistance(input_source_->points[basis[1]], input_source_->points[i]);
                double dist2 = euclideanDistance(input_source_->points[basis[2]], input_source_->points[i]);

                if (dist0 < 1e-6 || dist0 < dist * MIN_DELTA / 2 ||
                    dist1 < 1e-6 || dist1 < dist * MIN_DELTA / 2 ||
                    dist2 < 1e-6 || dist2 < dist * MIN_DELTA / 2 ) {
                    continue;
                }

                if (isPlane(basis, input_source_->points[i])) {
                    for (int j = 0; j < 3; j++) {
                        float alpha, beta;

                        int index = j;
                        float nx = input_source_->points[i].x - input_source_->points[basis[index]].x;
                        float ny = input_source_->points[i].y - input_source_->points[basis[index]].y;

                        index = (j + 1) % 3;
                        float px = input_source_->points[i].x - input_source_->points[basis[index]].x;
                        float py = input_source_->points[i].y - input_source_->points[basis[index]].y;

                        index = (j + 2) % 3;
                        float qx = input_source_->points[i].x - input_source_->points[basis[index]].x;
                        float qy = input_source_->points[i].y - input_source_->points[basis[index]].y;

                        alpha = (ny * qx - nx * qy) / (py * qx - px * qy);
                        beta = (ny * px - nx * py) / (qy * px - qx * py);
                        float now_dist = dist0 + dist1 + dist2;
                        if (alpha > 1e-6 && beta > 1e-6 && now_dist > max_dist) {
                            max_dist = now_dist;
                            basis_.p1_index_ = basis[j];
                            basis_.p2_index_ = i;
                            basis_.p3_index_ = basis[(j + 1) % 3];
                            basis_.p4_index_ = basis[(j + 2) % 3];

                        }
                    }
                    go = true;
                }
            }

            if (!go) {
                //std::cout << std::endl;
                return 0;
            }

            std::cout << "find basis!" << std::endl;

            return 1;
        }

        PointXYZ FPCSRegistration::getCross(const pcl::PointXYZ &p1, const pcl::PointXYZ &p2, const pcl::PointXYZ &p3,
                                            const pcl::PointXYZ &p4) {
            if (fabs((p4.x-p3.x)*(p2.y-p1.y)-(p2.x-p1.x)*(p4.y-p3.y)) < 1e-4) return PointXYZ(-1e9, -1e9, -1e9);
            float e_x = ((p3.y-p1.y)*(p2.x-p1.x)*(p4.x-p3.x) +
                         p1.x*(p4.x-p3.x)*(p2.y-p1.y) -
                         p3.x*(p2.x-p1.x)*(p4.y-p3.y)) /
                        ((p4.x-p3.x)*(p2.y-p1.y) -
                         (p2.x-p1.x)*(p4.y-p3.y)),
                    e_y = (e_x - p1.x) * (p2.y-p1.y) / (p2.x-p1.x) + p1.y,
                    e_z = (e_x - p1.x) * (p2.z-p1.z) / (p2.x-p1.x) + p1.z;
            return PointXYZ(e_x, e_y, e_z);
        }

        bool FPCSRegistration::check(pcl::FPCS::FPCSRegistration::Set4 ss, float delta) {
            PointXYZ e = getCross(input_source_->points[basis_.p1_index_], input_source_->points[basis_.p2_index_], input_source_->points[basis_.p3_index_], input_source_->points[basis_.p4_index_]);
            PointXYZ es = getCross(input_target_->points[ss.p1_index_], input_target_->points[ss.p2_index_], input_target_->points[ss.p3_index_], input_target_->points[ss.p4_index_]);

            Eigen::Vector3f pl[4], pr[4];
            for (int i = 0; i < 4; i++) {
                for (int j =0 ; j < 3; j++) {
                    pl[i](j) = input_source_->points[basis_[i]].data[j];
                    pr[i](j) = input_target_->points[ss[i]].data[j];
                }
            }

            float dotl = (pl[3] - pl[2]).dot(pl[1] - pl[0]) / (pl[3] - pl[2]).norm() / (pl[1] - pl[0]).norm();
            float dotr = (pr[3] - pr[2]).dot(pr[1] - pr[0]) / (pr[3] - pr[2]).norm() / (pr[1] - pr[0]).norm();
            float e0 = fabs(dotl - dotr);
            if (e0 > delta) return false;

            float d1 = euclideanDistance(input_source_->points[basis_.p1_index_], input_source_->points[basis_.p2_index_]);
            float d2 = euclideanDistance(input_source_->points[basis_.p3_index_], input_source_->points[basis_.p4_index_]);
            float d1s = euclideanDistance(input_target_->points[ss.p1_index_], input_target_->points[ss.p2_index_]);
            float d2s = euclideanDistance(input_target_->points[ss.p3_index_], input_target_->points[ss.p4_index_]);
            float e1 = fabs(d1 - d1s);
            float e2 = fabs(d2 - d2s);
            float e5 = fabs(d1 / d1s - d2 / d2s);


            float r1 = pcl::euclideanDistance(input_source_->points[basis_.p1_index_], e) / d1;
            float r2 = pcl::euclideanDistance(input_source_->points[basis_.p3_index_], e) / d2;
            float r1s = pcl::euclideanDistance(input_target_->points[ss.p1_index_], es) / d1s;
            float r2s = pcl::euclideanDistance(input_target_->points[ss.p3_index_], es) / d2s;
            float e3 = fabs(r1 - r1s);
            float e4 = fabs(r2 - r2s);
            /**
            std::cout << "cross point:" << e.x << " " << e.y << " " << e.z << std::endl;
            std::cout << "cross point:" << es.x << " " << es.y << " " << es.z << std::endl;
            std::cout << dotl << " dot " << dotr << std::endl;
            std::cout << d1 << " dis " << d2 << std::endl;
            std::cout << d1s << " dis " << d2s << std::endl;
            std::cout << r1 << " radio " << r2 << std::endl;
            std::cout << r1s << " radio " << r2s << std::endl;
             */
            //return e0 < DELTA && e1 < DELTA && e2 < DELTA && e3 < DELTA && e4 < DELTA;
            return e0 < delta && e3 < delta && e4 < delta && e5 < delta;
        }

        int FPCSRegistration::findTargetCandidateBasis() {

            PointXYZ e = getCross(input_source_->points[basis_[0]], input_source_->points[basis_[1]], input_source_->points[basis_[2]], input_source_->points[basis_[3]]);
            if (e.x == -1e9 && e.y == -1e9 && e.z == -1e9) return 0;

            float d1 = euclideanDistance(input_source_->points[basis_.p1_index_], input_source_->points[basis_.p2_index_]);
            float d2 = euclideanDistance(input_source_->points[basis_.p3_index_], input_source_->points[basis_.p4_index_]);

            if (d1 <= 1e-5 || d2 <= 1e-5) return 0;

            float r1 = pcl::euclideanDistance(input_source_->points[basis_.p1_index_], e) / d1;
            float r2 = pcl::euclideanDistance(input_source_->points[basis_.p3_index_], e) / d2;

            if (r1 > 1 || r2 > 1) return 0;

            std::vector<std::pair<int,int> > pairs1[100], pairs2[100];
            PointCloud<PointXYZ>::Ptr e1_set[100];
            PointCloud<PointXYZ>::Ptr e2_set[100];
            for (int i = 0; i < 100; i++) {
                e1_set[i] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
                e2_set[i] = PointCloud<PointXYZ>::Ptr(new PointCloud<PointXYZ>);
            }


            for (size_t i1 = 0; i1 < input_target_->size(); ++i1)
            {
                for (size_t i2 = 0; i2 < input_target_->size(); ++i2)
                {
                    std::pair<int,int> p;
                    float dist = pcl::euclideanDistance(input_target_->points[i1], input_target_->points[i2]);
                    p.first = i1;
                    p.second = i2;
                    float scale1 = dist / d1;
                    float scale2 = dist / d2;
                    int index1 = scale1 * 10;
                    int index2 = scale2 * 10;
                    //if(dist >= d1 * MIN_DELTA && dist <= d1 * MAX_DELTA)
                    //{
                    if (index1 >= 1 && index1 < 100) {
                        pairs1[index1].push_back(p);
                        float e1x = input_target_->points[i1].x +
                                    r1 * (input_target_->points[i2].x - input_target_->points[i1].x);
                        float e1y = input_target_->points[i1].y +
                                    r1 * (input_target_->points[i2].y - input_target_->points[i1].y);
                        float e1z = input_target_->points[i1].z +
                                    r1 * (input_target_->points[i2].z - input_target_->points[i1].z);
                        e1_set[index1]->push_back(PointXYZ(e1x, e1y, e1z));
                    }
                    //}
                    //if(dist >= d2 * MIN_DELTA && dist <= d2 * MAX_DELTA)
                    //{
                    if (index2 >= 1 && index2 < 100) {
                        float e2x = input_target_->points[i1].x +
                                    r2 * (input_target_->points[i2].x - input_target_->points[i1].x);
                        float e2y = input_target_->points[i1].y +
                                    r2 * (input_target_->points[i2].y - input_target_->points[i1].y);
                        float e2z = input_target_->points[i1].z +
                                    r2 * (input_target_->points[i2].z - input_target_->points[i1].z);
                        pairs2[index2].push_back(p);
                        e2_set[index2]->push_back(PointXYZ(e2x, e2y, e2z));
                    }
                    //}
                    /**
                    if (pairs1.size() > input_target_->size() || pairs2.size() > input_target_->size())
                    {
                        i1 = input_target_->size() - 2;
                        break;
                    }
                     */
                }
            }

            //if (pairs1.size() < 1 || pairs2.size() < 1) return 0;

            for (int si = 5; si < 20; si++) {
                if (pairs1[si].size() < 1 || pairs2[si].size() < 1) continue;
                std::cout << e1_set[si]->size() << std::endl;
                pcl::KdTreeFLANN<PointXYZ> kd_tree;
                kd_tree.setInputCloud(e1_set[si]);

                for (size_t i = 0; i < e2_set[si]->size(); ++i) {
                    std::vector<int> inds;
                    std::vector<float> dists;
                    PointXYZ e2 = e2_set[si]->points[i];
                    kd_tree.radiusSearch(e2, std::min(DELTA, DELTA * si * 0.1), inds, dists);

                    for (size_t j = 0; j < inds.size(); ++j) {
                        Set4 Ui(-27, -27, -27, -27);
                        int indexx = inds[j];
                        if (indexx != -1) {
                            Ui.p1_index_ = pairs1[si][indexx].first;
                            Ui.p2_index_ = pairs1[si][indexx].second;
                            indexx = i;
                            if (indexx != -1) {
                                Ui.p3_index_ = pairs2[si][indexx].first;
                                Ui.p4_index_ = pairs2[si][indexx].second;
                                if (check(Ui, std::min(DELTA, DELTA * si * 0.1))) {
                                    /**
                                    {
                                        for (int p = 0; p < 4; p++) std::cout << Ui[p] << " ";
                                        puts("");
                                    }
                                     */
                                    target_candidate_basis_->push_back(Ui);
                                }
                            }
                        }
                    }
                }
            }
            std::cout << target_candidate_basis_->size() << std::endl;


            if (target_candidate_basis_->size() < 1) return 0;

            //std::cout << "Target basis num : " << target_candidate_basis_->size() << std::endl;
            return target_candidate_basis_->size();
        }

        int FPCSRegistration::registration(const double dist) {
            if (!findBasis(dist)) {
                //std::cout << "cannot find basis! " << std::endl;
                return 0;
            }

            time_t st = clock();
            if (!findTargetCandidateBasis()) {
                //std::cout << "Cannot find corresponding basis in target!" << std::endl;
            }

            time_t ed = clock();
            std::cout << 1. * (ed - st) / CLOCKS_PER_SEC << std::endl;
            return 1;
        }

        Eigen::Vector3f FPCSRegistration::centering(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
            PointXYZ p(0,0,0);
            int number_of_points = point_cloud->size();
            for (int i = 0; i < number_of_points; ++i)
            {
                p.x += point_cloud->points[i].x;
                p.y += point_cloud->points[i].y;
                p.z += point_cloud->points[i].z;
            }
            p.x /= number_of_points;
            p.y /= number_of_points;
            p.z /= number_of_points;

            for (int i = 0; i < number_of_points; ++i)
            {
                point_cloud->points[i].x -= p.x;
                point_cloud->points[i].y -= p.y;
                point_cloud->points[i].z -= p.z;
            }
            Eigen::Vector3f res(p.x, p.y, p.z);

            return res;
        }



        Eigen::Quaternionf FPCSRegistration::estimateRotation(const int corr_index) {
            PointCloud<PointXYZ>::Ptr target_basis(new PointCloud<PointXYZ>);
            target_basis->push_back(copy_target_->points[target_candidate_basis_->points[corr_index][0]]);
            target_basis->push_back(copy_target_->points[target_candidate_basis_->points[corr_index][1]]);
            target_basis->push_back(copy_target_->points[target_candidate_basis_->points[corr_index][2]]);
            target_basis->push_back(copy_target_->points[target_candidate_basis_->points[corr_index][3]]);

            PointCloud<PointXYZ>::Ptr source_basis(new PointCloud<PointXYZ>);
            source_basis->push_back(copy_source_->points[basis_[0]]);
            source_basis->push_back(copy_source_->points[basis_[1]]);
            source_basis->push_back(copy_source_->points[basis_[2]]);
            source_basis->push_back(copy_source_->points[basis_[3]]);
            PointCloud<PointXYZ>::Ptr left1(new PointCloud<PointXYZ>);
            PointCloud<PointXYZ>::Ptr right1(new PointCloud<PointXYZ>);
            *left1 = *source_basis;
            *right1 = *target_basis;

            /** print point
            for (int i = 0; i < 4; i++) {
                std::cout << left1->points[i].x << " " << left1->points[i].y << " " << left1->points[i].z << std::endl;
            }

            for (int i = 0; i < 4; i++) {
                std::cout << right1->points[i].x << " " << right1->points[i].y << " " << right1->points[i].z << std::endl;
            }
             */

            //1.planes rotation

            //left surface normal
            float x1 = left1->points[0].x, y1 = left1->points[0].y, z1 = left1->points[0].z,
                    x2 = left1->points[1].x, y2 = left1->points[1].y, z2 = left1->points[1].z,
                    x3 = left1->points[2].x, y3 = left1->points[2].y, z3 = left1->points[2].z;

            Eigen::Vector3f n1((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));

            Eigen::Vector3f l1(x1, y1, z1), l2(x2, y2, z2), l3(x3,y3,z3), nl = (l2 - l1).cross(l3 - l2);

            //right surface normal
            x1 = right1->points[0].x; y1 = right1->points[0].y; z1 = right1->points[0].z;
            x2 = right1->points[1].x; y2 = right1->points[1].y; z2 = right1->points[1].z;
            x3 = right1->points[2].x; y3 = right1->points[2].y; z3 = right1->points[2].z;

            Eigen::Vector3f n2((y2-y1)*(z3-z1) - (y3-y1)*(z2-z1), (x2-x1)*(z3-z1) - (x3-x1)*(z2-z1), (x2-x1)*(y3-y1)-(x3-x1)*(y2-y1));

            Eigen::Vector3f r1(x1, y1, z1), r2(x2, y2, z2), r3(x3, y3, z3), nr = (r2 - r1).cross(r3 - r2);

            /**
            {
                std::cout << rotation_gt_ * l1 - r1 << std::endl;
                std::cout << rotation_gt_ * l2 - r2 << std::endl;
                std::cout << rotation_gt_ * l3 - r3 << std::endl;
            }
             */

            n1 = nl;
            n2 = nr;

            if (n1.isZero() || n2.isZero())
            {
                //left1->~PointCloud();
                //right1->~PointCloud();
                Eigen::Quaternionf q(0,0,0,0);
                //std::cout << "bad normals..." << std::endl;
                return q;
            }


            //crossing line for 2 planes (a) and sin & cos for rotation
            Eigen::Vector3f a = n1.cross(n2);
            if (a.norm() > 1e-6) a /= a.norm();

            n1 /= n1.norm();
            n2 /= n2.norm();

            float sin, cos = n1.dot(n2);
            sin = n1.cross(n2).norm();
            //std::cout << cos << " " << sin << std::endl;

            ///need cos alpha/2 and sin alpha/2 if cos alpha = n1 dot n2 and sin alpha = n1 cross n2
            cos = sqrt((cos + 1)/2);
            sin = cos == 0 ? 1 : sin/(2*cos);


            Eigen::Quaternionf qa(cos, sin*a.x(), sin*a.y(), sin*a.z());

            //applying plane rotation
            std::vector<Eigen::Vector3f> ll;
            for (size_t i = 0; i < left1->size(); ++i)
            {
                float x1 = left1->points[i].x, y1 = left1->points[i].y, z1 = left1->points[i].z;
                Eigen::Vector3f l1(x1, y1, z1);

                ll.push_back(qa.matrix() * l1);
                left1->points[i].x = ll[i].x();
                left1->points[i].y = ll[i].y();
                left1->points[i].z = ll[i].z();
            }

            //2.rotation in plane|points rotation
            float C = (r2 - r1).dot(ll[1] - ll[0]), S = (ll[1] - ll[0]).cross(r2 - r1).dot(n2),
                    osin = S/sqrt(S*S + C*C), ocos = C/sqrt(S*S + C*C);
            osin = ocos == -1 ? 1 : osin/sqrt(2*(1 + ocos));
            ocos = sqrt((ocos + 1)/2);
            //std::cout << ocos << " " << osin << std::endl;

            Eigen::Quaternionf qp(ocos, osin*n2.x(), osin*n2.y(), osin*n2.z());

            //result
            Eigen::Quaternionf qe = qp*qa;

            return qe;
        }

        Eigen::Matrix4f FPCSRegistration::getTransform() {
            Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    transform(i, j) = rotation_(i, j);
                }
                transform(i, 3) = offset_.data()[i];
                transform(3, i) = 0;
            }
            return transform;
        }

        bool FPCSRegistration::isPlane(Set4 basis, const PointXYZ &last_point) {
            Eigen::Matrix3f mat = Eigen::Matrix3f::Zero();
            for (size_t i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    mat(i, j) = input_source_->points[basis[i]].data[j] - last_point.data[j];
                }
            }
            if (fabs(mat.determinant()) < 1e-3) return true;
            return false;
        }

        void FPCSRegistration::estimateSim(const int corr_index, float &scale, Eigen::Matrix3f &rotation, Eigen::Vector3f &offset) {
            cv::Mat P1(3, 3, CV_32F), P2(3, 3, CV_32F);
            cv::Mat pt1(3, 1, CV_32F), pt2(3, 1, CV_32F);
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    pt2.at<float>(j) = input_source_->points[basis_[i]].data[j];
                    pt1.at<float>(j) = input_target_->points[target_candidate_basis_->points[corr_index][i]].data[j];
                }
                pt1.copyTo(P1.col(i));
                pt2.copyTo(P2.col(i));
            }

            cv::Mat Pr1(P1.size(), P1.type());
            cv::Mat Pr2(P1.size(), P2.type());
            cv::Mat O1(3,1,Pr1.type()); // Centroid of P1
            cv::Mat O2(3,1,Pr2.type()); // Centroid of P2

            cv::reduce(P1, O1, 1, CV_REDUCE_SUM);
            cv::reduce(P2, O2, 1, CV_REDUCE_SUM);
            O1 = O1 / P1.cols;
            O2 = O2 / P2.cols;
            for (int i = 0; i < P1.cols; i++) {
                Pr1.col(i) = P1.col(i) - O1;
                Pr2.col(i) = P2.col(i) - O2;
            }

            cv::Mat M = Pr2 * Pr1.t();

            double N11, N12, N13, N14, N22, N23, N24, N33, N34, N44;

            cv::Mat N(4,4,P1.type());

            N11 = M.at<float>(0,0)+M.at<float>(1,1)+M.at<float>(2,2);
            N12 = M.at<float>(1,2)-M.at<float>(2,1);
            N13 = M.at<float>(2,0)-M.at<float>(0,2);
            N14 = M.at<float>(0,1)-M.at<float>(1,0);
            N22 = M.at<float>(0,0)-M.at<float>(1,1)-M.at<float>(2,2);
            N23 = M.at<float>(0,1)+M.at<float>(1,0);
            N24 = M.at<float>(2,0)+M.at<float>(0,2);
            N33 = -M.at<float>(0,0)+M.at<float>(1,1)-M.at<float>(2,2);
            N34 = M.at<float>(1,2)+M.at<float>(2,1);
            N44 = -M.at<float>(0,0)-M.at<float>(1,1)+M.at<float>(2,2);

            N = (cv::Mat_<float>(4,4) << N11, N12, N13, N14,
                    N12, N22, N23, N24,
                    N13, N23, N33, N34,
                    N14, N24, N34, N44);


            // Step 4: Eigenvector of the highest eigenvalue

            cv::Mat eval, evec;

            cv::eigen(N,eval,evec); //evec[0] is the quaternion of the desired rotation

            cv::Mat vec(1,3,evec.type());
            (evec.row(0).colRange(1,4)).copyTo(vec); //extract imaginary part of the quaternion (sin*axis)

            // Rotation angle. sin is the norm of the imaginary part, cos is the real part
            double ang=atan2(norm(vec),evec.at<float>(0,0));

            vec = 2*ang*vec/norm(vec); //Angle-axis representation. quaternion angle is the half


            cv::Mat R12 = cv::Mat(3,3,P1.type());

            cv::Rodrigues(vec, R12); // computes the rotation matrix from angle-axis

            cv::Mat P3 = R12 * Pr2;

            double nom = Pr1.dot(P3);
            cv::Mat aux_P3(P3.size(),P3.type());
            aux_P3=P3;
            cv::pow(P3,2,aux_P3);
            double den = 0;

            for(int i=0; i<aux_P3.rows; i++)
            {
                for(int j=0; j<aux_P3.cols; j++)
                {
                    den+=aux_P3.at<float>(i,j);
                }
            }

            scale = (float)nom/den;
            cv::Mat offset_cv = cv::Mat(1, 3, P1.type());
            offset_cv = O1 - scale * R12 * O2;
            Eigen::Matrix4f T;
            for (int i = 0; i < 3; i++) {
                for (int j = 0; j < 3; j++) {
                    rotation(i, j) = R12.at<float>(i, j);
                }
                offset(i) = offset_cv.at<float>(i);
            }
        }
    }
}
