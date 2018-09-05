#include <iostream>
#include <cstdio>
#include <string>
#include <pcl/point_types.h>
#include <pcl/registration/ia_fpcs.h>
#include "Registration.h"
#include "System.h"

using namespace pcl;
using namespace pcl::io;
using namespace pcl::registration;

//const std::string image_folder = "/home/wu6shen/Computer-Vision/Data/rgbd_dataset_freiburg1_xyz/";
//const std::string image_list_file = "/home/wu6shen/Computer-Vision/Data/rgbd_dataset_freiburg1_xyz/rgb.txt";
//const std::string setting_file = "/home/wu6shen/Computer-Vision/Project/Relocal-ORB/TUM1.yaml";
const std::string image_folder = "/home/wu6shen/Computer-Vision/Data/kinectv2/Image-2018-08-31-13-13-29/";
const std::string image_list_file = "/home/wu6shen/Computer-Vision/Data/kinectv2/Image-2018-08-31-13-13-29/rgb.txt";
//const std::string image_folder = "/home/wu6shen/Computer-Vision/Data/kinectv2/Image-2018-08-02-15-32-29/";
//const std::string image_list_file = "/home/wu6shen/Computer-Vision/Data/kinectv2/Image-2018-08-02-15-32-29/rgb.txt";
const std::string setting_file = "/home/wu6shen/Computer-Vision/Project/Relocal-ORB/Kinectv2.yaml";
const std::string map_file = "/home/wu6shen/Computer-Vision/Project/Relocal-ORB/kinect-mappoint";
const std::string map_file_test = "/home/wu6shen/Computer-Vision/Project/Relocal-ORB/kinect-mappoint-test";
const std::string gth_file = "/home/wu6shen/Computer-Vision/Data/rgbd_dataset_freiburg1_xyz/groundtruth.txt";
const std::string voc_file = "/home/wu6shen/Computer-Vision/Project/Relocal-ORB/ORBvoc.txt";
/*
 */

void LoadImages(const std::string &strFile, std::vector<std::string> &vstrImageFilenames, std::vector<double> &vTimestamps)
{
    std::ifstream f;
    std::cout << strFile.c_str() << std::endl;
    f.open(strFile.c_str());

    // skip first three lines
    std::string s0;
    getline(f,s0);
    getline(f,s0);
    getline(f,s0);

    while(!f.eof())
    {
        std::string s;
        getline(f,s);
        if(!s.empty())
        {
            std::stringstream ss;
            ss << s;
            double t;
            std::string sRGB;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenames.push_back(sRGB);
        }
    }
}

void RunRelocal() {
    ORB_SLAM2::System *relocal = new ORB_SLAM2::System(setting_file, map_file, gth_file, true);
    std::vector<std::string> image_files;
    std::vector<double> timestamps;
    LoadImages(image_list_file, image_files, timestamps);
    int num = 0;
    //cv::Mat img = cv::imread(image_folder + image_files[0]);
    //relocal->Relocal(img, num);
    for (auto image_file : image_files) {
        //if (num > 1 && num <= 4) continue;
        num++;
        cv::Mat img = cv::imread(image_folder + image_file);
        relocal->Relocal(img, num);
        sleep(5);
    }
    while (1);
}

void RunSlam() {
    ORB_SLAM2::System SLAM(voc_file, setting_file, ORB_SLAM2::System::MONOCULAR, true);
    std::vector<double> timestamps;
    std::vector<std::string> image_files;
    LoadImages(image_list_file, image_files, timestamps);
    int num = image_files.size();

    std::vector<float> timestrack;
    cv::Mat im;
    for (int i = 0; i < num; i++) {
        std::string image_file = image_files[i];
        im = cv::imread(image_folder + image_file);
        double tframe = timestamps[i];

        SLAM.TrackMonocular(im, tframe);
    }
    SLAM.Save("/home/wu6shen/Computer-Vision/Project/Relocal-ORB/kinect-mappoint-test");
    SLAM.Shutdown();
}

void TestMYFPCS() {
    PointCloud<PointXYZ>::Ptr cloud_source_ptr(new PointCloud<PointXYZ>), cloud_target_ptr(new PointCloud<PointXYZ>), cloud_result_ptr(new PointCloud<PointXYZ>), cloud_temp_ptr(new PointCloud<PointXYZ>);
    std::ifstream in;
    in.open(map_file);
    std::string str;
    while (getline(in, str)) {
        float x, y, z;
        sscanf(str.c_str(), "%f %f %f", &x, &y, &z);
        PointXYZ point = {x, y, z};
        getline(in, str);
        cloud_source_ptr->push_back(point);
        cloud_temp_ptr->push_back(point);

    }
    in.close();
    in.open(map_file_test);
    while (getline(in, str)) {
        float x, y, z;
        sscanf(str.c_str(), "%f %f %f", &x, &y, &z);
        PointXYZ point = {x, y, z};
        getline(in, str);
        cloud_target_ptr->push_back(point);

    }
    /**
     */
    /**
    PointXYZ point = {0, 0, 0};
    cloud_source_ptr->push_back(point);
    point = {0, 1, 0};
    cloud_source_ptr->push_back(point);
    point = {1, 1, 0};
    cloud_source_ptr->push_back(point);
    point = {2, 0, 0};
    cloud_source_ptr->push_back(point);
     */

    float angle = static_cast<float> (M_PI) / 4.f;
    Eigen::Quaternionf initial_rotation (cos (angle / 2.f), 0, 0, sin (angle / 2.f));
    Eigen::Vector3f initial_offset (1.f, 0.f, 0.0f);
    Eigen::Matrix3f R = initial_rotation.toRotationMatrix();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    //transformPointCloud(*cloud_temp_ptr, *cloud_target_ptr, initial_offset, initial_rotation);
    FPCS::FPCSRegistration test;
    test.SetRotationGT(R.inverse());
    test.SetOffsetGT(initial_offset);
    test.SetInputSourceCloud(cloud_target_ptr);
    test.SetInputTargetCloud(cloud_source_ptr);
    std::cout << "Inlier Num : " << test.Align(cloud_result_ptr) << std::endl;
    Eigen::Matrix4f alianT = test.GetTransform();
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            std::cout << R(i, j) << " ";
        }
        puts("");
    }
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << alianT(i, j) << " ";
        }
        puts("");
    }
}

/**
void TestPCLFPCS() {
    PointCloud<PointXYZ>::Ptr cloud_source_ptr, cloud_target_ptr;
    PointCloud<PointXYZ> cloud_source, cloud_target, cloud_tmp;

    std::ifstream in;
    in.open(map_file);
    std::string str;
    while (getline(in, str)) {
        float x, y, z;
        sscanf(str.c_str(), "%f %f %f", &x, &y, &z);
        PointXYZ point = {x, y, z};
        getline(in, str);
        cloud_source.push_back(point);
        if (rand() % 4 == 0) cloud_tmp.push_back(point);
    }

    float angle = static_cast<float> (M_PI) / 4.f;
    Eigen::Quaternionf initial_rotation (cos (angle / 2.f), 0, 0, sin (angle / 2.f));
    Eigen::Vector3f initial_offset (0.f, 0.f, 0.0f);
    Eigen::Matrix3f R = initial_rotation.toRotationMatrix();
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    transformPointCloud(cloud_tmp, cloud_target, initial_offset, initial_rotation);
    cloud_source_ptr = cloud_source.makeShared();
    cloud_target_ptr = cloud_target.makeShared();

    PointCloud <PointXYZ> source_aligned;

    FPCSInitialAlignment <PointXYZ, PointXYZ> fpcs_ia;
    fpcs_ia.setInputSource (cloud_source_ptr);
    fpcs_ia.setInputTarget (cloud_target_ptr);

    fpcs_ia.setNumberOfThreads (1);
    fpcs_ia.setApproxOverlap (0.9);
    fpcs_ia.setDelta (1.0, true);
    fpcs_ia.setNumberOfSamples (0);

    // align
    fpcs_ia.align (source_aligned);
    //EXPECT_EQ (static_cast <int> (source_aligned.points.size ()), static_cast <int> (cloud_source.points.size ()));

    // check for correct coarse transformation marix
    Eigen::Matrix4f transform_res_from_fpcs = fpcs_ia.getFinalTransformation ();

    for (size_t i = 0; i < source_aligned.size(); i++) {
        cout << cloud_target.at(i) <<  " ";
        PointXYZ point = cloud_tmp.at(i);
        Eigen::Vector4f p3f;
        p3f(0) = point.x;
        p3f(1) = point.y;
        p3f(2) = point.z;
        p3f(3) = 1;
        cout << transform_res_from_fpcs * p3f << endl;
    }

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) cout << R(i, j) << " ";
        cout << endl;
    }

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j)
            cout << transform_res_from_fpcs(i, j) << " ";
        cout << endl;
    }
}
*/

int main() {
    RunSlam();
    //TestMYFPCS();
}