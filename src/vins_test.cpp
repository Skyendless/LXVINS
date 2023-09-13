//
// Create by Huws , 2023-09-12
//

#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>

#include <opencv2/opencv.hpp>
#include "estimator/estimator.h"
#include "estimator/parameters.h"
// #include <pangolin/pangolin.h>

//imu for vio
struct IMU_MSG
{
    double header;
    Eigen::Vector3d linear_acceleration;
    Eigen::Vector3d angular_velocity;
};
typedef std::shared_ptr<IMU_MSG> ImuPtr;

//image for vio    
struct IMG_MSG {
    double header;
    cv::Mat image_left;
    cv::Mat image_right;
};
typedef std::shared_ptr<IMG_MSG> ImgPtr;

struct VIO_Pose {
    double time;
    Eigen::Quaterniond vio_qwc;
    Eigen::Vector3d vio_twc;
};

Estimator estimator;

string pathCam0,pathCam1,pathImu,strPathTimes;

std::vector<VIO_Pose> vecVIO_Poses;

// img_thread
void LoadImages(void)
{
    string strPathTimes = "/home/efsz/workspace/efvins/config/euroc/MH04.txt";
    string strImageLeftPath;
    string strImageRightPath;
    ifstream fTimes;
    fTimes.open(strPathTimes.c_str());
    int dt = 0;
    // static double time_sum = 0;
    // static double max_time = -99999;
    // static double min_time = 99999;
    // static int circle_time = 0;
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            strImageLeftPath = pathCam0 + "/data/" + ss.str() + ".png";
            strImageRightPath = pathCam1 + "/data/" + ss.str() + ".png";
            
            double t;
            ss >> t;
            // vTimeStamps.push_back(t/1e9);
            double image_time = t/1e9;

            cv::Mat img_cam0 = cv::imread(strImageLeftPath, cv::IMREAD_GRAYSCALE);
            cv::Mat img_cam1 = cv::imread(strImageRightPath, cv::IMREAD_GRAYSCALE);

            // TicToc t_1;
            estimator.inputImage(image_time, img_cam0, img_cam1);
            // double cur_spin_cost = t_1.toc();
            // printf("estimator.inputImage cost %f ms.\n", cur_spin_cost);
            // circle_time ++;
            // if(cur_spin_cost > max_time) {
            //     max_time = cur_spin_cost;
            // }
            // time_sum += cur_spin_cost;
            // double average_time = time_sum / circle_time;
            // LOG(INFO) << "VO cost time: " << cur_spin_cost << "  ms."
            //             <<"vo average : " << average_time << " ms."
            //             <<" max :" << max_time << " ms. ";

            // cv::waitKey(1);
            //
            Eigen::Matrix<double, 4, 4> pose;
            estimator.getPoseInWorldFrame(pose);
            Eigen::Matrix3d cur_vio_R;
            Eigen::Vector3d cur_vio_t;
            cur_vio_R << pose(0,0), pose(0,1), pose(0,2),
                        pose(1,0), pose(1,1), pose(1,2),
                        pose(2,0), pose(2,1), pose(2,2);
            cur_vio_t << pose(0,3), pose(1,3), pose(2,3); 
            // std::cout << "cur_vio_pose is " << cur_vio_t.transpose() << std::endl;
            VIO_Pose vio_pose_tmp;
            vio_pose_tmp.time = image_time;
            vio_pose_tmp.vio_qwc = cur_vio_R;
            vio_pose_tmp.vio_twc = cur_vio_t;
            vecVIO_Poses.push_back(vio_pose_tmp);
            // vPath_to_draw.push_back(cur_vio_t);
            //
            // dt = 50 - (int)t_1.toc();
            // cv::imshow("img_cam0", img_cam0);
            // cv::imshow("img_cam1", img_cam1);
            // cv::waitKey(1);
            // std::cout << std::fixed <<"image_time is : " << image_time 
            //     << " , image0_path is : " << strImageLeftPath 
            //     << " , image1_path is : " << strImageRightPath << std::endl;
        }
        //
        if(dt <= 0 ) {
            dt = 1;
        }
        std::chrono::milliseconds dura(dt);
        std::this_thread::sleep_for(dura);
    }
}

// imu_thread
void LoadIMU(void)
{

}

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        printf("please intput: ./vins_test [config file] \n"
               "for example: ./vins_test ../config/euroc/euroc_stereo_imu_config.yaml \n");
        return -1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    readParameters(config_file);
    estimator.setParameter();

    //multi thread
    printf("waiting for image and imu...");

    std::thread img_trd(LoadImages);
    std::thread imu_trd(LoadIMU);

    // visualization
    // vPath_to_draw.clear();
    // std::thread thd_Draw(Draw);
    // thd_Draw.detach();

    img_trd.join();
    if (USE_IMU)
    {
        imu_trd.join();
    }

    // save trajectory
    // SaveKeyFrameTrajectory("./kf_pose.txt");

    cv::waitKey(0);
    return 0;
}

