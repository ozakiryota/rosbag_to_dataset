#include <filesystem>
#include <nlohmann/json.hpp>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class RosbagToDataset{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*publisher*/
		ros::Publisher pc_pub_;
		ros::Publisher image_pub_;
		ros::Publisher imu_pub_;
		ros::Publisher odom_pub_;
        /*buffer*/
        size_t num_save_ = 0;
        sensor_msgs::PointCloud2ConstPtr pc_ptr_;
        sensor_msgs::CompressedImageConstPtr image_ptr_;
        sensor_msgs::ImuConstPtr imu_ptr_;
        nav_msgs::OdometryConstPtr odom_ptr_;
        ros::Time last_saved_stamp_;
        nav_msgs::Odometry last_saved_odom_;
        /*flag*/
        bool pc_is_buffered_ = false;
        bool image_is_buffered_ = false;
        bool imu_is_buffered_ = false;
        bool odom_is_buffered_ = false;
        /*file*/
        std::ofstream file_list_csv_;
		/*parameter*/
		std::string rosbag_path_;
		std::string save_dir_;
		std::string save_csv_path_;
		std::string save_txt_path_;
        float min_time_diff_sec_;
        float min_odom_diff_m_;
        float min_odom_diff_deg_;
        float debug_hz_;
		std::string debug_frame_;

	public:
		RosbagToDataset();
        std::string getDefaultSaveDir();
        void createDirectory();
        void load();
        bool isReadyToSave(const rosbag::View::iterator& view_itr);
        void getOdomDiff(float& largest_diff_m, float& largest_diff_deg);
		void save();
		void publishDebugMsg();
        float anglePiToPi(const float& angle);
};

RosbagToDataset::RosbagToDataset()
	: nh_private_("~")
{
	std::cout << "----- rosbag_to_dataset -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("rosbag_path", rosbag_path_)){
        std::cerr << "Set rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "rosbag_path_ = " << rosbag_path_ << std::endl;
    nh_private_.param("save_dir", save_dir_, getDefaultSaveDir());
	std::cout << "save_dir_ = " << save_dir_ << std::endl;
    nh_private_.param("save_csv_path", save_csv_path_, std::string(save_dir_ + "/file_list.csv"));
	std::cout << "save_csv_path_ = " << save_csv_path_ << std::endl;
    nh_private_.param("save_txt_path", save_txt_path_, std::string(save_dir_ + "/param.txt"));
	std::cout << "save_txt_path_ = " << save_txt_path_ << std::endl;

    nh_private_.param("min_time_diff_sec", min_time_diff_sec_, float(-1));
	std::cout << "min_time_diff_sec_ = " << min_time_diff_sec_ << std::endl;
    nh_private_.param("min_odom_diff_m", min_odom_diff_m_, float(-1));
	std::cout << "min_odom_diff_m_ = " << min_odom_diff_m_ << std::endl;
    nh_private_.param("min_odom_diff_deg", min_odom_diff_deg_, float(-1));
	std::cout << "min_odom_diff_deg_ = " << min_odom_diff_deg_ << std::endl;

    nh_private_.param("debug_hz", debug_hz_, float(-1));
	std::cout << "debug_hz_ = " << debug_hz_ << std::endl;
    nh_private_.param("debug_frame", debug_frame_, std::string("debug"));
	std::cout << "debug_frame_ = " << debug_frame_ << std::endl;

    /*publisher*/
	pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/debug/point_cloud", 1);
	image_pub_ = nh_.advertise<sensor_msgs::CompressedImage>("/debug/image/compressed", 1);
	imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/debug/imu", 1);
	odom_pub_ = nh_.advertise<nav_msgs::Odometry>("/debug/odom", 1);
}

std::string RosbagToDataset::getDefaultSaveDir()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string save_path(tmp ? tmp : "");
    if(save_path.empty()){
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    save_path += std::string("/src/rosbag_to_dataset/save/") + rosbag_path_;
    return save_path;
}

void RosbagToDataset::createDirectory()
{
    std::filesystem::remove_all(save_dir_);
    std::filesystem::create_directory(save_dir_);

    file_list_csv_.open(save_csv_path_, std::ios::app);
	if(!file_list_csv_){
		std::cout << "Cannot open " << save_csv_path_ << std::endl;
		exit(true);
	}

	std::ofstream param_txt;
    param_txt.open(save_txt_path_, std::ios::app);
	if(!param_txt){
		std::cout << "Cannot open " << save_txt_path_ << std::endl;
		exit(true);
	}
    param_txt << "rosbag_path_ = " << rosbag_path_ << std::endl
        << "min_time_diff_sec_ = " << min_time_diff_sec_ << std::endl
        << "min_odom_diff_m_ = " << min_odom_diff_m_ << std::endl
        << "min_odom_diff_deg_ = " << min_odom_diff_deg_ << std::endl;
	param_txt.close();
}

void RosbagToDataset::load()
{
    rosbag::Bag bag;
    rosbag::View view;
    rosbag::View::iterator view_itr;

    try{
        bag.open(rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << rosbag_path_ << std::endl;
        exit(true);
    }

    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/PointCloud2"));
    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/CompressedImage"));
    view.addQuery(bag, rosbag::TypeQuery("sensor_msgs/Imu"));
    view.addQuery(bag, rosbag::TypeQuery("nav_msgs/Odometry"));
    view_itr = view.begin();

    ros::Rate loop_rate(debug_hz_);
    while(view_itr != view.end()){
        if(view_itr->instantiate<sensor_msgs::PointCloud2>() != NULL){
            pc_ptr_ = view_itr->instantiate<sensor_msgs::PointCloud2>();
            pc_is_buffered_ = true;
        }
        if(view_itr->instantiate<sensor_msgs::CompressedImage>() != NULL){
            image_ptr_ = view_itr->instantiate<sensor_msgs::CompressedImage>();
            image_is_buffered_ = true;
        }
        if(view_itr->instantiate<sensor_msgs::Imu>()){
            imu_ptr_ = view_itr->instantiate<sensor_msgs::Imu>();
            imu_is_buffered_ = true;
        }
        if(view_itr->instantiate<nav_msgs::Odometry>()){
            odom_ptr_ = view_itr->instantiate<nav_msgs::Odometry>();
            odom_is_buffered_ = true;
        }

        if(isReadyToSave(view_itr)){
            save();

            last_saved_stamp_ = view_itr->getTime();
            last_saved_odom_ = *odom_ptr_;
            num_save_++;
            pc_is_buffered_ = false;
            image_is_buffered_ = false;
            imu_is_buffered_ = false;
            odom_is_buffered_ = false;

            publishDebugMsg();
            if(debug_hz_ > 0)    loop_rate.sleep();
        }

        view_itr++;
    }

    file_list_csv_.close();
    std::cout << "Save: " << save_csv_path_ << std::endl;
}

bool RosbagToDataset::isReadyToSave(const rosbag::View::iterator& view_itr)
{
    if(pc_is_buffered_ && image_is_buffered_ && imu_is_buffered_ && odom_is_buffered_){
        if(num_save_ == 0)  return true;
        if(min_time_diff_sec_ > 0){
            if((view_itr->getTime() - last_saved_stamp_).toSec() < min_time_diff_sec_)  return false;
        }
        float largest_diff_m, largest_diff_deg;
        getOdomDiff(largest_diff_m, largest_diff_deg);
        if(min_odom_diff_m_ > 0 && largest_diff_m < min_odom_diff_m_)   return false;
        if(min_odom_diff_deg_ > 0 && largest_diff_deg < min_odom_diff_deg_)   return false;
        return true;
    }
    else    return false;
}

void RosbagToDataset::getOdomDiff(float& largest_diff_m, float& largest_diff_deg)
{
    float diff_x = std::abs(odom_ptr_->pose.pose.position.x - last_saved_odom_.pose.pose.position.x);
    float diff_y = std::abs(odom_ptr_->pose.pose.position.y - last_saved_odom_.pose.pose.position.y);
    float diff_z = std::abs(odom_ptr_->pose.pose.position.z - last_saved_odom_.pose.pose.position.z);
    largest_diff_m = std::max(std::max(diff_x, diff_y), diff_z);

    tf::Quaternion curr_q, last_q;
    quaternionMsgToTF(odom_ptr_->pose.pose.orientation, curr_q);
    quaternionMsgToTF(last_saved_odom_.pose.pose.orientation, last_q);
    largest_diff_deg = last_q.angleShortestPath(curr_q) / M_PI * 180.0;
}

void RosbagToDataset::save()
{
    std::string save_sub_dir = save_dir_ + "/" + std::to_string(num_save_);
    std::filesystem::create_directory(save_sub_dir);

    /*pc*/
    std::stringstream save_pc_path;
    save_pc_path << save_sub_dir << "/" << pc_ptr_->header.stamp << ".pcd";
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc_ptr_, *pcl_pc);
    pcl::io::savePCDFileASCII(save_pc_path.str(), *pcl_pc);
    std::cout << "Save: " << save_pc_path.str() << std::endl;
    /*image*/
    std::stringstream save_image_path;
    save_image_path << save_sub_dir << "/" << image_ptr_->header.stamp << ".jpeg";
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_ptr_, sensor_msgs::image_encodings::BGR8);
    cv::imwrite(save_image_path.str(), cv_ptr->image);
    std::cout << "Save: " << save_image_path.str() << std::endl;
    /*others*/
    std::string save_json_path = save_sub_dir + "/data.json";
    nlohmann::json data = {
        {"imu", {
            {"stamp", imu_ptr_->header.stamp.toSec()},
            {"angular_velocity", {
                {"x", imu_ptr_->angular_velocity.x},
                {"y", imu_ptr_->angular_velocity.y},
                {"z", imu_ptr_->angular_velocity.z}
            }},
            {"linear_acceleration", {
                {"x", imu_ptr_->linear_acceleration.x},
                {"y", imu_ptr_->linear_acceleration.y},
                {"z", imu_ptr_->linear_acceleration.z}
            }}
        }},
        {"odometry", {
            {"stamp", odom_ptr_->header.stamp.toSec()},
            {"position", {
                {"x", odom_ptr_->pose.pose.position.x},
                {"y", odom_ptr_->pose.pose.position.y},
                {"z", odom_ptr_->pose.pose.position.z}
            }},
            {"orientation", {
                {"x", odom_ptr_->pose.pose.orientation.x},
                {"y", odom_ptr_->pose.pose.orientation.y},
                {"z", odom_ptr_->pose.pose.orientation.z},
                {"w", odom_ptr_->pose.pose.orientation.w}
            }}
        }}
    };
    std::ofstream data_json(save_json_path);
    data_json << data;
    std::cout << "Save: " << save_json_path << std::endl;
    /*csv*/
    file_list_csv_ << save_pc_path.str() << "," << save_image_path.str() << "," << save_json_path << std::endl;
}

void RosbagToDataset::publishDebugMsg()
{
    /*pc*/
    sensor_msgs::PointCloud2 debug_pc = *pc_ptr_;
    debug_pc.header.frame_id = debug_frame_;
    pc_pub_.publish(debug_pc);
    /*image*/
    sensor_msgs::CompressedImage debug_image = *image_ptr_;
    debug_image.header.frame_id = debug_frame_;
    image_pub_.publish(debug_image);
    /*imu*/
    sensor_msgs::Imu debug_imu = *imu_ptr_;
    debug_imu.header.frame_id = debug_frame_;
    imu_pub_.publish(debug_imu);
    /*odom*/
    nav_msgs::Odometry debug_odom = *odom_ptr_;
    debug_odom.header.frame_id = debug_frame_;
    odom_pub_.publish(debug_odom);
}

float RosbagToDataset::anglePiToPi(const float& angle)
{
	return atan2(sin(angle), cos(angle)); 
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_dataset");
	
	RosbagToDataset rosbag_to_dataset;
    rosbag_to_dataset.createDirectory();
    rosbag_to_dataset.load();
}