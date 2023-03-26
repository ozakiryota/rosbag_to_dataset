#include <filesystem>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>


class RosbagToCsv{
	private:
		/*node handle*/
		ros::NodeHandle nh_;
		ros::NodeHandle nh_private_;
        /*buffer*/
        std::vector<std::string> topic_name_list_;
        ros::Time buffered_clock_;
        /*message*/
        struct Topic{
            std::string topic_name;
            bool is_buffered = false;
        };
        struct OdomTopic : Topic{
            nav_msgs::OdometryConstPtr msg_ptr;
        };
        struct ImuTopic : Topic{
            sensor_msgs::ImuConstPtr msg_ptr;
        };
        std::vector<OdomTopic> odom_topic_list_;
        std::vector<ImuTopic> imu_topic_list_;
        /*file*/
        std::ofstream written_csv_;
		/*parameter*/
		std::string read_rosbag_path_;
		std::string write_csv_path_;
        /*function*/
        std::string getDefaultCsvPath();
        void getTopicList();
		void writeHeader();
		void writeData();

	public:
		RosbagToCsv();
        void execute();
};

RosbagToCsv::RosbagToCsv()
	: nh_private_("~")
{
	std::cout << "----- rosbag_to_csv -----" << std::endl;

	/*parameter*/
    if(!nh_private_.getParam("read_rosbag_path", read_rosbag_path_)){
        std::cerr << "Set read_rosbag_path." << std::endl; 
        exit(true);
    }
	std::cout << "read_rosbag_path_ = " << read_rosbag_path_ << std::endl;
    nh_private_.param("write_csv_path", write_csv_path_, getDefaultCsvPath());
	std::cout << "write_csv_path_ = " << write_csv_path_ << std::endl;

    getTopicList();

    /*file*/
    written_csv_.open(write_csv_path_, std::ios::out);
	if(!written_csv_){
		std::cout << "Cannot open " << write_csv_path_ << std::endl;
		exit(true);
	}
    writeHeader();
}

std::string RosbagToCsv::getDefaultCsvPath()
{
    const char *tmp = getenv("ROS_WORKSPACE");
    std::string csv_path(tmp ? tmp : "");
    if(csv_path.empty()){
        std::cerr << "Cannot get $ROS_WORKSPACE." << std::endl;
        exit(true);
    }
    csv_path += std::string("/src/rosbag_to_dataset/save/") + std::filesystem::path(read_rosbag_path_).stem().string() + ".csv";
    return csv_path;
}

void RosbagToCsv::getTopicList()
{
    for(size_t i = 0; ; i++){
        OdomTopic tmp_topic;
        if(!nh_private_.getParam("odom_" + std::to_string(i), tmp_topic.topic_name))  break;
        odom_topic_list_.push_back(tmp_topic);
        topic_name_list_.push_back(tmp_topic.topic_name);
        std::cout << "odom_topic_list_[" << i << "].topic_name = " << odom_topic_list_[i].topic_name << std::endl;
    }
    for(size_t i = 0; ; i++){
        ImuTopic tmp_topic;
        if(!nh_private_.getParam("imu_" + std::to_string(i), tmp_topic.topic_name))  break;
        imu_topic_list_.push_back(tmp_topic);
        topic_name_list_.push_back(tmp_topic.topic_name);
        std::cout << "imu_topic_list_[" << i << "].topic_name = " << imu_topic_list_[i].topic_name << std::endl;
    }
}

void RosbagToCsv::writeHeader()
{
    written_csv_ << "time_sec";

    for(OdomTopic& topic : odom_topic_list_){
        written_csv_ << "," << topic.topic_name + "/pose/pose/position/x";
        written_csv_ << "," << topic.topic_name + "/pose/pose/position/y";
        written_csv_ << "," << topic.topic_name + "/pose/pose/position/z";
        written_csv_ << "," << topic.topic_name + "/pose/pose/orientation/x";
        written_csv_ << "," << topic.topic_name + "/pose/pose/orientation/y";
        written_csv_ << "," << topic.topic_name + "/pose/pose/orientation/z";
        written_csv_ << "," << topic.topic_name + "/pose/pose/orientation/w";
        written_csv_ << "," << topic.topic_name + "/twist/twist/linear/x";
        written_csv_ << "," << topic.topic_name + "/twist/twist/linear/y";
        written_csv_ << "," << topic.topic_name + "/twist/twist/linear/z";
        written_csv_ << "," << topic.topic_name + "/twist/twist/angular/x";
        written_csv_ << "," << topic.topic_name + "/twist/twist/angular/y";
        written_csv_ << "," << topic.topic_name + "/twist/twist/angular/z";
    }
    for(ImuTopic& topic : imu_topic_list_){
        written_csv_ << "," << topic.topic_name + "/angular_velocity/x";
        written_csv_ << "," << topic.topic_name + "/angular_velocity/y";
        written_csv_ << "," << topic.topic_name + "/angular_velocity/z";
        written_csv_ << "," << topic.topic_name + "/linear_acceleration/x";
        written_csv_ << "," << topic.topic_name + "/linear_acceleration/y";
        written_csv_ << "," << topic.topic_name + "/linear_acceleration/z";
    }

    written_csv_ << std::endl;
}

void RosbagToCsv::execute()
{
    rosbag::Bag bag;

    try{
        bag.open(read_rosbag_path_, rosbag::bagmode::Read);
    }
    catch(rosbag::BagException const&){
        std::cerr << "Cannot open " << read_rosbag_path_ << std::endl;
        exit(true);
    }

    rosbag::View view(bag, rosbag::TopicQuery(topic_name_list_));
    rosbag::View::iterator view_itr;
    view_itr = view.begin();

    while(view_itr != view.end()){
        if(view_itr->getDataType() == "nav_msgs/Odometry"){
            for(OdomTopic& topic : odom_topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    if(topic.is_buffered)   writeData();
                    topic.msg_ptr = view_itr->instantiate<nav_msgs::Odometry>();
                    topic.is_buffered = true;
                    break;
                }
            }
        }
        else if(view_itr->getDataType() == "sensor_msgs/Imu"){
            for(ImuTopic& topic : imu_topic_list_){
                if(view_itr->getTopic() == topic.topic_name){
                    if(topic.is_buffered)   writeData();
                    topic.msg_ptr = view_itr->instantiate<sensor_msgs::Imu>();
                    topic.is_buffered = true;
                    break;
                }
            }
        }
        else{
            std::cerr << view_itr->getTopic() << " is not supported msg." << std::endl;
            continue;
        }
        buffered_clock_ = view_itr->getTime();
        view_itr++;
    }

    bag.close();
    written_csv_.close();
    std::cout << "Save: " << write_csv_path_ << std::endl;
}

void RosbagToCsv::writeData()
{
    const size_t default_precition = std::cout.precision();
    const size_t clock_precition = 20;
    written_csv_ << std::setprecision(clock_precition) << buffered_clock_.toSec();
    written_csv_ << std::setprecision(default_precition);

    const size_t odom_data_size = 13;
    for(OdomTopic& topic : odom_topic_list_){
        if(topic.is_buffered){
            written_csv_ << "," << topic.msg_ptr->pose.pose.position.x;
            written_csv_ << "," << topic.msg_ptr->pose.pose.position.y;
            written_csv_ << "," << topic.msg_ptr->pose.pose.position.z;
            written_csv_ << "," << topic.msg_ptr->pose.pose.orientation.x;
            written_csv_ << "," << topic.msg_ptr->pose.pose.orientation.y;
            written_csv_ << "," << topic.msg_ptr->pose.pose.orientation.z;
            written_csv_ << "," << topic.msg_ptr->pose.pose.orientation.w;
            written_csv_ << "," << topic.msg_ptr->twist.twist.linear.x;
            written_csv_ << "," << topic.msg_ptr->twist.twist.linear.y;
            written_csv_ << "," << topic.msg_ptr->twist.twist.linear.z;
            written_csv_ << "," << topic.msg_ptr->twist.twist.angular.x;
            written_csv_ << "," << topic.msg_ptr->twist.twist.angular.y;
            written_csv_ << "," << topic.msg_ptr->twist.twist.angular.z;
        }
        else{
            for(size_t i = 0; i < odom_data_size; i++)   written_csv_ << ",";
        }
        topic.is_buffered = false;
    }
    const size_t imu_data_size = 6;
    for(ImuTopic& topic : imu_topic_list_){
        if(topic.is_buffered){
            written_csv_ << "," << topic.msg_ptr->angular_velocity.x;
            written_csv_ << "," << topic.msg_ptr->angular_velocity.y;
            written_csv_ << "," << topic.msg_ptr->angular_velocity.z;
            written_csv_ << "," << topic.msg_ptr->linear_acceleration.x;
            written_csv_ << "," << topic.msg_ptr->linear_acceleration.y;
            written_csv_ << "," << topic.msg_ptr->linear_acceleration.z;
        }
        else{
            for(size_t i = 0; i < imu_data_size; i++)   written_csv_ << ",";
        }
        topic.is_buffered = false;
    }

    written_csv_ << std::endl;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "rosbag_to_csv");
	
	RosbagToCsv rosbag_to_csv;
    rosbag_to_csv.execute();
}