#include <rclcpp/rclcpp.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include "pandarGeneral_sdk/pandarGeneral_sdk.h"

#include <boost/lambda/lambda.hpp>

using namespace std;
using boost::lambda::_1;

class HesaiLidarClient : public rclcpp::Node
{
public:
  HesaiLidarClient()
  : Node("hesai_lidar_node")
  {
    lidarPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pandar", 10);
    packetPublisher = this->create_publisher<hesai_lidar::msg::PandarScan>("pandar_packets", 10);

    string serverIp;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;  // Get local correction when getting from lidar failed
    string lidarType;
    string frameId;
    int pclDataType;
    string pcapFile;
    string dataType;

    // declare parameters and set default values
    this->declare_parameter("pcap_file", "");
    this->declare_parameter("server_ip","192.168.1.201");
    this->declare_parameter("lidar_recv_port",2368);
    this->declare_parameter("gps_port",10110);
    this->declare_parameter("start_angle",0);
    this->declare_parameter("lidar_correction_file","correction.csv");
    this->declare_parameter("lidar_type","PandarQT");
    this->declare_parameter("frame_id","PandarQT");
    this->declare_parameter("pcldata_type",0);
    this->declare_parameter("publish_type","points");
    this->declare_parameter("timestamp_type","");
    this->declare_parameter("data_type","");

    // get parameters
    serverIp = this->get_parameter("server_ip").get_value<string>();
    lidarRecvPort = this->get_parameter("lidar_recv_port").get_value<int>();
    gpsPort = this->get_parameter("gps_port").get_value<int>();
    startAngle = this->get_parameter("start_angle").get_value<int>();
    lidarCorrectionFile = this->get_parameter("lidar_correction_file").get_value<string>();
    lidarType = this->get_parameter("lidar_type").get_value<string>();
    frameId = this->get_parameter("frame_id").get_value<string>();
    pclDataType = this->get_parameter("pcldata_type").get_value<int>();
    m_sPublishType = this->get_parameter("publish_type").get_value<string>();
    m_sTimestampType = this->get_parameter("timestamp_type").get_value<string>();
    dataType = this->get_parameter("data_type").get_value<string>();
    pcapFile = this->get_parameter("pcap_file").get_value<string>();

    if(!pcapFile.empty()){
      hsdk = new PandarGeneralSDK(pcapFile, boost::bind(&HesaiLidarClient::lidarCallback, this, boost::lambda::_1, boost::lambda::_2, boost::lambda::_3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType);
      if (hsdk != NULL) {
        ifstream fin(lidarCorrectionFile);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        hsdk->LoadLidarCorrectionFile(strlidarCalibration);
      }
    }
    else if ("rosbag" == dataType){
      hsdk = new PandarGeneralSDK("", boost::bind(&HesaiLidarClient::lidarCallback, this, boost::lambda::_1, boost::lambda::_2, _3), \
      static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType);
      if (hsdk != NULL) {
        ifstream fin(lidarCorrectionFile);
        int length = 0;
        std::string strlidarCalibration;
        fin.seekg(0, std::ios::end);
        length = fin.tellg();
        fin.seekg(0, std::ios::beg);
        char *buffer = new char[length];
        fin.read(buffer, length);
        fin.close();
        strlidarCalibration = buffer;
        hsdk->LoadLidarCorrectionFile(strlidarCalibration);
        packetSubscriber = this->create_subscription<hesai_lidar::msg::PandarScan>("pandar_packets", 10, std::bind(&HesaiLidarClient::scanCallback, this, std::placeholders::_1));
      }
    }
    else {
      hsdk = new PandarGeneralSDK(serverIp, lidarRecvPort, gpsPort, \
        boost::bind(&HesaiLidarClient::lidarCallback, this, boost::lambda::_1, boost::lambda::_2, boost::lambda::_3), \
        boost::bind(&HesaiLidarClient::gpsCallback, this, boost::lambda::_1), static_cast<int>(startAngle * 100 + 0.5), 0, pclDataType, lidarType, frameId, m_sTimestampType);
    }

    if (hsdk != NULL) {
        hsdk->Start();
        // hsdk->LoadLidarCorrectionFile("...");  // parameter is stream in lidarCorrectionFile
    } else {
        printf("create sdk fail\n");
    }
  }

  void lidarCallback(boost::shared_ptr<PPointCloud> cld, double timestamp, hesai_lidar::msg::PandarScan::Ptr scan) // the timestamp from first point cloud of cld
  {
    if(m_sPublishType == "both" || m_sPublishType == "points"){
      pcl_conversions::toPCL(rclcpp::Time(timestamp), cld->header.stamp);
      sensor_msgs::msg::PointCloud2 output;
      pcl::toROSMsg(*cld, output);
      lidarPublisher->publish(output);
      printf("timestamp: %f, point size: %ld.\n",timestamp, cld->points.size());
    }
    if(m_sPublishType == "both" || m_sPublishType == "raw"){
      packetPublisher->publish(*scan);
      printf("raw size: %d.\n", scan->packets.size());
    }
  }

  void gpsCallback(int timestamp) {
    printf("gps: %d\n", timestamp);
  }

  void scanCallback(const hesai_lidar::msg::PandarScan::Ptr scan) const
  {
    // printf("pandar_packets topic message received,\n");
    hsdk->PushScanPacket(scan);
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr lidarPublisher;
  rclcpp::Publisher<hesai_lidar::msg::PandarScan>::SharedPtr packetPublisher;
  PandarGeneralSDK* hsdk;
  string m_sPublishType;
  string m_sTimestampType;
  rclcpp::Subscription<hesai_lidar::msg::PandarScan>::SharedPtr packetSubscriber;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HesaiLidarClient>());
  rclcpp::shutdown();
  return 0;
}
