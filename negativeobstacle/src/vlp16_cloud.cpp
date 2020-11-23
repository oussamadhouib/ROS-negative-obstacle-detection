#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <sensor_msgs/LaserScan.h>

#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLImage.h>
#include <pcl/point_cloud.h>
#include <pcl/exceptions.h>
#include <pcl/console/print.h>
#include <pcl/PCLPointField.h>
#include <pcl/PCLPointCloud2.h>

#include <iostream>
#include <ctime>
#include <cstdio>
#include <cstdlib>

#include <tf/transform_broadcaster.h>

#include <std_msgs/Int32.h>

using namespace std;
using namespace pcl;

ros::Publisher pub;
ros::Publisher pub2;
ros::Publisher pub3;
ros::Publisher pub4;
ros::Publisher pub5;
ros::Publisher state_pub;
ros::Publisher pub_final;
int state = 0;
int negative_obstacle =0;


//catkin_make -DCMAKE_BUILD_TYPE=Release

template <typename PointT>
void fromPCLPointCloud2ToVelodyneCloud(const pcl::PCLPointCloud2& msg, pcl::PointCloud<PointT>& cloud1D, std::vector< pcl::PointCloud<PointT> >& cloudVector, unsigned int rings)
{
  cloud1D.header   = msg.header;
  cloud1D.width    = msg.width;
  cloud1D.height   = msg.height;
  cloud1D.is_dense = msg.is_dense == 1;
  uint32_t num_points = msg.width * msg.height;
  cloud1D.points.resize (num_points);
  uint8_t* cloud_data1 = reinterpret_cast<uint8_t*>(&cloud1D.points[0]);
  
    //size_t num_points = cloud_z->size();  

  pcl::PointCloud<PointT>* cloudPerLaser = new pcl::PointCloud<PointT>[rings];
  uint8_t* cloud_data2[rings];

	unsigned int pointsCounter[16]  = {0};

	for(unsigned int i=0; i<rings; ++i)
	{
		cloudPerLaser[i] = pcl::PointCloud<PointT>();
		cloudPerLaser[i].header   = msg.header;
        cloudPerLaser[i].width    = msg.width;
        cloudPerLaser[i].height   = msg.height;
        cloudPerLaser[i].is_dense = msg.is_dense == 1;
        cloudPerLaser[i].points.resize (num_points);
		cloud_data2[i] = reinterpret_cast<uint8_t*>(&cloudPerLaser[i].points[0]);
	}

	for (uint32_t row = 0; row < msg.height; ++row)
	{
	  const uint8_t* row_data = &msg.data[row * msg.row_step];
	    
	  for (uint32_t col = 0; col < msg.width; ++col)
	  {
	      const uint8_t* msg_data = row_data + col * msg.point_step;


	  	  uint16_t* ring = (uint16_t*)(msg_data+20);
	     	memcpy (cloud_data2[*ring], msg_data, 22);
        memcpy (cloud_data1, msg_data, 22);
        pointsCounter[*ring]++;
        cloud_data1 += sizeof (PointT);
	      cloud_data2[*ring] += sizeof (PointT);
	  }
	}

  cloudVector = std::vector< pcl::PointCloud<PointT> >(rings);

	for(unsigned int i=0; i<rings; ++i)
	{
    	cloudPerLaser[i].width = pointsCounter[i];
      cloudPerLaser[i].height = 1;
      cloudPerLaser[i].points.resize (pointsCounter[i]);
      cloudVector[i] = (cloudPerLaser[i]);
  }

  delete[] cloudPerLaser;
}

void cloud_callback (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	


  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud1D(new pcl::PointCloud<pcl::PointXYZI>);
	vector< pcl::PointCloud<pcl::PointXYZI> > cloudVector;
	pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
  fromPCLPointCloud2ToVelodyneCloud (pcl_pc2, *cloud1D, cloudVector, 16);
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZI> ("cloud1D.pcd", *cloud1D, false);




float l1 = cloudVector[0].width;
float l2 = cloudVector[1].width;
float l3 = cloudVector[2].width;
float l4 = cloudVector[3].width;
float l5 = cloudVector[4].width;

/*float h1 = cloudVector[0].height;
float h2 = cloudVector[1].height;
float h3 = cloudVector[2].height;
float h4 = cloudVector[3].height;
float h5 = cloudVector[4].height;*/

//ROS_INFO("width = %f",l2);

float p1,x1,x2,x3,x4,x5;//cloudVector[0][l1/2].x;
float p2;//cloudVector[2][l2/2].x;
float p3;//cloudVector[4][l3/2].x;
float p4,p5;//cloudVector[6][l4/2].x;




// centre du lidar
for(int i=0;i<l1;i++)
  if((cloudVector[0][i].y<0.01)&&(cloudVector[0][i].y>-0.01))
   if(cloudVector[0][i].x>0){
    p1=cloudVector[0][i].x;
    x1=cloudVector[0][i].y;}

for(int i=0;i<l2;i++)
  if((cloudVector[1][i].y<0.01)&&(cloudVector[1][i].y>-0.01))
   if(cloudVector[1][i].x>0){
    p2=cloudVector[1][i].x;
    x2=cloudVector[1][i].y;}

for(int i=0;i<l3;i++)
  if((cloudVector[2][i].y<0.01)&&(cloudVector[2][i].y>-0.01))
   if(cloudVector[2][i].x>0){
    p3=cloudVector[2][i].x;
    x3=cloudVector[2][i].y;}

for(int i=0;i<l4;i++)
  if((cloudVector[3][i].y<0.01)&&(cloudVector[3][i].y>-0.01))
   if(cloudVector[3][i].x>0){
    p4=cloudVector[3][i].x;
    x4=cloudVector[3][i].y;}

for(int i=0;i<l5;i++)
  if((cloudVector[4][i].y<0.01)&&(cloudVector[4][i].y>-0.01))
   if(cloudVector[4][i].x>0){
    p5=cloudVector[4][i].x;
    x5=cloudVector[4][i].y;}

//roue Droite

float pd1,pd2,pd3,pd4,xd1,xd2,xd3,xd4;

for(int i=0;i<l1;i++)
  if((cloudVector[0][i].y<9.78)&&(cloudVector[0][i].y>9.77)) //9.782 = angle fi1 = 90 - arctg(d1/d'1) avec d1 = distance entre roue et ring1 , d'1 = distance entre lidar et roue (+-0.01)
   if(cloudVector[0][i].x>0){
    pd1=cloudVector[0][i].x;
    xd1=cloudVector[0][i].y;}

for(int i=0;i<l2;i++)
  if((cloudVector[1][i].y<7.97)&&(cloudVector[1][i].y>7.96))
   if(cloudVector[1][i].x>0){
    pd2=cloudVector[1][i].x;
    xd2=cloudVector[1][i].y;}

for(int i=0;i<l3;i++)
  if((cloudVector[2][i].y<6.63)&&(cloudVector[2][i].y>6.62))
   if(cloudVector[2][i].x>0){
    pd3=cloudVector[2][i].x;
    xd3=cloudVector[2][i].y;}

for(int i=0;i<l4;i++)
  if((cloudVector[3][i].y<5.39)&&(cloudVector[3][i].y>5.38))
   if(cloudVector[3][i].x>0){
    pd4=cloudVector[3][i].x;
    xd4=cloudVector[3][i].y;}

//roue Gauche

float pg1,pg2,pg3,pg4,xg1,xg2,xg3,xg4;

for(int i=0;i<l1;i++)
  if((cloudVector[0][i].y>-9.78)&&(cloudVector[0][i].y<-9.77)) //9.782 = angle fi1 = 90 - arctg(d1/d'1) avec d1 = distance entre roue et ring1 , d'1 = distance entre lidar et roue (+-0.01)
   if(cloudVector[0][i].x>0){
    pg1=cloudVector[0][i].x;
    xg1=cloudVector[0][i].y;}

for(int i=0;i<l2;i++)
  if((cloudVector[1][i].y>-7.97)&&(cloudVector[1][i].y<-7.96))
   if(cloudVector[1][i].x>0){
    pg2=cloudVector[1][i].x;
    xg2=cloudVector[1][i].y;}

for(int i=0;i<l3;i++)
  if((cloudVector[2][i].y>-6.63)&&(cloudVector[2][i].y<-6.62))
   if(cloudVector[2][i].x>0){
    pg3=cloudVector[2][i].x;
    xg3=cloudVector[2][i].y;}

for(int i=0;i<l4;i++)
  if((cloudVector[3][i].y>-5.39)&&(cloudVector[3][i].y<-5.38))
   if(cloudVector[3][i].x>0){
    pg4=cloudVector[3][i].x;
    xg4=cloudVector[3][i].y;}





int num_points;
float d1= p2-p1;
float d2= p3-p2;
float d3= p4-p3;
float d4= p5-p4;

/*float d11= p1-pd1;
float d111= p1-pg1;
float d22= p2-pd2;
float d222= p2-pg2;*/

//if ( ((d1<0.12)&&(d1>0.09)) || ((d2<0.12)&&(d2>0.09)) || ((d3<0.16)&&(d3>0.14)) || ((d4<0.17)&&(d4>0.15)) )
if ( ((d1<0.17)&&(d1>0.09)) || ((d2<0.17)&&(d2>0.09)) || ((d3<0.21)&&(d3>0.14)) || ((d4<0.23)&&(d4>0.15)) )
  state = 1;
//if ( (d1>0.12) || (d2>0.12) || (d3>0.16) || (d4>0.17) )

if ( (d1>0.17) || (d2>0.185) || (d3>0.225) || (d4>0.25) )
  state = 0;



//negative obstacle...........................................................................................................................

//comparaison des distances (distance initiale et la distance obtenue)
if ( (d1>1) || (d2>1) || (d3>1.3) ) { // pdi > distance de lidar au point de RINGi projete sur le roue
  negative_obstacle =1;
  //pub_final.publish(negative_obstacle);
  ROS_INFO("Attention !!! il existe un obstacle negative " );}


else 
  {negative_obstacle =0;
  //pub_final.publish(negative_obstacle);
  ROS_INFO("Avancez ==>");}  
  


// fin negative obstacle........................................................................................................................

 
  
//ROS_INFO("angle = %f  width = %f  p1 = %f   p2 = %f     p3 = %f     p4 = %f    p5=%f   d1=%f    d2=%f     d3=%f     d4=%f",l1,p1,p2,p3,p4,p5,d1,d2,d3,d4);
//ROS_INFO("y1 = %f   y2 = %f   y3 = %f     y4 = %f",x1,x2,x3,x4,x5);

sensor_msgs::PointCloud2 object_msg;
pcl::toROSMsg(cloudVector[0],object_msg );
object_msg.header.frame_id="/rings";

sensor_msgs::PointCloud2 object_msg2;
pcl::toROSMsg(cloudVector[1],object_msg2 );
object_msg.header.frame_id="/rings";

sensor_msgs::PointCloud2 object_msg3;
pcl::toROSMsg(cloudVector[2],object_msg3 );
object_msg.header.frame_id="/rings";

sensor_msgs::PointCloud2 object_msg4;
pcl::toROSMsg(cloudVector[3],object_msg4 );
object_msg.header.frame_id="/rings";

sensor_msgs::PointCloud2 object_msg5;
pcl::toROSMsg(cloudVector[4],object_msg5 );
object_msg.header.frame_id="/rings";




  static tf::TransformBroadcaster br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);

  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rings", "base_footprint"));


 pub.publish(object_msg);
 pub2.publish(object_msg2);
 pub3.publish(object_msg3);
 pub4.publish(object_msg4);
 pub5.publish(object_msg5);
 state_pub.publish(state);
 pub_final.publish(negative_obstacle);

}

int main (int argc, char** argv)
{
  ros::init (argc, argv, "vlp16_cloud");
  ros::NodeHandle nh;

  pub = nh.advertise<sensor_msgs::PointCloud2>("ring1", 1000);
  pub2 = nh.advertise<sensor_msgs::PointCloud2>("ring2", 1000);
  pub3 = nh.advertise<sensor_msgs::PointCloud2>("ring3", 1000);
  pub4 = nh.advertise<sensor_msgs::PointCloud2>("ring4", 1000);
  pub5 = nh.advertise<sensor_msgs::PointCloud2>("ring5", 1000);
  state_pub = nh.advertise<std_msgs::Int32>("state", 1000);
  pub_final = nh.advertise<std_msgs::Int32>("info_detect", 1000);

  ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2> ("/points_ground", 10, cloud_callback);
  ros::spin ();
}
