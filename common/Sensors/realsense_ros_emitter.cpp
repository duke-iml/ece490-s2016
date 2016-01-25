#include <KrisLibrary/myfile.h>
#include <KrisLibrary/utils/AnyCollection.h>
#include <KrisLibrary/Timer.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>
using namespace std;

string ros_rgb_topic = "/realsense/rgb";
string ros_depth_topic = "/realsense/depth";
string ros_pc_topic = "/realsense/pc";

static const int packet_size = 1024*1024;

bool IsBigEndian() {
  int n = 1;
  // little endian if true
  if(*(char *)&n == 1) return false;
  return true;
}

bool ReadIntPrependedString(File& file,std::string& buf)
{
  int slen;
  if(!file.ReadData(&slen,4)) {
    fprintf(stderr,"Socket::ReadString read length failed\n");
    return false;
  }
  if(slen < 0) {
    fprintf(stderr,"ReadIntPrependedString read length %d is negative\n",slen);
    return false;
  }
  buf.resize(slen);
  if(!file.ReadData(&buf[0],slen)) {
    fprintf(stderr,"ReadIntPrependedString read string failed\n");
    return false;
  }
  return true;
}

class Loop
{
public:
  Loop(ros::NodeHandle& nh) {
    startTime = ros::Time::now();
    rgb_pub = nh.advertise<sensor_msgs::Image>(ros_rgb_topic,1,false);
    depth_pub = nh.advertise<sensor_msgs::Image>(ros_depth_topic,1,false);
    pc_pub = nh.advertise<sensor_msgs::PointCloud2>(ros_pc_topic,1,false);
    frames_read = 0;
    bytes_read = 0;
  }
  bool ReadAndProcess(File& file);
  bool ReadImage(AnyCollection& msg,File& datastream);
  bool ReadPC(AnyCollection& msg,File& datastream);

  ros::Time startTime;
  ros::Publisher rgb_pub, depth_pub;
  ros::Publisher pc_pub;
  sensor_msgs::Image rgb_msg,depth_msg;
  sensor_msgs::PointCloud2 pc_msg;
  vector<char> pc_data;
  int frames_read;
  int bytes_read;
};


//process a header/data message and send it to the appropriate ros topic
bool Loop::ReadAndProcess(File& file)
{
  AnyCollection header;
  string msg;
  if(!ReadIntPrependedString(file,msg)) {
    return false;
  }
  bytes_read += msg.size();
  frames_read += 1;

  stringstream ss(msg);
  ss>>header;
  if(!ss) {
    printf("Error parsing header from string \"%s\"\n",msg.c_str());
    return false;
  }
  string type,name;
  if(!header["type"].as(type)) {
    printf("Header doesn't have \"type\" field\n");
    return false;
  }
  if(type == "Image") {
    if(!header["name"].as(name)) {
      printf("Header doesn't have \"name\" field\n");
      return false;
    }
    if(name == "rgb") {
      if(!ReadImage(header,file)) return false;
      rgb_pub.publish(rgb_msg);
    }
    else if(name == "depth") {
      if(!ReadImage(header,file)) return false;
      depth_pub.publish(depth_msg);
    }
    else {
      printf ("Unknown image %s\n",name.c_str());
      return false;
    }
  }
  else if(type == "PointCloud3D") {
    if(!ReadPC(header,file)) return false;
    pc_pub.publish(pc_msg);
  }
  else {
    printf("Unknown type %s\n",type.c_str());
    return false;
  }
  return true;
}

bool Loop::ReadImage(AnyCollection& msg,File& datastream)
{
  Timer timer;
  sensor_msgs::Image* rosmsg = NULL;
  string name;
  msg["name"].as(name);
  if(name=="rgb")
    rosmsg = &rgb_msg;
  else
    rosmsg = &depth_msg;
  rosmsg->header.seq = (int)msg["id"];
  rosmsg->header.stamp = startTime + ros::Duration(double(msg["time"]));
  rosmsg->width = msg["width"];
  rosmsg->height = msg["height"];
  string format;
  if(!msg["format"].as(format)) {
    printf("Image message doesn't contain \"format\"\n"); 
  }
  int pixelsize = 0;
  if(format == "rgb8") {
    pixelsize = 3;
    rosmsg->encoding = sensor_msgs::image_encodings::BGR8;
  }
  else if(format == "u16")  {
    pixelsize = 2;
    rosmsg->encoding = sensor_msgs::image_encodings::MONO16;
  }
  if(pixelsize == 0) {
    printf("Invalid format %s\n",format.c_str());
    return false;
  }
  rosmsg->is_bigendian = true;
  rosmsg->step = rosmsg->width*pixelsize;
  int size = rosmsg->width*rosmsg->height*pixelsize;
  rosmsg->data.resize(size);
  string buf;
  int cnt = 0;
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return false;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
      printf("Current count: %d\n",cnt);
      printf("Size: %d\n",size);
      return false;
    }
    //printf("%d\n",buf.length());
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return false;
    }
    copy(buf.begin(),buf.end(),&rosmsg->data[cnt]);
    cnt += buf.length();
  }
  bytes_read += cnt;
  //printf("ReadImage: %gs\n",timer.ElapsedTime());
  return true;
}

bool Loop::ReadPC(AnyCollection& msg,File& datastream)
{
  Timer timer;
  pc_msg.header.frame_id = "/realsense";
  pc_msg.header.seq = (int)msg["id"];
  pc_msg.header.stamp = startTime + ros::Duration(double(msg["time"]));
  pc_msg.width=(int)msg["width"];
  pc_msg.height=(int)msg["height"];
  string properties,format;
  if(!msg["properties"].as(properties) || !msg["format"].as(format)) {
    printf("PointCloud3D message doesn't have properties or format items\n");
    return false;
  }
  if(properties != "xyzrgb") {
    printf("TODO: parse non-xyzrgb point clouds\n");
    return false;
  }
  if(format != "fffccc" && format != "sssccc") {
    printf("TODO: parse non fffccc or sssccc format point clouds\n");
    return false;
  }
  pc_msg.fields.resize(6);
  pc_msg.fields[0].name="x";
  pc_msg.fields[0].offset = 0;
  pc_msg.fields[0].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[0].count = 1;
  pc_msg.fields[1].name="y";
  pc_msg.fields[1].offset = sizeof(float);
  pc_msg.fields[1].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[1].count = 1;
  pc_msg.fields[2].name="z";
  pc_msg.fields[2].offset = sizeof(float)*2;
  pc_msg.fields[2].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[2].count = 1;

  //packed RGB
  pc_msg.fields[3].name="rgb";
  pc_msg.fields[3].offset = sizeof(float)*3;
  pc_msg.fields[3].datatype = sensor_msgs::PointField::FLOAT32;
  pc_msg.fields[3].count = 1;

  //packed RGB
  pc_msg.point_step = sizeof(float)*3+4;
  pc_msg.row_step = pc_msg.point_step*pc_msg.width;
  pc_msg.is_bigendian = IsBigEndian();
  pc_msg.is_dense = false;
  pc_msg.data.resize(pc_msg.row_step*pc_msg.height);

  int inpixelsize;
  if(format == "fffccc")
    inpixelsize = 3*4+3;
  else
    inpixelsize = 3*2+3;
  int size = pc_msg.width*pc_msg.height*inpixelsize;
  pc_data.resize(size);
  string buf;
  int cnt = 0;
  while(cnt < size) {
    if(!ReadIntPrependedString(datastream,buf)) {
      printf("Error reading string during data portion\n");
      return false;
    }
    if(buf.length() != packet_size && cnt+buf.length() != size) {
      printf("Warning, abnormal buffer size %d\n",(int)buf.length());
    }
    if(cnt + buf.length() > size) {
      printf("Weird, sum of streamed message lengths is too large: %d vs %d\n",cnt+(int)buf.length(),size);
      printf("Buf size: %d\n",(int)buf.length());
      return false;
    }
    copy(buf.begin(),buf.end(),&pc_data[cnt]);
    cnt += buf.length();
  }
  bytes_read += cnt;
  //printf("ReadPC: %gs\n",timer.ElapsedTime());
  int k=0;
  if(format == "fffccc") {
    for(int i=0;i<pc_msg.width;i++) {
      for(int j=0;j<pc_msg.height;j++,k++) {
        //copy the k'th pixel
        char* indata = &pc_data[inpixelsize*k];
        unsigned char* outdata = &pc_msg.data[pc_msg.point_step*k];
        memcpy(outdata,indata,sizeof(float)*3);
        unsigned char b=*(unsigned char*)(indata+12);
        unsigned char g=*(unsigned char*)(indata+13);
        unsigned char r=*(unsigned char*)(indata+14);
        unsigned int rgb =  (r << 16) | (g<<8) | b;
        *(unsigned int*)(outdata+12) = rgb;
      }
    }
  }
  else {
    assert(format == "sssccc");
    float scale;
    if(!msg["scale"].as(scale)) scale = 1.0;
    for(int i=0;i<pc_msg.width;i++) {
      for(int j=0;j<pc_msg.height;j++,k++) {
        //copy the k'th pixel
        char* indata = &pc_data[inpixelsize*k];
        unsigned char* outdata = &pc_msg.data[pc_msg.point_step*k];
        short x=*(short*)(indata+0);
        short y=*(short*)(indata+2);
        short z=*(short*)(indata+4);
        unsigned char b=*(unsigned char*)(indata+6);
        unsigned char g=*(unsigned char*)(indata+7);
        unsigned char r=*(unsigned char*)(indata+8);
        *(float*)(outdata+0) = float(x)*scale;
        *(float*)(outdata+4) = float(y)*scale;
        *(float*)(outdata+8) = float(z)*scale;
        unsigned int rgb = (r << 16) | (g<<8) | b;
        *(unsigned int*)(outdata+12) = rgb;
      }
    }
  }
  return true;
}



int main(int argc,char** argv)
{
  const char* addr = "tcp://192.168.1.128:3457";
  const char* rostopic_prefix = "/realsense";
  if(argc <= 1) {
    printf("USAGE: RealSense_ROS_Emitter tcp://SERVER_IP_ADDR:PORT\n");
    printf("   Running on %s by default\n",addr);
    printf("   Publishing to %s/{rgb,depth,pc} by default\n",rostopic_prefix);
  }
  if(argc >= 2) {
    addr = argv[1];
    printf("   Reading from addr %s\n",addr);
  }
  if(argc >= 3) {
    rostopic_prefix = argv[2];
    printf("   Publishing to %s/{rgb,depth,pc}\n",rostopic_prefix);
    ros_rgb_topic  = string(rostopic_prefix)+"/rgb";
    ros_depth_topic  = string(rostopic_prefix)+"/depth";
    ros_pc_topic  = string(rostopic_prefix)+"/pc";
  }
  File f;
  if(!f.Open(addr,FILEREAD|FILEWRITE)) {
    printf("Unable to open client to %s... did you run \"RealSenseClient %s\"?\n",addr,addr);
    return 1;
  }
  //initialize ROS and publish messages
  string nodename = string("RealSense_ROS_Emitter_")+string(rostopic_prefix);
  ros::init(argc, argv, nodename.c_str(), ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  Loop loop(nh);
  Timer timer;
  double lastPrintTime = 0.0;
  while(ros::ok()) {
    if(!loop.ReadAndProcess(f)) {
      printf("Abnormal termination\n");
      return 1;
    }
    if(timer.ElapsedTime() > lastPrintTime + 1.0) {
      double t = timer.ElapsedTime();
      cout<<"Read rate: "<<double(loop.bytes_read)/(t-lastPrintTime)/1024/1024<<"mb/s, "<<float(loop.frames_read)/(t-lastPrintTime)<<" images/s"<<endl;
      loop.bytes_read = 0;
      loop.frames_read = 0;
      lastPrintTime = t;
    }
  }
  printf("Terminated due to ros::ok\n");
  return 0;
}
