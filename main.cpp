#include <iostream>
#include <string>
#include <math.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
using namespace std;
typedef pcl::PointXYZ PoinyT;
typedef pcl::PointCloud<PoinyT> PointCloudT;
bool select_flag=0;
bool open_flag=0;
PointCloudT::Ptr cloud_in(new PointCloudT());


void ppa_callback(const pcl::visualization::AreaPickingEvent& event,void *args)
{
  struct callback_args * data=(struct callback_args *)args;
  pcl::visualization::PCLVisualizer part_viewr("cut map");
  PointCloudT::Ptr cut_point(new PointCloudT());
  std::vector<int> indices;
  if(event.getPointsIndices(indices)==-1)
  {
    return;
  }
  cut_point->width=1;
  cut_point->height=indices.size();
  for(size_t i=0;i<indices.size();i++)
  {
    cut_point->points.push_back(cloud_in->at(indices[i]));
  }
 
  part_viewr.addPointCloud(cut_point,"cut_map");
  pcl::io::savePCDFileASCII("part2.pcd",*cut_point);
  
  open_flag=1;
  
}
void keyboardEventOccurred(const pcl::visualization::KeyboardEvent& event,void* nothing)
{
  if(event.keyDown())
  {
    if(event.getKeySym()=="space")
    {
      select_flag=1-select_flag;
    }
    if((event.getKeySym()=="C"||event.getKeySym()=="c")&&open_flag)
    {
      open_flag=0;
    }
  }
}
int main(int argc, char **argv) {
  if(argc!=2)
  {
    cerr<<"please input the pointcloud file"<<endl;
    return -1;
  }
 
  pcl::console::TicToc time;
  time.tic();
  if(pcl::io::loadPCDFile(argv[1],*cloud_in)<0)
  {
    PCL_ERROR("Error load cloud");
    return -1;
  }
  cout<<"Load cloud file:"<<argv[1]<<"(inlcude "<<cloud_in->size()<<" points) in"<<time.toc()<<" ms\n"<<endl;
  pcl::visualization::PCLVisualizer viewer ("map_cut_demo");
  pcl::visualization::PointCloudColorHandlerCustom <PoinyT> cloud_color_h (cloud_in,230,20,20);
  viewer.addPointCloud(cloud_in,cloud_color_h,"map1");
  viewer.setBackgroundColor(0,0,0);
  viewer.setSize(1280,1024);
  viewer.registerAreaPickingCallback(ppa_callback,(void*)&cloud_in);
  viewer.registerKeyboardCallback(&keyboardEventOccurred,(void*)NULL);
  while(!viewer.wasStopped())
  {
    viewer.spinOnce();
  }
  
  
}
