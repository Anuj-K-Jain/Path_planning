#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>
#include <limits>
using namespace std;
int N=1000;
pcl::PointCloud<pcl::PointXYZ>::Ptr obst_cloud (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr rrg_cloud (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<double> rrg_bubble;
pcl::KdTreeFLANN<pcl::PointXYZ> obst_kdtree;
pcl::KdTreeFLANN<pcl::PointXYZ> rrg_kdtree;
pcl::PointXYZ start, infi;
bool graph[1000][1000];
double dist(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
  double d=sqrt(pow((p1.x-p2.x),2)+pow((p1.y-p2.y),2)+pow((p1.z-p2.z),2));
  return d;
}
void set_graph()
{
  for(int i=0;i<N;i++)
    for(int j=0;j<N;j++)
      graph[i][j]=0;
}
void set_obst()
{
  // Generate pointcloud data
  srand (time (NULL));
  obst_cloud->width = 1000;
  obst_cloud->height = 1;
  obst_cloud->points.resize (obst_cloud->width * obst_cloud->height);

  for (size_t i = 0; i < obst_cloud->points.size (); ++i)
  {
    obst_cloud->points[i].x = 1024.0f * rand () / (RAND_MAX + 1.0f);
    obst_cloud->points[i].y = 1024.0f * rand () / (RAND_MAX + 1.0f);
    obst_cloud->points[i].z = 1024.0f * rand () / (RAND_MAX + 1.0f);
  }
  obst_kdtree.setInputCloud (obst_cloud);

  start.x=start.y=start.z=0;
}
double bubble_radius(pcl::PointXYZ p1)
{
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;
  if ( obst_kdtree.nearestKSearch (p1, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 ){
  cout<<"bubble radius completed"<<endl;
    return sqrt(pointNKNSquaredDistance[0]);
  }
  return 0;
}
bool CONNECT(int index)
{
  rrg_bubble[index]=bubble_radius(rrg_cloud->points[index]);/*
  for(int i=0;i<index;i++)
    if(dist(rrg_cloud->points[index],rrg_cloud->points[i])<rrg_bubble[i])
    {
      graph[i][index]=1;
      graph[index][i]=1;
    }*/
  std::vector<int> pointIdxRadiusSearch;
  std::vector<float> pointRadiusSquaredDistance;
  cout<<"entered CONNECT --------"<<endl;
  cout<<" rrg_bubble[index]= " <<rrg_bubble[index]<<endl;
  if(rrg_kdtree.radiusSearch (rrg_cloud->points[index], rrg_bubble[index], pointIdxRadiusSearch, pointRadiusSquaredDistance) > 0 )
  { 
    cout<<"entered for inside CONNECT --------"<<endl; 
    for(int i=0;i<pointIdxRadiusSearch.size();i++)
    {  
      graph[pointIdxRadiusSearch[i]][index]=1;
      graph[index][pointIdxRadiusSearch[i]]=1;
    }
    cout<<"CONNECTED"<<endl;
    return 1;}
  else
    cout<<"error in CONNECT"<<endl;
  return 0;
}
void init_rrg()
{
  rrg_cloud->width = N;
  rrg_cloud->height = 1;
  rrg_cloud->points.resize (rrg_cloud->width * rrg_cloud->height);
  rrg_cloud->points.resize (rrg_cloud->width * rrg_cloud->height);
  rrg_cloud->points[0].x=0;
  rrg_cloud->points[0].y=0;
  rrg_cloud->points[0].z=0;  
  infi.x=std::numeric_limits<float>::infinity();
  infi.y=std::numeric_limits<float>::infinity();
  infi.z=std::numeric_limits<float>::infinity();
  for(int i=1;i<N;i++)
  {
    rrg_cloud->points[i]=infi;
  }
  cout<<dist(rrg_cloud->points[0],rrg_cloud->points[1]);
  cout<<"rrg initialized"<<endl;
}
int main (int argc, char** argv)
{
  init_rrg();
  set_obst();
  cout<<"obstacles set"<<endl;
  set_graph();
  cout<<"graph set"<<endl;
  int K=10;
  std::vector<int> pointIdxNKNSearch;
  std::vector<float> pointNKNSquaredDistance;

  rrg_bubble.push_back(bubble_radius(rrg_cloud->points[0]));
  cout<<"start point set"<<endl;
  for(int i=1;i<N;i++)
  {
    pointIdxNKNSearch.clear();
    pointNKNSquaredDistance.clear();    
    rrg_kdtree.setInputCloud (rrg_cloud);
    pcl::PointXYZ searchPoint;
    searchPoint.x= 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.y= 1024.0f * rand () / (RAND_MAX + 1.0f);
    searchPoint.z= 1024.0f * rand () / (RAND_MAX + 1.0f);

    cout<<"search point set for i="<<i<<endl;
    if ( rrg_kdtree.nearestKSearch (searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      cout<<"entered first if "<<endl;
      cout<<"pointIdxNKNSearch[0]= "<<pointIdxNKNSearch[0]<<endl;
    cout<<"rrg_cloud->points[pointIdxNKNSearch[0]]= "<<rrg_cloud->points[pointIdxNKNSearch[0]]<<endl;
    cout<<"rrg_bubble[pointIdxNKNSearch[0]]= "<<rrg_bubble[pointIdxNKNSearch[0]]<<endl;
    cout<<"bubble_radius(searchPoint)= "<<bubble_radius(searchPoint);
      if(dist(searchPoint,rrg_cloud->points[pointIdxNKNSearch[0]])<max(rrg_bubble[pointIdxNKNSearch[0]],bubble_radius(searchPoint)))
      {

  cout<<"entered second if"<<endl;
        rrg_cloud->points[i]=searchPoint;
        if(!CONNECT(i)){
          i--;
          rrg_cloud->points[i]=infi;
        }
      }
      else
      {
  cout<<"entered second else"<<endl;
        pcl::PointXYZ p1;
        p1.x=rrg_cloud->points[pointIdxNKNSearch[0]].x+rrg_bubble[pointIdxNKNSearch[0]]*(searchPoint.x-rrg_cloud->points[pointIdxNKNSearch[0]].x)/dist(searchPoint,rrg_cloud->points[pointIdxNKNSearch[0]]);
      
        p1.y=rrg_cloud->points[pointIdxNKNSearch[0]].y+rrg_bubble[pointIdxNKNSearch[0]]*(searchPoint.y-rrg_cloud->points[pointIdxNKNSearch[0]].y)/dist(searchPoint,rrg_cloud->points[pointIdxNKNSearch[0]]);
      
        p1.z=rrg_cloud->points[pointIdxNKNSearch[0]].z+rrg_bubble[pointIdxNKNSearch[0]]*(searchPoint.z-rrg_cloud->points[pointIdxNKNSearch[0]].z)/dist(searchPoint,rrg_cloud->points[pointIdxNKNSearch[0]]);
        rrg_cloud->points[i]=p1;

  cout<<"trying to connect"<<endl;
        
        if(!CONNECT(i)){
          i--;
          rrg_cloud->points[i]=infi;
        }
      }
    }
    else
    {
      cout<<"FUCKED UPPPP !"<<endl;
      break;
    }
    cout<<endl;
  }

    for(int i=0;i<N;i++){
      for(int j=0;j<N;j++)
        cout<<graph[i][j]<<"\t";
      cout<<endl;}
  
return 0;
}

/*void obstacles_within(pcl::PointXYZ A, pcl::PointXYZ B,std::vector<int> *pointIdxRadiusSearch&)
{
  double d=dist(A,B);
  std::vector<int> pointIdxRadiusSearchA;
  std::vector<int> pointIdxRadiusSearchB;
  pointIdxRadiusSearch.clear();
  std::vector<float> pointRadiusSquaredDistanceA,pointRadiusSquaredDistanceB;

  if ( rrg_kdtree.radiusSearch (A, d, pointIdxRadiusSearchA, pointRadiusSquaredDistance) > 0 )
  {
    if ( rrg_kdtree.radiusSearch (B, d, pointIdxRadiusSearchB, pointRadiusSquaredDistance) > 0 )
    {
      int si=max(,);
      for(int i=0;i<pointIdxRadiusSearchA.size();i++)
      {
        for(int j=0;j<pointIdxRadiusSearchB.size();j++)
          if(pointIdxRadiusSearchA[i]==pointIdxRadiusSearchB[j])
            pointIdxRadiusSearch.push_back(pointIdxRadiusSearchA[i]);
      }
    }
  }
}*/