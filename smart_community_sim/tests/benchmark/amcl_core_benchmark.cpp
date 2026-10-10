// Exercises the installed ROS AMCL libraries without starting ROS or Gazebo.
#include <amcl/sensors/amcl_laser.h>
#include <amcl/sensors/amcl_odom.h>
#include <amcl/pf/pf.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>

static pf_vector_t random_pose(void* ptr) {
  auto* map=static_cast<map_t*>(ptr);pf_vector_t pose;
  while (true) {
    const int x=static_cast<int>(drand48()*map->size_x);
    const int y=static_cast<int>(drand48()*map->size_y);
    if(map->cells[MAP_INDEX(map,x,y)].occ_state!=-1)continue;
    pose.v[0]=MAP_WXGX(map,x);pose.v[1]=MAP_WYGY(map,y);
    pose.v[2]=(2*drand48()-1)*M_PI;return pose;
  }
}
static double angle(double a){return std::atan2(std::sin(a),std::cos(a));}
int main(int argc,char** argv) {
  if(argc!=7){std::cerr<<"dataset output seed beams min_particles max_particles\n";return 2;}
  std::ifstream input(argv[1]);std::ofstream out(argv[2]);
  if(!input || !out)return 2;
  srand48(std::stoi(argv[3]));
  map_t* map=map_alloc();double ox,oy;
  input>>map->size_x>>map->size_y>>map->scale>>ox>>oy;
  map->origin_x=ox+map->size_x/2*map->scale;map->origin_y=oy+map->size_y/2*map->scale;
  map->cells=static_cast<map_cell_t*>(calloc(map->size_x*map->size_y,sizeof(map_cell_t)));
  for(int i=0;i<map->size_x*map->size_y;++i)input>>map->cells[i].occ_state;
  int frames,beams;input>>frames>>beams;
  pf_t* pf=pf_alloc(std::stoi(argv[5]),std::stoi(argv[6]),.001,.1,random_pose,map);
  srand48(std::stoi(argv[3]));
  pf->pop_err=.03;pf->pop_z=.99;
  amcl::AMCLOdom motion;motion.SetModel(amcl::ODOM_MODEL_OMNI_CORRECTED,.12,.12,.15,.12,.15);
  amcl::AMCLLaser laser(std::stoi(argv[4]),map);laser.SetModelLikelihoodField(.95,.05,.05,.6);
  pf_vector_t offset=pf_vector_zero();laser.SetLaserPose(offset);
  pf_vector_t last=pf_vector_zero();
  out<<"time,true_x,true_y,true_yaw,odom_x,odom_y,odom_yaw,est_x,est_y,est_yaw,update_ms,particles\n";
  for(int frame=0;frame<frames;++frame) {
    double time;pf_vector_t truth,odometry;input>>time;
    for(int k=0;k<3;++k)input>>truth.v[k];
    for(int k=0;k<3;++k)input>>odometry.v[k];
    amcl::AMCLLaserData scan;scan.sensor=&laser;scan.range_count=beams;scan.range_max=8.;scan.ranges=new double[beams][2];
    for(int k=0;k<beams;++k){input>>scan.ranges[k][0];scan.ranges[k][1]=-M_PI+2*M_PI*k/(beams-1);}
    if(!input){std::cerr<<"Invalid dataset at frame "<<frame<<"\n";return 3;}
    const auto begin=std::chrono::steady_clock::now();
    amcl::AMCLOdomData data;data.sensor=&motion;data.pose=odometry;
    for(int k=0;k<3;++k)data.delta.v[k]=odometry.v[k]-last.v[k];
    data.delta.v[2]=angle(data.delta.v[2]);
    if(frame==0){
      auto initial=truth;initial.v[0]+=.08;initial.v[1]-=.06;initial.v[2]=angle(initial.v[2]+.06);
      auto covariance=pf_matrix_zero();covariance.m[0][0]=covariance.m[1][1]=.01;covariance.m[2][2]=.03;
      pf_init(pf,initial,covariance);
    }
    if(frame==0 || std::abs(data.delta.v[0])>.03 || std::abs(data.delta.v[1])>.03 || std::abs(data.delta.v[2])>.06){
      if(frame>0)motion.UpdateAction(pf,&data);
      laser.UpdateSensor(pf,&scan);pf_update_resample(pf);last=odometry;
    }
    auto* set=&pf->sets[pf->current_set];pf_vector_t mean=set->mean;double best=-1;
    for(int k=0;k<set->cluster_count;++k)if(set->clusters[k].weight>best){best=set->clusters[k].weight;mean=set->clusters[k].mean;}
    // AMCL's map->odom stays fixed between filter updates; apply subsequent odometry.
    const double rotation=angle(mean.v[2]-last.v[2]);
    const double dx=odometry.v[0]-last.v[0],dy=odometry.v[1]-last.v[1];
    mean.v[0]+=std::cos(rotation)*dx-std::sin(rotation)*dy;
    mean.v[1]+=std::sin(rotation)*dx+std::cos(rotation)*dy;
    mean.v[2]=angle(mean.v[2]+angle(odometry.v[2]-last.v[2]));
    const double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-begin).count();
    out<<time;
    for(double v:truth.v)out<<','<<v;
    for(double v:odometry.v)out<<','<<v;
    for(double v:mean.v)out<<','<<v;
    out<<','<<ms<<','<<set->sample_count<<'\n';
  }
  pf_free(pf);map_free(map);return 0;
}
