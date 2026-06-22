#include <gz/transport/Node.hh>
#include <gz/msgs/pointcloud_packed.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/msgs/contacts.pb.h>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/stringmsg.pb.h>

#include <iostream>
#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>
#include <mutex>
#include <chrono>
#include <thread>
#include <csignal>
#include <atomic>
#include <cstring>

static std::atomic<bool> g_running{true};
void sigHandler(int) { g_running = false; }

class VFHController {
public:
    gz::transport::Node node;
    gz::transport::Node::Publisher thrustPub, vertFinPub, horizFinPub, resultPub;
    std::mutex mtx;
    std::string droneNs = "tethys_0";

    double posX=0, posY=0, posZ=0, yaw=0, pitch=0, roll=0;
    double goalX=0, goalY=0, goalZ=0;

    enum class EpisodeState { RUNNING, ARRIVED, COLLISION, TIMEOUT };
    EpisodeState episodeState = EpisodeState::RUNNING;
    bool resultPublished = false;
    int64_t maxIterations = 200000;
    int64_t iterCount = 0;
    double simTimeSec=0, pathLength=0;
    double prevX=0, prevY=0, prevZ=0;
    bool pathStarted = false;

    double gainSteer=0.8, gainPitch=0.8, maxFinAngle=0.15, radiusArrived=6.0;
    float r_min=2.5f, r_max=60.0f;
    int n_r=60; float delta_r=1.0f;
    float A=10.88f, B=0.0f, C_MAX=14.625f;
    float r_active=50.0f; int n_r_active=50;
    std::vector<float> grid_c, c_star;
    float gridDecay=0.982f;
    int n_phi=36, n_theta=36;
    float delta_phi=M_PI/36, delta_theta=M_PI/36;
    float phi_min=-M_PI/2, phi_max=M_PI/2;
    float theta_min=-M_PI/2, theta_max=M_PI/2;
    int L=5; float VALLEY_THRESHOLD=34.4f;
    double measurements[36][36]={};
    float h[36][36]={}, h_smooth[36][36]={};
    int k_targ=18, k_targ_theta=18, best_phi=18, best_theta=18;
    int safetyWindow=2;

    inline float& gc(int r,int p,int t){return grid_c[r*n_phi*n_theta+p*n_theta+t];}
    inline float& cs(int r,int p,int t){return c_star[r*n_phi*n_theta+p*n_theta+t];}

    void setup(const std::string& ns, double gx, double gy, double gz_,
               double gs, double gp, double vt, double gd, double ma, int sl, int sw) {
        droneNs=ns; goalX=gx; goalY=gy; goalZ=gz_;
        gainSteer=gs; gainPitch=gp; VALLEY_THRESHOLD=vt;
        gridDecay=gd; A=ma; B=ma/r_max; L=sl; safetyWindow=sw;
        n_r=(int)(r_max/delta_r); n_r_active=(int)(r_active/delta_r);
        grid_c.assign(n_r*n_phi*n_theta,0.0f);
        c_star.assign(n_r_active*n_phi*n_theta,0.0f);

        node.Subscribe("/"+ns+"/lidar/points",   &VFHController::OnPointCloud,this);
        node.Subscribe("/world/empty_environment/dynamic_pose/info",&VFHController::OnPose,this);
        node.Subscribe("/world/empty_environment/model/"+ns+"/link/base_link/sensor/contact_sensor/contact",
                       &VFHController::OnContact,this);

        thrustPub  =node.Advertise<gz::msgs::Double>("/model/"+ns+"/joint/propeller_joint/cmd_thrust");
        vertFinPub =node.Advertise<gz::msgs::Double>("/model/"+ns+"/joint/vertical_fins_joint/0/cmd_pos");
        horizFinPub=node.Advertise<gz::msgs::Double>("/model/"+ns+"/joint/horizontal_fins_joint/0/cmd_pos");
        resultPub  =node.Advertise<gz::msgs::StringMsg>("/"+ns+"/es/episode_result");

        std::cout<<"[VFH] ns="<<ns<<" goal=("<<gx<<","<<gy<<","<<gz_<<")"<<std::endl;
    }

    void OnPointCloud(const gz::msgs::PointCloudPacked& _msg) {
        std::lock_guard<std::mutex> lock(mtx);
        if(episodeState!=EpisodeState::RUNNING)return;
        std::memset(measurements,0,sizeof(measurements));
        int step=_msg.point_step(), total=_msg.width()*_msg.height();
        const char* data=_msg.data().data();
        for(int i=0;i<total;i++){
            const char* base=data+i*step;
            float x,y,z;
            std::memcpy(&x,base+0,4); std::memcpy(&y,base+4,4); std::memcpy(&z,base+8,4);
            if(std::isinf(x)||std::isinf(y)||std::isinf(z))continue;
            double r=std::sqrt(x*x+y*y+z*z);
            double phi=std::atan2(y,x);
            double theta=std::atan2(z,std::sqrt(x*x+y*y));
            if(r<r_min||r>r_max)continue;
            int pi=(int)((phi_max-phi)/delta_phi);
            int ti=(int)((theta-theta_min)/delta_theta);
            if(pi<0||pi>=n_phi)continue;
            if(ti<0||ti>=n_theta)continue;
            measurements[pi][ti]=(float)r;
        }
        updateGrid(); extractActive(); buildHistogram(); smoothHistogram(); findBest();
    }

    void updateGrid(){
        for(int r=0;r<n_r;r++) for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++)
            gc(r,p,t)*=gridDecay;
        for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++){
            float m=measurements[p][t]; if(m<=0||m<r_min)continue;
            int ri=(int)(m/delta_r); if(ri<0||ri>=n_r)continue;
            float mag=A-B*m; if(mag<0)continue;
            gc(ri,p,t)+=mag; if(gc(ri,p,t)>C_MAX)gc(ri,p,t)=C_MAX;
        }
    }

    void extractActive(){
        std::fill(c_star.begin(),c_star.end(),0.0f);
        for(int r=0;r<n_r_active;r++) for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++)
            cs(r,p,t)=gc(r,p,t);
    }

    void buildHistogram(){
        std::memset(h,0,sizeof(h));
        for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++) for(int r=0;r<n_r_active;r++)
            h[p][t]+=cs(r,p,t);
    }

    void smoothHistogram(){
        std::memset(h_smooth,0,sizeof(h_smooth));
        for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++){
            float sum=0; int cnt=0;
            for(int dp=-L;dp<=L;dp++) for(int dt=-L;dt<=L;dt++){
                int pp=p+dp, tt=t+dt;
                if(pp<0||pp>=n_phi)continue; if(tt<0||tt>=n_theta)continue;
                int w=(std::abs(dp)==L)?1:2; w*=(std::abs(dt)==L)?1:2;
                sum+=w*h[pp][tt]; cnt+=w;
            }
            h_smooth[p][t]=(cnt>0)?sum/cnt:0.0f;
        }
    }

    void findBest(){
        // Goal sectors
        double dx=goalX-posX, dy=goalY-posY;
        double phi_goal=std::atan2(dy,dx)-yaw-M_PI;
        while(phi_goal>M_PI)phi_goal-=2*M_PI;
        while(phi_goal<-M_PI)phi_goal+=2*M_PI;
        phi_goal=std::clamp(phi_goal,(double)phi_min,(double)phi_max);
        k_targ=std::clamp((int)((phi_max-phi_goal)/delta_phi),0,n_phi-1);
        double dxy=std::sqrt(dx*dx+dy*dy), dz=goalZ-posZ;
        double tg=std::atan2(dz,dxy);
        while(tg>M_PI)tg-=2*M_PI; while(tg<-M_PI)tg+=2*M_PI;
        tg=std::clamp(tg,(double)theta_min,(double)theta_max);
        k_targ_theta=std::clamp((int)((tg-theta_min)/delta_theta),0,n_theta-1);

        // Best direction
        best_phi=k_targ; best_theta=k_targ_theta;
        float best_cost=std::numeric_limits<float>::max();
        int W=safetyWindow;
        for(int p=0;p<n_phi;p++) for(int t=0;t<n_theta;t++){
            bool free=true;
            for(int dp=-W;dp<=W&&free;dp++) for(int dt=-W;dt<=W&&free;dt++){
                int pp=p+dp, tt=t+dt;
                if(pp<0||pp>=n_phi)continue; if(tt<0||tt>=n_theta)continue;
                if(h_smooth[pp][tt]>=VALLEY_THRESHOLD)free=false;
            }
            if(free){
                float c=std::sqrt((float)((p-k_targ)*(p-k_targ)+(t-k_targ_theta)*(t-k_targ_theta)*4));
                if(c<best_cost){best_cost=c; best_phi=p; best_theta=t;}
            }
        }

        double dist=std::sqrt(dx*dx+dy*dy+dz*dz);
        if(dist<radiusArrived&&episodeState==EpisodeState::RUNNING){
            episodeState=EpisodeState::ARRIVED;
            std::cout<<"[VFH] ARRIVED! dist="<<dist<<std::endl;
        }
        if(episodeState!=EpisodeState::RUNNING){publishResult();return;}

        double pb=phi_max-best_phi*delta_phi;
        double tb=theta_min+best_theta*delta_theta;
        gz::msgs::Double mh,mv;
        mh.set_data(std::clamp(pb*gainSteer,-maxFinAngle,maxFinAngle));
        mv.set_data(std::clamp(-tb*gainPitch,-maxFinAngle,maxFinAngle));
        vertFinPub.Publish(mh); horizFinPub.Publish(mv);
    }

    void publishResult(){
        if(resultPublished)return; resultPublished=true;
        std::string r;
        switch(episodeState){
            case EpisodeState::ARRIVED:   r="arrived";   break;
            case EpisodeState::COLLISION: r="collision"; break;
            default:                      r="timeout";   break;
        }
        double dx=goalX-posX,dy=goalY-posY,dz=goalZ-posZ;
        gz::msgs::StringMsg msg;
        msg.set_data(r+";t="+std::to_string(simTimeSec)
                      +";path="+std::to_string(pathLength)
                      +";dist="+std::to_string(std::sqrt(dx*dx+dy*dy+dz*dz)));
        resultPub.Publish(msg);
        gz::msgs::Double z; z.set_data(0.0);
        thrustPub.Publish(z); vertFinPub.Publish(z); horizFinPub.Publish(z);
        g_running=false;
    }

    void OnPose(const gz::msgs::Pose_V& _msg){
        std::lock_guard<std::mutex> lock(mtx);
        for(int i=0;i<_msg.pose_size();i++){
            if(_msg.pose(i).name()!=droneNs)continue;
            auto& p=_msg.pose(i).position();
            auto& o=_msg.pose(i).orientation();
            posX=p.x(); posY=p.y(); posZ=p.z();
            double ox=o.x(),oy=o.y(),oz=o.z(),ow=o.w();
            yaw=std::atan2(2*(ow*oz+ox*oy),1-2*(oy*oy+oz*oz));
            if(episodeState==EpisodeState::RUNNING){
                if(pathStarted)
                    pathLength+=std::sqrt((posX-prevX)*(posX-prevX)+
                                         (posY-prevY)*(posY-prevY)+
                                         (posZ-prevZ)*(posZ-prevZ));
                prevX=posX; prevY=posY; prevZ=posZ; pathStarted=true;
            }
        }
    }

    void OnContact(const gz::msgs::Contacts& _msg){
        std::lock_guard<std::mutex> lock(mtx);
        if(_msg.contact_size()>0&&episodeState==EpisodeState::RUNNING){
            episodeState=EpisodeState::COLLISION;
            std::cout<<"[VFH] COLLISION!"<<std::endl;
            publishResult();
        }
    }

    void run(){
        auto start=std::chrono::steady_clock::now();
        while(g_running&&episodeState==EpisodeState::RUNNING){
            gz::msgs::Double t; t.set_data(-31.0);
            thrustPub.Publish(t);
            auto now=std::chrono::steady_clock::now();
            simTimeSec=std::chrono::duration<double>(now-start).count();
            iterCount+=100;
            if(iterCount>maxIterations){
                episodeState=EpisodeState::TIMEOUT;
                publishResult();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
};

int main(int argc, char** argv){
    signal(SIGINT,sigHandler); signal(SIGTERM,sigHandler);
    if(argc<5){
        std::cerr<<"Uso: "<<argv[0]<<" <ns> <goal_x> <goal_y> <goal_z>"<<std::endl;
        return 1;
    }
    VFHController ctrl;
    ctrl.setup(argv[1],std::stod(argv[2]),std::stod(argv[3]),std::stod(argv[4]),
               0.8,0.8,34.4,0.982,10.88,5,2);
    std::cout<<"[VFH] In attesa di dati sonar..."<<std::endl;
    ctrl.run();
    return 0;
}