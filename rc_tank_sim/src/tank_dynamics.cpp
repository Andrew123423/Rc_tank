// BSD-3-Clause. Compliant pin constraints and homogenised sprocket engagement.
#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Joint.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/JointForceCmd.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/JointPositionReset.hh>
#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/double.pb.h>
#include <gz/msgs/int32.pb.h>
#include <gz/msgs/twist.pb.h>
#include <gz/msgs/clock.pb.h>
#include <gz/msgs/pose_v.pb.h>
#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/sim/components/Material.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <mutex>
#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <vector>
#include <string>
#include <iostream>

namespace rc_tank {
using V=gz::math::Vector3d;
using P=gz::math::Pose3d;
struct State {P p;V v,w;};
struct Pin {int a,b;std::array<V,2> ra,rb;};
class Dynamics: public gz::sim::System,public gz::sim::ISystemConfigure,public gz::sim::ISystemPreUpdate {
 gz::sim::Model model{gz::sim::kNullEntity};
 gz::transport::Node node;
 gz::transport::Node::Publisher trackPoses,closureError,clockPub;
 std::atomic<double> left{0},right{0};std::atomic<int> gear{1};std::atomic<unsigned long> serial{0};
 unsigned long seen=0;double lastCommand=-1,lastTime=-1,lastPublish=-1,lastClock=-1;
 bool heavy=false,ready=false;int n=107;
 double radius=.03487,sep=.347,a=.239,k=4000,d=1,timeout=.35,low=5.308235294117647,high=1.92,eff=.8,rpm=3000,power=200;
 gz::sim::Link hull;
 std::array<gz::sim::Link,2> sprockets;
 std::array<gz::sim::Entity,2> motorJoints{},idlerSlides{},frontSusp{};
 std::vector<gz::sim::Entity> lightWheels;
 std::vector<gz::sim::Link> shoes;
 std::vector<Pin> pins;
 std::mutex ledMutex;std::string ledHex=std::string(144,'0');bool ledDirty=true;
 gz::transport::Node::Publisher drivePub;
 gz::sim::Entity servo=gz::sim::kNullEntity;
 double param(const std::shared_ptr<const sdf::Element>&s,const std::string &key,double fallback){return s->Get<double>(key,fallback).first;}
 template<class C,class T>void set(gz::sim::Entity e,gz::sim::EntityComponentManager &ecm,const T&value){
   if(e==gz::sim::kNullEntity)return;
   auto c=ecm.Component<C>(e);if(c)c->Data()=value;else ecm.CreateComponent(e,C(value));
 }
 State read(gz::sim::Link &l,gz::sim::EntityComponentManager&ecm){return {gz::sim::worldPose(l.Entity(),ecm),l.WorldLinearVelocity(ecm).value_or(V::Zero),l.WorldAngularVelocity(ecm).value_or(V::Zero)};}
 static V limited(V v,double max){double len=v.Length();return len>max?v*(max/len):v;}
 void init(gz::sim::EntityComponentManager&ecm){
   if(!heavy){ready=true;return;}
   std::vector<State> st;for(auto &s:shoes)st.push_back(read(s,ecm));
   for(int side=0;side<2;++side){
     int i=n-1;
     int ia=side*n+i,ib=side*n+(i+1)%n;Pin pin;pin.a=ia;pin.b=ib;
     V centre=(st[ia].p.Pos()+st[ib].p.Pos())*.5;
     // Only the final/first hinge is compliant; all interior pins are native SDF revolute joints.
     for(int j=0;j<2;++j){V world=centre+read(hull,ecm).p.Rot().RotateVector(V(0,j? .018:-.018,0));
       pin.ra[j]=st[ia].p.Rot().RotateVectorReverse(world-st[ia].p.Pos());
       pin.rb[j]=st[ib].p.Rot().RotateVectorReverse(world-st[ib].p.Pos());}
     pins.push_back(pin);
   }
   ready=true;
 }
public:
 void Configure(const gz::sim::Entity&e,const std::shared_ptr<const sdf::Element>&s,gz::sim::EntityComponentManager&ecm,gz::sim::EventManager&)override{
   model=gz::sim::Model(e);heavy=s->Get<bool>("detailed",false).first;
   radius=param(s,"sprocket_radius",radius);sep=param(s,"track_separation",sep);a=param(s,"half_axle_spacing",a);
   n=static_cast<int>(param(s,"links_per_side",n));k=param(s,"hinge_stiffness",k);d=param(s,"hinge_damping",d);
   low=param(s,"gear_low",low);high=param(s,"gear_high",high);eff=param(s,"gear_efficiency",eff);
   timeout=param(s,"command_timeout",timeout);rpm=param(s,"motor_rpm",rpm);power=param(s,"motor_power_w",power);
   hull=gz::sim::Link(model.LinkByName(ecm,"base_link"));hull.EnableVelocityChecks(ecm);
   servo=model.JointByName(ecm,"shift_servo_joint");
   for(int side=0;side<2;++side){std::string sideName=side==0?"left":"right";
     sprockets[side]=gz::sim::Link(model.LinkByName(ecm,sideName+"_sprocket"));sprockets[side].EnableVelocityChecks(ecm);
     motorJoints[side]=model.JointByName(ecm,sideName+"_sprocket_joint");
     idlerSlides[side]=model.JointByName(ecm,sideName+"_tensioner_joint");
     frontSusp[side]=model.JointByName(ecm,sideName+"_suspension_0_joint");
     if(heavy){gz::sim::Joint(idlerSlides[side]).EnablePositionCheck(ecm);gz::sim::Joint(frontSusp[side]).EnablePositionCheck(ecm);
       for(int i=0;i<n;++i){gz::sim::Link l(model.LinkByName(ecm,sideName+"_shoe_"+std::to_string(i)));l.EnableVelocityChecks(ecm);shoes.push_back(l);}}
     else{
       lightWheels.push_back(motorJoints[side]);lightWheels.push_back(model.JointByName(ecm,sideName+"_idler_joint"));
       for(int i=0;i<3;++i)for(int j=0;j<2;++j)lightWheels.push_back(model.JointByName(ecm,sideName+"_road_"+std::to_string(i)+"_"+std::to_string(j)+"_joint"));
       for(int i=0;i<2;++i)lightWheels.push_back(model.JointByName(ecm,sideName+"_return_"+std::to_string(i)+"_joint"));
     }
   }
   node.Subscribe("/tank/motor_left",&Dynamics::onLeft,this);node.Subscribe("/tank/motor_right",&Dynamics::onRight,this);node.Subscribe("/tank/gear_internal",&Dynamics::onGear,this);
   node.Subscribe("/tank/led_colors",&Dynamics::onLeds,this);
   clockPub=node.Advertise<gz::msgs::Clock>("/tank/clock");
   drivePub=node.Advertise<gz::msgs::Twist>("/tank/drive_internal");
   trackPoses=node.Advertise<gz::msgs::Pose_V>("/tank/shoe_poses");
   closureError=node.Advertise<gz::msgs::Double>("/tank/closure_error");
 }
 void onLeds(const gz::msgs::StringMsg&m){
   if(m.data().size()!=144||m.data().find_first_not_of("0123456789abcdefABCDEF")!=std::string::npos)return;
   std::lock_guard<std::mutex> lock(ledMutex);if(ledHex!=m.data()){ledHex=m.data();ledDirty=true;}
 }
 void updateLeds(gz::sim::EntityComponentManager&ecm){
   std::lock_guard<std::mutex> lock(ledMutex);if(!ledDirty)return;ledDirty=false;
   ecm.Each<gz::sim::components::Name,gz::sim::components::ParentEntity,gz::sim::components::Material>(
     [&](const gz::sim::Entity &e,const gz::sim::components::Name *name,const gz::sim::components::ParentEntity *parent,gz::sim::components::Material *mat){
       if(parent->Data()!=hull.Entity()||name->Data().rfind("led_",0)!=0)return true;
       int i=std::stoi(name->Data().substr(4));if(i<0||i>=24)return true;
       double r=std::stoi(ledHex.substr(i*6,2),nullptr,16)/255.,g=std::stoi(ledHex.substr(i*6+2,2),nullptr,16)/255.,b=std::stoi(ledHex.substr(i*6+4,2),nullptr,16)/255.;
       gz::math::Color color(r,g,b,1);auto value=mat->Data();value.SetAmbient(color);value.SetDiffuse(color);value.SetEmissive(color);mat->Data()=value;
       ecm.SetChanged(e,gz::sim::components::Material::typeId,gz::sim::ComponentState::OneTimeChange);return true;});
 }
 void onLeft(const gz::msgs::Double&m){left=std::isfinite(m.data())?std::clamp(m.data(),-1.,1.):0.;++serial;}
 void onRight(const gz::msgs::Double&m){right=std::isfinite(m.data())?std::clamp(m.data(),-1.,1.):0.;++serial;}
 void onGear(const gz::msgs::Int32&m){if(m.data()==1||m.data()==2)gear=m.data();}
 void PreUpdate(const gz::sim::UpdateInfo&info,gz::sim::EntityComponentManager&ecm)override{
   double time=std::chrono::duration<double>(info.simTime).count();if(info.paused)return;
   // Reset command latch when the simulation clock moves backwards.
   if(time<lastTime){lastCommand=-1;left=0;right=0;seen=serial.load();lastPublish=-1;lastClock=-1;}
   lastTime=time;
   // Avoid bridging the world's per-iteration clock (up to 10 kHz in detailed mode).
   if(time-lastClock>=.02){gz::msgs::Clock clock;auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();clock.mutable_sim()->set_sec(ns/1000000000);clock.mutable_sim()->set_nsec(ns%1000000000);clockPub.Publish(clock);lastClock=time;}
   if(!ready)init(ecm);
   updateLeds(ecm);
   if(serial.load()!=seen){seen=serial.load();lastCommand=time;}
   bool active=lastCommand>=0 && time-lastCommand<timeout;
   std::array<double,2> duty={active?left.load():0.,active?right.load():0.};
   double ratio=gear.load()==1?low:high;
   double noLoad=rpm*2*M_PI/60/ratio;
   set<gz::sim::components::JointPositionReset>(servo,ecm,std::vector<double>{(gear.load()==1?40.:140.)*M_PI/180});
   if(!heavy){
     for(int side=0;side<2;++side){// Track surface commands and centres of rotation are supplied by TrackedVehicle.
       for(int j=0;j<10;++j){double rr=j<2?radius:(j<8?.0225:.015);set<gz::sim::components::JointVelocityCmd>(lightWheels[side*10+j],ecm,std::vector<double>{duty[side]*noLoad*radius/rr});}
     }
     if(time-lastPublish>=.019){gz::msgs::Twist drive;drive.mutable_linear()->set_x((duty[0]+duty[1])*.5*noLoad*radius);drive.mutable_angular()->set_z((duty[1]-duty[0])*noLoad*radius/sep);drivePub.Publish(drive);lastPublish=time;}
     return;
   }
   std::vector<State> st;st.reserve(shoes.size());for(auto &shoe:shoes)st.push_back(read(shoe,ecm));
   std::vector<V> forces(shoes.size(),V::Zero),torques(shoes.size(),V::Zero);
   double maxClosureError=0;
   for(const auto &pin:pins)for(int j=0;j<2;++j){
     const auto &A=st[pin.a],&B=st[pin.b];V ra=A.p.Rot().RotateVector(pin.ra[j]),rb=B.p.Rot().RotateVector(pin.rb[j]);
     V error=B.p.Pos()+rb-A.p.Pos()-ra;
     maxClosureError=std::max(maxClosureError,error.Length());
     V f=limited(k*error+d*(B.v+B.w.Cross(rb)-A.v-A.w.Cross(ra)),200.);
     forces[pin.a]+=f;torques[pin.a]+=ra.Cross(f);forces[pin.b]-=f;torques[pin.b]-=rb.Cross(f);
   }
   auto H=read(hull,ecm);
   for(int side=0;side<2;++side){auto S=read(sprockets[side],ecm);V axis=H.p.Rot().RotateVector(V::UnitY);
     double omega=(S.w-H.w).Dot(axis);
     // Linear DC torque-speed approximation, with current-limited stall torque.
     double nominal=power/(rpm*2*M_PI/60),stall=4*nominal*ratio*eff;
     double torque=std::clamp(stall*(duty[side]-omega/noLoad),-stall,stall);
     double meshTorque=0;V reaction=V::Zero;
     for(int j=0;j<n;++j){int id=side*n+j;V rel=H.p.Rot().RotateVectorReverse(st[id].p.Pos()-S.p.Pos());
       double dist=std::hypot(rel.X(),rel.Z());
       if(dist>radius+.005 || dist<radius-.006 || rel.X()>.005 || std::abs(rel.Y())>.028)continue;
       V tangent=H.p.Rot().RotateVector(V(rel.Z()/dist,0,-rel.X()/dist));
       double surface=(st[id].v-S.v).Dot(tangent);
       double magnitude=std::clamp(18*(omega*radius-surface),-8.,8.);
       V f=tangent*magnitude;forces[id]+=f;reaction-=f;meshTorque-=magnitude*radius;
     }
     sprockets[side].AddWorldWrench(ecm,reaction,axis*meshTorque);
     set<gz::sim::components::JointForceCmd>(motorJoints[side],ecm,std::vector<double>{torque});
     auto q=gz::sim::Joint(frontSusp[side]).Position(ecm);auto t=gz::sim::Joint(idlerSlides[side]).Position(ecm);
     if(q&&t&&!q->empty()&&!t->empty())set<gz::sim::components::JointForceCmd>(idlerSlides[side],ecm,std::vector<double>{1500*(.35*(*q)[0]-(*t)[0])});
   }
   for(size_t i=0;i<shoes.size();++i)shoes[i].AddWorldWrench(ecm,forces[i],torques[i]);
   if(time-lastPublish>=.1){gz::msgs::Double error;error.set_data(maxClosureError);closureError.Publish(error);gz::msgs::Pose_V msg;
     auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();msg.mutable_header()->mutable_stamp()->set_sec(ns/1000000000);msg.mutable_header()->mutable_stamp()->set_nsec(ns%1000000000);
     auto *data=msg.mutable_header()->add_data();data->set_key("frame_id");data->add_value("base_link");
     for(const auto &state:st){P p=H.p.Inverse()*state.p;auto *out=msg.add_pose();out->mutable_position()->set_x(p.Pos().X());out->mutable_position()->set_y(p.Pos().Y());out->mutable_position()->set_z(p.Pos().Z());out->mutable_orientation()->set_w(p.Rot().W());out->mutable_orientation()->set_x(p.Rot().X());out->mutable_orientation()->set_y(p.Rot().Y());out->mutable_orientation()->set_z(p.Rot().Z());}
     trackPoses.Publish(msg);lastPublish=time;
   }
 }
};
}
GZ_ADD_PLUGIN(rc_tank::Dynamics,gz::sim::System,gz::sim::ISystemConfigure,gz::sim::ISystemPreUpdate)
