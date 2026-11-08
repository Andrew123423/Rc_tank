#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <std_msgs/msg/string.hpp>
#include <iomanip>
#include <rc_tank_sim/firmware_shim.hpp>
#include <chrono>
#include <sstream>
using namespace std::chrono_literals;
class Controller: public rclcpp::Node {
 rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr lpub,rpub;
 rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gpub;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr leds;
 rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status;
 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist;
 rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr rc;
 rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gears,lights;
 rclcpp::TimerBase::SharedPtr timer;
 std::chrono::steady_clock::time_point last=std::chrono::steady_clock::time_point::min();
 double throttle=0,steering=0,linear=0,angular=0,timeout=.35,low=5.308235294117647,high=1.92,radius=.03487,rpm=3000,sep=.347;
 int gear=1,light=0;bool physical=false;int count=0;int64_t lastRos=-1;
public:
 Controller():Node("tank_controller"){
   timeout=declare_parameter("command_timeout",.35);low=declare_parameter("gear_low",5.308235294117647);high=declare_parameter("gear_high",1.92);
   radius=declare_parameter("sprocket_radius",.03487);rpm=declare_parameter("motor_rpm",3000.);sep=declare_parameter("track_separation",.347);
   lpub=create_publisher<std_msgs::msg::Float64>("/tank/motor_left",10);rpub=create_publisher<std_msgs::msg::Float64>("/tank/motor_right",10);
   gpub=create_publisher<std_msgs::msg::Int32>("/tank/gear_internal",10);leds=create_publisher<std_msgs::msg::String>("/tank/led_colors",10);status=create_publisher<std_msgs::msg::String>("/tank/status",10);
   twist=create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10,[this](geometry_msgs::msg::Twist::ConstSharedPtr m){
     if(!std::isfinite(m->linear.x)||!std::isfinite(m->angular.z))return;
     linear=m->linear.x;angular=m->angular.z;physical=true;last=std::chrono::steady_clock::now();});
   rc=create_subscription<geometry_msgs::msg::Vector3>("/tank/rc_input",10,[this](geometry_msgs::msg::Vector3::ConstSharedPtr m){
     if(!std::isfinite(m->x)||!std::isfinite(m->y))return;
     throttle=std::clamp(m->x,-1.,1.);steering=std::clamp(m->y,-1.,1.);physical=false;last=std::chrono::steady_clock::now();});
   gears=create_subscription<std_msgs::msg::UInt8>("/tank/gear",10,[this](std_msgs::msg::UInt8::ConstSharedPtr m){if(m->data==1||m->data==2)gear=m->data;});
   lights=create_subscription<std_msgs::msg::UInt8>("/tank/lights",10,[this](std_msgs::msg::UInt8::ConstSharedPtr m){if(m->data<=4 && light!=m->data){light=m->data;firmware::FastLED.clear();firmware::FastLED.show();}});
   firmware::setup();
   // 200 Hz preserves the source's fastest 5 ms light animation stage.
   timer=create_wall_timer(5ms,[this]{tick();});
 }
 void tick(){
   auto now=std::chrono::steady_clock::now();double age=last==std::chrono::steady_clock::time_point::min()?1e9:std::chrono::duration<double>(now-last).count();
   int64_t ros=get_clock()->now().nanoseconds();if(lastRos>=0&&ros<lastRos){age=1e9;last=std::chrono::steady_clock::time_point::min();}lastRos=ros;
   bool stopped=age>timeout;
   firmware::channels[0]=static_cast<int>(std::lround(1500-500*steering));
   firmware::channels[1]=static_cast<int>(std::lround(1500+500*throttle));
   firmware::channels[6]=gear==1?1000:2000;firmware::channels[7]=light==4?2000:1000;firmware::channels[8]=light==2?1500:(light==3?2000:1000);firmware::channels[9]=light==1?2000:1000;
   firmware::tick(static_cast<unsigned long>(ros/1000000));
   double l=firmware::leftDuty(),r=firmware::rightDuty();
   // /cmd_vel is SI differential drive; /tank/rc_input uses the source PWM mixer.
   double vmax=rpm*2*M_PI/60/(gear==1?low:high)*radius;
   if(physical){l=(linear-angular*sep/2)/vmax;r=(linear+angular*sep/2)/vmax;double scale=std::max({1.,std::abs(l),std::abs(r)});l/=scale;r/=scale;}
   if(stopped)l=r=0;
   if(++count%4==0){
     std_msgs::msg::Float64 m;m.data=l;lpub->publish(m);m.data=r;rpub->publish(m);
     std_msgs::msg::Int32 g;g.data=gear;gpub->publish(g);
     std_msgs::msg::String colors;std::ostringstream hex;hex<<std::hex<<std::setfill('0');for(const auto &c:firmware::FastLED.shown)hex<<std::setw(2)<<c.r<<std::setw(2)<<c.g<<std::setw(2)<<c.b;colors.data=hex.str();leds->publish(colors);
     std_msgs::msg::String st;std::ostringstream out;out<<"{\"gear\":"<<gear<<",\"ratio\":"<<(gear==1?low:high)<<",\"lights\":"<<light<<",\"left_duty\":"<<l<<",\"right_duty\":"<<r<<",\"watchdog\":"<<(stopped?"true":"false")<<",\"max_speed_m_s\":"<<vmax<<",\"servo_deg\":"<<firmware::myservo.angle<<"}";st.data=out.str();status->publish(st);
   }
 }
};
int main(int argc,char**argv){rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<Controller>());rclcpp::shutdown();}
