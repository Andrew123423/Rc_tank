#include <rc_tank_sim/firmware_shim.hpp>
#include <cassert>
#include <iostream>
int main(){
 using namespace firmware;setup();
 tick(0);assert(leftDuty()==0&&rightDuty()==0);
 channels[1]=2000;tick(10);assert(leftDuty()==1&&rightDuty()==1);
 channels[1]=1000;tick(20);assert(leftDuty()==-1&&rightDuty()==-1);
 channels[1]=1500;tick(30);assert(leftDuty()==0&&rightDuty()==0);
 channels[0]=1000;tick(40);assert(leftDuty()<0&&rightDuty()>0);
 channels[0]=2000;tick(50);assert(leftDuty()>0&&rightDuty()<0);
 channels[0]=1500;channels[6]=1000;tick(60);assert(myservo.angle==40);
 channels[6]=2000;tick(70);assert(myservo.angle==140);
 channels[9]=2000;tick(80);assert(FastLED.shown[0].r==255&&FastLED.shown[18].r==255);
 channels[9]=1000;channels[8]=1500;tick(90);assert(FastLED.shown[6].r==255);
 channels[8]=1000;channels[7]=2000;
 bool red=false,blue=false,white=false;int stages=0;
 for(unsigned long ms=95;ms<180000;ms+=5){tick(ms);stages|=1<<r;for(const auto &c:FastLED.shown){red|=c.r==255&&c.b==0;blue|=c.b==255&&c.r==0;white|=c.r==255&&c.b==255;}}
 assert(red&&blue&&white);assert((stages&0x1fe)==0x1fe);
 channels[7]=1000;channels[8]=2000;bool on=false,off=false;
 for(unsigned long ms=180000;ms<180500;ms+=5){tick(ms);on|=FastLED.shown[0].r==255;off|=FastLED.shown[0].r==0;}
 assert(on&&off);
 std::cout<<"PASS: drive endpoints, neutral, pivot, gears, lighting and all eight police stages\n";
}
