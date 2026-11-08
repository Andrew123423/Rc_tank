#pragma once
// Arduino compatibility shim. Single translation unit only: upstream sketch has globals.
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstddef>
namespace firmware {
inline unsigned long simulated_ms=0;
inline unsigned long millis(){return simulated_ms;}
inline void delay(int){}
constexpr int OUTPUT=1,LOW=0,HIGH=1,A0=0,GRB=0,IBUSBM_NOTIMER=0,IBUSS_INTV=0;
struct WS2812{};
struct CRGB {int r=0,g=0,b=0; CRGB()=default;CRGB(int R,int G,int B):r(R),g(G),b(B){}};
template<size_t N>struct LedBuffer {
 std::array<CRGB,N> pixels{};CRGB discard;
 CRGB&operator[](int i){return i>=0&&i<static_cast<int>(N)?pixels[i]:discard;}
 operator CRGB*(){return pixels.data();}
};
struct FastLEDClass {
 std::array<CRGB*,3> buffers{};std::array<int,3> counts{};int count=0;std::array<CRGB,24> shown{};
 template<class T,int Pin,int Order>void addLeds(CRGB*p,int n){buffers[count]=p;counts[count]=n;++count;}
 void clear(){for(int i=0;i<count;++i)std::fill(buffers[i],buffers[i]+counts[i],CRGB());}
 void show(){int p=0;for(int i=0;i<count;++i)for(int j=0;j<counts[i];++j)shown[p++]=buffers[i][j];}
};
inline FastLEDClass FastLED;
struct SerialMock {void begin(int){} template<class T>void print(T){}template<class T>void println(T){} };
inline SerialMock Serial,Serial1,Serial2;
inline std::array<int,16> channels={1500,1500,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000,1000};
struct IBusBM {void begin(SerialMock&,int){}void addSensor(int){}void loop(){}int readChannel(int i){return channels.at(i);}void setSensorMeasurement(int,int){}};
struct PWMServo {int angle=90;void attach(int){}void write(int x){angle=x;}};
inline std::array<int,64> pwm{};
inline void pinMode(int,int){}
inline void analogWrite(int pin,int value){pwm.at(pin)=std::clamp(value,0,255);}
inline void digitalWrite(int pin,int value){pwm.at(pin)=value?255:0;}
inline int analogRead(int){return 810;} // constant 11.96 V only for compatibility, no battery simulation
inline long map(long x,long il,long ih,long ol,long oh){return (x-il)*(oh-ol)/(ih-il)+ol;}
inline int constrain(int x,int a,int b){return std::clamp(x,a,b);}
using std::abs;
#include "tank_firmware.inc"
inline void tick(unsigned long now){
 simulated_ms=now;
 // Upstream leaves old PWM latched at exact RC endpoints / centred stick.
 // Clear drive outputs before each update; include endpoints in adapted sketch.
 pwm[4]=pwm[5]=pwm[9]=pwm[10]=0;
 loop();
}
inline double leftDuty(){return (pwm[10]-pwm[9])/255.;}
inline double rightDuty(){return (pwm[5]-pwm[4])/255.;}
}
