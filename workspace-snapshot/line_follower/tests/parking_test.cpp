#include "Parking.h"
#include <cassert>
#include <iostream>
#include <limits>
using namespace parking;
bool near(float a, float b, float tol=0.01f) { return std::fabs(a-b)<tol; }

// Ideal differential-drive kinematics: validates geometry, not tire slip/real hardware.
void integrate(const Plan &p, float track, float &x, float &y, float &yaw) {
    x=y=yaw=0;
    for (size_t i=0; i<p.count; ++i) {
        float d=(p.steps[i].leftMm+p.steps[i].rightMm)/2;
        float t=(p.steps[i].rightMm-p.steps[i].leftMm)/track;
        if (std::fabs(t)<0.00001f) { x+=d*std::cos(yaw); y+=d*std::sin(yaw); }
        else { x+=d/t*(std::sin(yaw+t)-std::sin(yaw));
               y-=d/t*(std::cos(yaw+t)-std::cos(yaw)); }
        yaw+=t;
    }
}
int main() {
    Plan p; float x,y,yaw;
    assert(!reverseBay(p, 0, 150, 1));
    assert(!parallelBay(p,120,60,45,0,1));
    assert(!parallelBay(p,120,180,95,0,1));
    assert(!reverseBay(p,120,std::numeric_limits<float>::quiet_NaN(),1));
    for (int side : {-1,1}) {
        assert(reverseBay(p,120,180,side)); integrate(p,120,x,y,yaw);
        assert(near(x,0) && near(y,-side*180) && near(yaw,side*kPi/2));
        assert(parallelBay(p,120,180,60,20,side)); integrate(p,120,x,y,yaw);
        assert(near(x,-2*180*std::sin(kPi/3)+20));
        assert(near(y,-side*2*180*(1-std::cos(kPi/3))) && near(yaw,0));
    }
    Controller c;
    assert(!c.start(Plan{},0,0,0));
    p.count=1; p.steps[0]={100,100};
    assert(c.start(p,0,0,0)); auto out=c.update(20,0,0);
    assert(out.left>0 && out.right>0 && out.left<=90);
    out=c.update(40,50,20); assert(out.left<out.right); // Synchronise a leading wheel.
    out=c.update(850,50,20); assert(c.status==Status::Stalled && out.left==0 && out.right==0);
    c.start(p,0,0,0); out=c.update(40,-6,2); assert(c.status==Status::WrongDirection && out.left==0);
    c.start(p,0,0,0); out=c.update(8000,100,100); assert(c.status==Status::Timeout && out.right==0);
    c.start(p,0,0,0); out=c.update(20,100,0); assert(out.left==0 && out.right>0);
    out=c.update(820,100,0); assert(c.status==Status::Stalled);
    c.start(p,0,0,0); c.stop(); out=c.update(20,0,0); assert(out.left==0 && out.right==0);
    c.start(p,0,0,0); out=c.update(20,NAN,0); assert(c.status==Status::Invalid && out.left==0);
    // Successful two-segment run, with mandatory zero-output settling between phases.
    reverseBay(p,120,180,1); c.start(p,0,0,0);
    out=c.update(20,p.steps[0].leftMm,p.steps[0].rightMm); assert(out.left==0 && out.right==0);
    c.update(320,p.steps[0].leftMm,p.steps[0].rightMm); assert(c.step==1);
    out=c.update(340,p.steps[0].leftMm,p.steps[0].rightMm); assert(out.left<0 && out.right<0);
    c.update(500,p.steps[0].leftMm-180,p.steps[0].rightMm-180);
    out=c.update(800,p.steps[0].leftMm-180,p.steps[0].rightMm-180);
    assert(c.status==Status::Complete && out.left==0 && out.right==0);
    // millis() wrap must preserve stall timing.
    p.count=1; p.steps[0]={100,100}; c.start(p,0xffffff00U,0,0);
    out=c.update(0x20U,0,0); assert(c.status==Status::Running);
    out=c.update(0x300U,0,0); assert(c.status==Status::Stalled);
    std::cout << "parking geometry, feedback faults, completion and wrap tests passed\n";
}
