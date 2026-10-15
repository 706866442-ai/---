#include <cassert>
#include <cmath>
#include <iostream>
#include "../src/main.cpp"

void tick(unsigned dt=20) { mockMs+=dt;loop(); }
void moveByOutput() {
    const int l=static_cast<int>(std::round(commandedLeft*.02f));
    const int r=static_cast<int>(std::round(commandedRight*.02f));
    encoder1 += cfg::E1_IS_RIGHT ? r : l;
    encoder2 += cfg::E1_IS_RIGHT ? l : r;
}
void resetFollow() {
    enterState(State::WaitForRelease,mockMs);
    pinLevels[KEY]=HIGH; grayDiagnostics[1]=0;
    state=State::Follow; stateSinceMs=lastControlMs=mockMs;
    applyLine(0,.04f,mockMs);
}
int main() {
    for(int &p:pinLevels)p=HIGH;
    setup();
    resetFollow(); const unsigned before=mockGrayReads;
    encoder1+=2;encoder2+=2;tick();
    assert(mockGrayReads==before); // Speed updated without a gray sample.
    assert(commandedLeft>0 && commandedRight>0);
    setSpeedTargets(-cfg::SPEED_BASE_MMPS,mockMs);
    assert(commandedLeft==0);
    resetFollow();
    for(int n=0;n<55 && state==State::Follow;++n) {
        if(cfg::E1_IS_RIGHT)encoder1+=2;else encoder2+=2;
        tick();
    }
    assert(state==State::Fault && commandedLeft==0 && commandedRight==0);
    resetFollow();pinLevels[KEY]=LOW;tick();
    assert(state==State::WaitForRelease && pinLevels[STBY]==LOW);
    pinLevels[KEY]=HIGH;tick(40);
    // The real firmware dispatch path, not just the standalone sequencer.
    for(Mode mode:{Mode::FullReverse,Mode::FullParallel}) {
        selectedMode=mode;grayDiagnostics[1]=0;startMission(mockMs);
        assert(state==State::Mission);
        for(int n=0;n<1500 && state==State::Mission;++n) {moveByOutput();tick();}
        assert(state==State::WaitForRelease && grayDiagnostics[1]==0);
        assert(commandedLeft==0 && commandedRight==0);
    }
    if(taskcfg::VISION_UART_ENABLED) {
        debugPort.tx.clear();selectedMode=Mode::ExitTarget;startMission(mockMs);
        assert(state==State::Mission && missionController.waitingVision());
        for(uint8_t seq=0;seq<3;++seq) {
            uint8_t b[]={0xa5,1,1,seq,3,95,0};b[6]=vision::crc8(b+1,5);
            for(uint8_t v:b)debugPort.rx.push_back(v);
            tick();assert(commandedLeft==0 && commandedRight==0);
        }
        bool joined=false;float origin=0;
        for(int n=0;n<2000 && state==State::Mission;++n) {
            moveByOutput();
            if(missionController.needsGray()) {
                float l,r;uint32_t errors;wheelDistances(l,r,errors);
                if(!joined){origin=(l+r)*.5f;joined=true;}
                mockGray=(l+r)*.5f-origin>=450?0:0xe7;
            }
            tick();
        }
        assert(state==State::WaitForRelease && grayDiagnostics[1]==0);
        assert(debugPort.tx.empty()); // Shared UART emits no diagnostic text in camera mode.
    } else {
        state=State::Ready;debugPort.room=0;
        for(int n=0;n<60;++n)tick();
        assert(telemetryQueue.dropped>0);
        debugPort.room=64;for(int n=0;n<100;++n)tick();
        assert(debugPort.assertWriteBudget<=16);
    }
    std::cout << "PASS: real main dispatch, independent speed tick, stall/button stop, full parking, "
                 "vision exit/target or bounded telemetry\n";
}
