#pragma once
#include "Mission.h"
#include "RobotConfig.h"

namespace taskcfg {
// Keep these locked until the exact starting pose and each segment are measured.
constexpr bool COMPLETE_MISSIONS_ENABLED = false;
constexpr bool VISION_UART_ENABLED = false; // Reuses existing PA3 RX / PA2 TX, disables text TX.
constexpr bool VISION_PROTOCOL_VERIFIED = false;
constexpr bool EXIT_POSE_VERIFIED = false;
struct Approach {
    bool verified;
    uint8_t routeLegs; // zero: begin already aligned on the destination edge
    float firstLegMm;
    float cornerAdvanceMm, cornerYawDeg; // measured alignment after last route point
    float followMm; // distance along the destination edge to parking alignment pose
    float alignAdvanceMm, alignYawDeg;
};
constexpr Approach REVERSE_APPROACH = {false, 0, 0, 0, 0, 0, 0, 0};
constexpr Approach PARALLEL_APPROACH = {false, 0, 0, 0, 0, 0, 0, 0};
constexpr float EXIT_FORWARD_MM = 0;
constexpr float EXIT_TURN_DEG = 0;
constexpr float EXIT_ALIGN_MM = 0;
constexpr float EXIT_FIRST_LEG_MM = 0;
constexpr uint8_t EXIT_NEXT_POINT = 255; // A=0, B=1, C=2, D=3; unknown=255
constexpr int EXIT_POINT_STEP = 0; // +1 ABCD, -1 ADCB; unknown=0

inline bool addMotion(mission::Plan &plan, float distance, float yawDeg) {
    if (!std::isfinite(distance) || !std::isfinite(yawDeg) || std::fabs(yawDeg) > 180) return false;
    if (std::fabs(distance) < 0.01f && std::fabs(yawDeg) < 0.01f) return true;
    const auto m = parking::motion(distance, yawDeg * parking::kPi / 180, cfg::TRACK_MM);
    return plan.add(mission::Action::Move, m.leftMm, m.rightMm, 0, 10000);
}
inline bool buildParking(mission::Plan &plan, bool parallel, const Approach &a) {
    plan = mission::Plan{};
    if (!a.verified || !cfg::PARKING_START_POSE_VERIFIED || !parking::positive(a.followMm) ||
        a.routeLegs > 3 || !parking::positive(cfg::TRACK_MM)) return false;
    if (a.routeLegs && (!cfg::ROUTE_START_POSE_VERIFIED ||
        !plan.add(mission::Action::Route, a.firstLegMm, 0, a.routeLegs, 30000))) return false;
    if (a.routeLegs && !addMotion(plan, a.cornerAdvanceMm, a.cornerYawDeg)) return false;
    if (!plan.add(mission::Action::FollowDistance, a.followMm)) return false;
    if (!addMotion(plan, a.alignAdvanceMm, a.alignYawDeg)) return false;
    parking::Plan park;
    const bool ok = parallel
        ? parking::parallelBay(park, cfg::TRACK_MM, cfg::PARALLEL_RADIUS_MM,
                              cfg::PARALLEL_ANGLE_DEG, cfg::PARALLEL_TRIM_MM, cfg::BAY_SIDE)
        : parking::reverseBay(park, cfg::TRACK_MM, cfg::REVERSE_DEPTH_MM, cfg::BAY_SIDE);
    return ok && mission::appendParking(plan, park);
}
inline bool buildExit(mission::Plan &plan) {
    plan = mission::Plan{};
    if (!EXIT_POSE_VERIFIED || !VISION_UART_ENABLED || !VISION_PROTOCOL_VERIFIED ||
        !parking::positive(EXIT_FORWARD_MM) || !parking::positive(EXIT_FIRST_LEG_MM) ||
        EXIT_NEXT_POINT > 3 || (EXIT_POINT_STEP != 1 && EXIT_POINT_STEP != -1)) return false;
    return plan.add(mission::Action::ReadRoman, 0, 0, 0, 10000) &&
        addMotion(plan, EXIT_FORWARD_MM, 0) && addMotion(plan, 0, EXIT_TURN_DEG) &&
        addMotion(plan, EXIT_ALIGN_MM, 0) &&
        plan.add(mission::Action::Route, EXIT_FIRST_LEG_MM, 0, 0, 30000);
}
} // namespace taskcfg
