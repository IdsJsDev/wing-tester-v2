#include "../firmware/wing_tester/flow_detection.h"

constexpr bool sustained() {
  FlowDetection gate;
  return !gate.update(true, 0, 300) && !gate.update(true, 299, 300) &&
      gate.update(true, 300, 300);
}
constexpr bool spike() {
  FlowDetection gate;
  return !gate.update(true, 100, 300) && !gate.update(false, 120, 300) &&
      !gate.update(true, 400, 300) && !gate.update(true, 699, 300) &&
      gate.update(true, 700, 300);
}
constexpr bool noFlow() {
  FlowDetection gate;
  return !gate.update(false, 0, 300) && !gate.update(false, 10000, 300);
}
constexpr bool rollover() {
  FlowDetection gate;
  return !gate.update(true, 0xFFFFFFF0UL, 32) &&
      !gate.update(true, 0x0000000FUL, 32) && gate.update(true, 0x00000010UL, 32);
}
static_assert(sustained(), "Sustained reaction must pass only after hold time");
static_assert(spike(), "Spike or interrupted reaction must reset hold time");
static_assert(noFlow(), "No flow must never pass");
static_assert(rollover(), "Time rollover must preserve hold duration");

constexpr bool returnToZero() {
  FlowDetection gate;
  return !gate.update(false, 0, 1000) && !gate.update(true, 500, 1000) &&
      !gate.update(true, 1499, 1000) && gate.update(true, 1500, 1000);
}
constexpr bool crossesZeroThenLeaves() {
  FlowDetection gate;
  return !gate.update(true, 0, 1000) && !gate.update(false, 500, 1000) &&
      !gate.update(true, 700, 1000) && !gate.update(true, 1699, 1000) &&
      gate.update(true, 1700, 1000);
}
static_assert(returnToZero(), "Return must stay within band for one full second");
static_assert(crossesZeroThenLeaves(), "Brief crossing must not pass the return stage");

constexpr bool earlyBlowIsRemembered() {
  FlowDetection gate;
  return !gate.updateLatched(true, 100, 100) &&
      gate.updateLatched(true, 200, 100) &&
      gate.updateLatched(false, 220, 100) &&
      gate.updateLatched(false, 4000, 100) &&
      gate.updateLatched(false, 10000, 100);
}
constexpr bool isolatedSpikeIsIgnored() {
  FlowDetection gate;
  return !gate.updateLatched(true, 100, 100) &&
      !gate.updateLatched(false, 120, 100) &&
      !gate.updateLatched(false, 10000, 100);
}
static_assert(earlyBlowIsRemembered(), "Early blow must survive zero pressure at stage end");
static_assert(isolatedSpikeIsIgnored(), "Single-sample spike must not confirm flow");
