#ifndef TIAGO_IASLAB_SIMULATION_CONSTANT_H
#define TIAGO_IASLAB_SIMULATION_CONSTANT_H

namespace status {

static const unsigned int READY = 0;
static const unsigned int MOVING = 1;
static const unsigned int MOVING_CORRIDOR = 2;
static const unsigned int ARRIVED = 3;
static const unsigned int NOT_ARRIVED = 4;
static const unsigned int SCANNING = 5;
static const unsigned int DONE = 6;
static const unsigned int FAILED = 7;

}  // namespace status

#endif  // TIAGO_IASLAB_SIMULATION_CONSTANT_H