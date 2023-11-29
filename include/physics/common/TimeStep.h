#ifndef PHYSICS_TIME_STEP_H
#define PHYSICS_TIME_STEP_H

namespace physics {

struct TimeStep {

  public:
    /* -- Attributes -- */

    /* Delta */
    float delta;

    /* Inverse delta */
    float inverseDelta;

    /* Delta ratio */
    float deltaRatio;
};

}

#endif