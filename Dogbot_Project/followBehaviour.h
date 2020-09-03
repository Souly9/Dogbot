#ifndef FOLLOWBEHAVIOUR_H
#define FOLLOWBEHAVIOUR_H

#include <math.h>
#include <Aria.h>
#include <ArAction.h>

#include "ObjectTracker.h"
#include "ObjectIdentifier.h"
#include "CUtils.h"
#include "MapUtilities.h"
#include "zeitungBehaviour.h"

class followBehaviour : public ArAction
{
public:
    followBehaviour(ObjectTracker *reader, zeitungBehaviour *paper, ArMap *map, ObjectIdentifier *ident);
    ~followBehaviour();
    enum State
    {
        init,
        moving,
        rotate,
        pathfinding
    };
    ArActionDesired *fire(ArActionDesired currentAction);

    // Check left and right readingBoxes for possible readings
    double checkSides();
    bool compareTargetPositions(CVector pos);
    bool getGo();
    bool getGoBack();
    void setState(State s);

    ArMap *getMap();

private:
    ArActionDesired action;
    double distance;
    State state = init;
    ObjectTracker *objectTracker;
    ObjectIdentifier *m_identifier;

    ArMap *m_map;
    zeitungBehaviour *m_paper;
    double m_edgeAvoidanceStrength = 0.5;
    int m_standOffDistance = 700;

    double speed = 100;
    double space = 1000;
    double angle = 20;

    bool brake = false;
    bool go = false;     //used to start the GoToBehaviour
    bool goBack = false; //used to signal the GoToBehaviour to drive back

    CVector m_prevTargetPos;
};
#endif
