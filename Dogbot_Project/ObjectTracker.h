#ifndef OBJECTTRACKER_H
#define OBJECTTRACKER_H

#include <Aria.h>
#include <ArAction.h>

#include "ReadingBox.h"
#include "CUtils.h"


/**
 * Class that holds X ReadingBoxes for the Herrchen, L/R side and walls
 * Boxes are checked and distances updated so other classes can access the readings and decide how to react
 */
class ObjectTracker : public ArAction
{
public:
    ObjectTracker(int centerLength, int centerWidth, ArLaser* m_laser);
    ~ObjectTracker();

    ArActionDesired *fire(ArActionDesired currentAction);

    /**
     * Adds a ReadingBox to this tracker, values is the array of coordinates (x1, y1, x2, y2)
     * @param targetPos The position of the target
     * @param dist The distance between the robot and target
    */
    void SetTarget(CVector targetPos, double dist);

    CVector getTargetReading();
    // Used to check if the Tracker is active and the target was found
    bool isTracking() {return m_trackTarget;}

protected:
    CVector getLaserReading(ReadingBox box);
private:
    ArActionDesired m_action;
    ArLaser* m_laser;
    ArPose m_reading;

    bool m_trackTarget = false;
    int m_targetLength, m_targetWidth;

    ReadingBox m_targetBox;
    // Coordinates of readings of the boxes
    CVector m_targetPos;
    CVector m_prevTargetPos;
};
#endif
