#include "ObjectTracker.h"

ObjectTracker::ObjectTracker(int centerLength, int centerWidth, ArLaser *m_laser) : ArAction("ObjectTracker", "Tracks a driving object in front of the robot"), m_targetLength(centerLength), m_targetWidth(centerWidth), m_laser(m_laser)
{
}
ObjectTracker::~ObjectTracker() {}

ArActionDesired *ObjectTracker::fire(ArActionDesired desiredAction)
{
    if (m_laser->isConnected())
    {
        // Read positions if we track something
        if (m_trackTarget)
        {
            m_laser->lockDevice();
            m_targetPos = getLaserReading(m_targetBox);
            m_laser->unlockDevice();
        }
    }
    else
    {
        m_laser->asyncConnect();
    }
    return &m_action;
}

CVector ObjectTracker::getLaserReading(ReadingBox box)
{
    m_laser->currentReadingBox(box.x1, box.y1, box.x2, box.y2, &m_reading);
    CVector tmp(m_reading.getX(), m_reading.getY());
    return tmp;
}

// Initializes ReadingBoxes with the target Position
// Initialized at the target so we can track it immediatly
void ObjectTracker::SetTarget(CVector targetPos, double dist)
{
    // Creates the central ReadingBox structure at the heart of the reader
    // Very rough outline:
    // 
    // 
    //          .-.
    //          | |
    //          | |
    //          | |
    //          | |
    //          '–'
    //  \                 /
    //        roboter
    //        center 

    int yOffset = 100 + dist;
    CVector centerL(400, 200);
    CVector centerR(20000, -200);
    m_targetBox = ReadingBox(centerL.x, centerL.y, centerR.x, centerR.y);
    m_targetPos = targetPos;
    m_prevTargetPos = targetPos;

    m_trackTarget = true;
}
CVector ObjectTracker::getTargetReading()
{
    return m_targetPos;
}
