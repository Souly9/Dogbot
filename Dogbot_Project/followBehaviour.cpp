#include "followBehaviour.h"

followBehaviour::followBehaviour(ObjectTracker *reader, zeitungBehaviour *paper, ArMap *map, ObjectIdentifier *ident)
    : ArAction("followBehaviour", "Makes the Robot follow another robot"), objectTracker(reader), m_paper(paper), m_map(map), m_identifier(ident)
{
}

followBehaviour::~followBehaviour() {}

bool followBehaviour::getGo()
{
    return go;
}

bool followBehaviour::getGoBack()
{
    return goBack;
}

ArMap *followBehaviour::getMap()
{
    return m_map;
}

void followBehaviour::setState(State s)
{
    state = s;
}

ArActionDesired *followBehaviour::fire(ArActionDesired currentAction)
{
    CVector centerPos = objectTracker->getTargetReading();
    double dist = centerPos.length();

    double leftBackDistance = myRobot->getSonarRange(12);
    double thirteen = myRobot->getSonarRange(13);
    double fourteen = myRobot->getSonarRange(14);
    double fifteen = myRobot->getSonarRange(15);
    double sumLeft = leftBackDistance + fifteen + fourteen + thirteen; //sum of all sensors on the left back side
    double rightBackDistance = myRobot->getSonarRange(11);
    double ten = myRobot->getSonarRange(10);
    double nine = myRobot->getSonarRange(9);
    double eight = myRobot->getSonarRange(8);
    double sumRight = rightBackDistance + eight + nine + ten; //sum of all sensors on the right back side
    double lastDist = m_prevTargetPos.length() - dist;

    //If ordered to get the newspaper we pause the followbehaviour and enter GoToBehaviour
    if (m_paper->getGetPaper())
    {
        state = pathfinding;
    }

    if (state == init)
    { // Find owner
        if (goBack)
        {
            m_identifier->setState(herrchenLocalizing);
        }
        if (objectTracker->isTracking())
        {
            m_prevTargetPos = objectTracker->getTargetReading();
            state = moving;
        }
        return &action;
    }
    // Start moving
    if (state == moving)
    {

        ArTransform worldTransform = myRobot->getToGlobalTransform();
        ArPose worldPos = worldTransform.doTransform(ArPose(centerPos.x, centerPos.y, 0));
        // Is the current central reading an obstacle?
        if (MapUtilities::isObstacle(worldPos, m_map))
        {
            // Are there any really close by walls?
            // If so, try avoiding them
            double angle = m_identifier->detectTarget(false);
            std::cout << angle << '\n';
            if (angle != 900)
            {
                myRobot->setDeltaHeading(angle);
            }
        }
        // Drive towards it otherwise
        else
        {

            if (dist > space)
            {
                action.setVel(speed * (dist / space));
            }

            // If we dont hold enough space -> drive backwards, a safe space to walls behind us is ensured
            // this mostly means something is coming towards us
            else if (dist < space - 100 && rightBackDistance > space && leftBackDistance > space)
            {
                action.setVel(-100 * ((space) / dist));
            } // If we cant hold the safe space to our behind, turn so we drive in the direction with more free space
            else if (dist < space - 100 && sumRight <= sumLeft && rightBackDistance <= space)
            {
                myRobot->setDeltaHeading(-angle);
            } // This reading doesn't need a strong turn so we lower the angle
            else if (dist < space - 100 && sumRight <= sumLeft && (ten <= space || nine <= space))
            {
                myRobot->setDeltaHeading(-(angle / 2));
            } // The same for the other direction
            else if (dist < space - 100 && sumRight > sumLeft && leftBackDistance <= space)
            {
                myRobot->setDeltaHeading(angle);
            }
            else if (dist < space - 100 && sumRight > sumLeft && (thirteen <= space || fourteen <= space))
            {
                myRobot->setDeltaHeading(angle / 2);
            }
            else // If none of these cases is true, the robot is standing safely before its owner
            {
                action.setVel(0);
            }
        }
        // If our backspacing gets to low we pause and turn safely before continuing to drive
        if (leftBackDistance < (space / 4) || fifteen < (space / 4) || fourteen < (space / 4) || thirteen < (space / 4) || rightBackDistance < (space / 4) || ten < (space / 4) || nine < (space / 4) || eight < (space / 4))
        {
            state = rotate;
        }
        m_prevTargetPos = objectTracker->getTargetReading();
        return &action;
    }
    if (state == rotate)
    {
        //same as above
        action.setVel(0);
        if (sumRight >= sumLeft)
        {
            myRobot->setDeltaHeading((angle));
        }
        else
        {
            myRobot->setDeltaHeading(-(angle));
        }
        state = moving;
    }

    if (state == pathfinding)
    {
        //ensures we only brake ones and don't set the velocity to 0 on every fire
        if (!brake)
        {
            brake = true;
            action.setVel(0);
        }
        // activate pathfiding to get paper
        go = true;
        if (m_paper->getGotPaper())
        {
            go = false;
            goBack = true;
        }
    }

    return &action;
}