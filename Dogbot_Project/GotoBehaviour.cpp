#include "GotoBehaviour.h"
#include <vector>
#include <algorithm>
#include "VisibilityGraph.h"
#include "dijkstra.h"

#define OFFSET 800
GotoBehaviour::GotoBehaviour(bool useTheta, followBehaviour *follow)
    : ArAction("GotoBehaviour", "Moves the robot to desired position"), useTheta(useTheta), m_follow(follow), myState(Calc), m_myOverArchingState(Calc) {}

GotoBehaviour::~GotoBehaviour() {}

ArActionDesired *GotoBehaviour::fire(ArActionDesired currentAction)
{
    // Should we get the paper?
    if (m_follow->getGo())
    {
        if (myState == Calc)
        {
            // Compute the graph and extract the shortest path
            vector<Node> visGraph = VisibilityGraph::ComputeGraph(m_follow->getMap(), myRobot->getPose());
            std::vector<std::unordered_map<int, int>> dijkstraGraph = dijkstra::graphconversion(visGraph);
            std::deque<int> path = dijkstra::dijkstra_algorithm(dijkstraGraph, 0, dijkstraGraph.size() - 1, dijkstraGraph.size());

            std::vector<ArPose> pathCoords;
            for (int i = 0; path.size() > i; ++i)
            {
                Node tmp = visGraph[path[i]];
                pathCoords.push_back(ArPose(tmp.position[0], tmp.position[1], 0));
            }
            // Setup to iterate over the pathCoords list
            m_desiredPoses = pathCoords;
            myState = Init;
            m_myOverArchingState = PosInit;
        }
    }
    // Should we return to the original position? / Did we find the paper?
    if (m_follow->getGoBack())
    {
         // Reverse the list only once
        if (!reverse)
        {
            reverse = true;
            std::reverse(m_desiredPoses.begin(), m_desiredPoses.end());
            m_currentIndex = 0;
            myState = Init;
            m_myOverArchingState = PosInit;
        }
    }

    // Grab the first position in the vector
    if (m_myOverArchingState == PosInit)
    {
        ArPose target = m_desiredPoses[m_currentIndex];

        m_currentIndex++;
        desiredPose = target;
        m_myOverArchingState = Driving;
    }
    // Grab the next position or drive back
    else if (m_myOverArchingState == WaitingForNewCoords)
    {
        if (m_currentIndex >= m_desiredPoses.size())
        {
            m_myOverArchingState = Done;
            if (m_follow->getGoBack())
            {
                myState = Done;
            }
        }
        else
        {
            ArPose target = m_desiredPoses[m_currentIndex];

            m_currentIndex++;
            desiredPose = target;
            m_myOverArchingState = Driving;
        }
    }
    // Main driving procedure
    else if (m_myOverArchingState == Driving)
    {
        float currX = this->myRobot->getPose().getX(), currY = this->myRobot->getPose().getY();

        // First determine the angle between the current pose and the next desiredPose
        if (myState == Init)
        {
            float angle = myRobot->findDeltaHeadingTo(desiredPose);
            totalAngle += angle;
            myRobot->setDeltaHeading(angle);
            myState = DriveTowards;
            return &this->action;
        }
        // Then drive towards it
        if (myState == DriveTowards && myRobot->isHeadingDone())
        {
            this->action.setVel(1000);
            myState = DoneDriving;
            return &this->action;
        }

        float destVec[2] = {this->desiredPose.getX() - myRobot->getPose().getX(), this->desiredPose.getY() - myRobot->getPose().getY()};
        float len = sqrt(destVec[0] * destVec[0] + destVec[1] * destVec[1]);

        // And constantly check if we're close enough to have arrived
        if (myState == DoneDriving && len < 1200)
        {
            this->action.setVel(0);
            myState = Init;
            m_myOverArchingState = WaitingForNewCoords;
            if (useTheta)
            {
                double th = myRobot->getTh();
                double diff = desiredPose.getTh() - th;
                if (diff > 0.5f)
                    myRobot->setDeltaHeading(diff);
            }
        }
        return &this->action;
    }
    // State to disable the behaviour completely and redetect the target
    else if (m_myOverArchingState == Done && myState == Done)
    {
        if (!fin)
        {
            fin = true;
            followBehaviour::State tmp;
            tmp = followBehaviour::init;
            m_follow->setState(tmp);
        }
    }
    return &this->action;
}
