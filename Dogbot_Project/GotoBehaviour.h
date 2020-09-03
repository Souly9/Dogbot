#include <Aria.h>
#include "followBehaviour.h"

class GotoBehaviour : public ArAction
{
public:
    enum GotoState
    {
        Calc,
        PosInit,
        Init,
        Driving,
        WaitingForNewCoords,
        DetDir,
        DetAngle,
        DriveTowards,
        DoneDriving,
        Done
    };

    GotoBehaviour(bool useTheta, followBehaviour *follow);
    virtual ~GotoBehaviour();

    ArActionDesired *fire(ArActionDesired currentAction);

private:
    std::vector<ArPose> m_desiredPoses;
    ArPose desiredPose;
    int m_currentIndex = 0;
    bool useTheta;
    followBehaviour *m_follow;
    ArActionDesired action;
    GotoState myState;
    GotoState m_myOverArchingState;
    float dir[2] = {1, 0};
    float start[2];
    double m_offset = 0.93;
    double m_totalLength = 0;
    int totalAngle = 0;
    bool reverse = false;
    bool fin = false;
};