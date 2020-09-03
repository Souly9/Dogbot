#ifndef ACTSACTION_H
#define ACTSACTION_H

#include <ArAction.h>
#include <ArACTS.h>

class zeitungBehaviour : public ArAction
{
public:
    zeitungBehaviour();
    ~zeitungBehaviour();
    ArActionDesired *fire(ArActionDesired currentDesired);

    virtual void activate();
    virtual void deactivate();

    void printChannel(int channel);
    void printBlobInfo(ArACTSBlob &blob);
    void giveCommand(int channel);
    bool getGetPaper();
    bool getGotPaper();

protected:
    ArACTS_1_2 acts;
    ArACTSBlob blob;
    bool getPaper=false;
    bool gotPaper=false;
};
#endif
