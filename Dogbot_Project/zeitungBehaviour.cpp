#include "zeitungBehaviour.h"

#include <Aria.h>

// Copies most of the funcionality from actsaction snippet
// adds methods to exchange signals with followBehaviour to start GoToBehaviour

zeitungBehaviour::zeitungBehaviour() : ArAction("zeitungBehaviour",
             "A simple action to give orders via ACTS.")
{
}

zeitungBehaviour::~zeitungBehaviour(){

    if (acts.isConnected()) {
        acts.closePort();
    }
}

ArActionDesired *zeitungBehaviour::fire(ArActionDesired currentDesired)
{
    if (acts.isConnected()) {
        for (int channel = 1; channel <= 2; ++channel)
        {
            printChannel(channel);
        }
    } else {
        ArLog::log(ArLog::Normal, "Not connected to ACTS.");
        this->deactivate();
    }
    return 0;
}
void zeitungBehaviour::activate()
{
    acts.openPort(myRobot);
    ArAction::activate();

}

void zeitungBehaviour::deactivate()
{
    acts.closePort();
    ArAction::deactivate();
}

void zeitungBehaviour::printChannel(int channel)
{
    int numBlobs = acts.getNumBlobs(channel);
    if (numBlobs > 0) {
        ArLog::log(ArLog::Normal, "%d blobs in channel %d", numBlobs, channel);
    }
    for (int i = 0; i < numBlobs; ++i) {
        int blobNo = i+1;
        if(acts.getBlob(channel, blobNo, &blob)) {
            ArLog::log(ArLog::Normal, "  Blob %d:",blobNo);
            printBlobInfo(blob);
            std::cout << "getpaper:" << getPaper <<"\n"<< "gotpaper:" << gotPaper <<"\n";
            giveCommand(channel); // if we found a blob we give a command based on the channel the blob was found in 
        } else {
            ArLog::log(ArLog::Normal, "  Can't get blob %d:",blobNo);
        }
    }
}

void zeitungBehaviour::printBlobInfo(ArACTSBlob &blob)
{
    ArLog::log(ArLog::Normal, "    Area:        %d",blob.getArea());
    ArLog::log(ArLog::Normal, "    BoundingBox: (%d, %d, %d, %d)",
        blob.getTop(), blob.getLeft(), blob.getBottom(), blob.getRight());
    ArLog::log(ArLog::Normal, "    Position:    (%d, %d)",
        blob.getXCG(), blob.getYCG());
}

void zeitungBehaviour::giveCommand(int channel){
    //Channel 1 is used to give the order to get the newspaper, while channel 2 is used to detect the newspaper 
    if(channel == 1){
        getPaper = true;
    }
    else if(channel == 2){
        gotPaper = true;
        getPaper = false;
    }
    else
    {
        ArLog::log(ArLog::Normal, "ERROR: We don't use this channel.");
    }
}
bool zeitungBehaviour::getGotPaper(){
    return gotPaper;
}
bool zeitungBehaviour::getGetPaper(){
    return getPaper;
}