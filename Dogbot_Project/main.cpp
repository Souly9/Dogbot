#include <defaultrobotserver.h>

#include <Aria.h>
#include "followBehaviour.h"
#include "ObjectIdentifier.h"
#include "ObjectTracker.h"
#include "zeitungBehaviour.h"
#include "GotoBehaviour.h"
#include "VisibilityGraph.h"
#include "dijkstra.h"

int main(int argc, char **argv)
{

    DefaultRobotServer server;
    server.init(argc, argv);
    ArMap *map = server.getMap();
    ObjectTracker tracker(500, 2000, server.getLaser());
    server.addAction(tracker, 30);
    tracker.activate();

    ObjectIdentifier reader(server.getLaser(), &tracker, map);
    server.addAction(reader, 30);
    reader.activate();

    zeitungBehaviour paper;
    server.addAction(paper);
    paper.activate();

    followBehaviour follow(&tracker, &paper, map, &reader);
    server.addAction(follow);
    follow.activate();

    GotoBehaviour goTo(false, &follow);
    server.addAction(goTo);
    goTo.activate();
    server.run();

    server.run();
    return 0;
}
