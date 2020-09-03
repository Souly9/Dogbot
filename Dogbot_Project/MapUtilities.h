#ifndef MAPUTILITIES_H
#define MAPUTILITIES_H

#include "CUtils.h"
#define EPSILON 500
//#define EPSILON 600
class MapUtilities
{
public:
    static bool isObstacle(ArPose worldPos, ArMap *map)
    {

        std::vector<ArLineSegment> *lines;
        lines = map->getLines();
        ArLineSegment line;
        ArPose reference1(worldPos.getX() + 50, worldPos.getY() + 50, 0);
        ArPose reference2(worldPos.getX() - 50, worldPos.getY() - 50, 0);
        ArPose perpP;
        ArLineSegment toCheckRef1(worldPos.getX(), worldPos.getY(), worldPos.getX() + 20, worldPos.getY() + 20);
        ArLineSegment toCheckRef2(worldPos.getX(), worldPos.getY(), worldPos.getX() - 20, worldPos.getY() - 20);
        ArLineSegment toCheckForward(worldPos.getX(), worldPos.getY(), worldPos.getX() + EPSILON, worldPos.getY());
        ArLineSegment toCheckBack(worldPos.getX(), worldPos.getY(), worldPos.getX() - EPSILON, worldPos.getY());
        ArLineSegment toCheckForward2(worldPos.getX(), worldPos.getY(), worldPos.getX(), worldPos.getY() + EPSILON);
        ArLineSegment toCheckBack2(worldPos.getX(), worldPos.getY(), worldPos.getX(), worldPos.getY() - EPSILON);

        for (int i = 0; i < lines->size(); ++i)
        {
            line = lines->at(i);

            if (line.intersects(&toCheckRef1, &perpP) ||
                line.intersects(&toCheckRef2, &perpP) ||
                line.intersects(&toCheckForward, &perpP) ||
                line.intersects(&toCheckBack, &perpP) ||
                line.intersects(&toCheckForward2, &perpP) ||
                line.intersects(&toCheckBack2, &perpP))
            {
                return true;
            }
        }
        return false;
    }
};

#endif
