#include "ObjectIdentifier.h"

ObjectIdentifier::ObjectIdentifier(ArLaser *l, ObjectTracker *tracker, ArMap *map) : ArAction("DistanceReader",
                                                                                              "This action detects and reads the distance between robot and target"),
                                                                                     m_initialized(false),
                                                                                     m_laser(l),
                                                                                     m_tracker(tracker),
                                                                                     m_map(map)

{
    // Compute how many readings the target is long on a circle with radius 1000mm
    m_amountOfReadingsToSkip = m_targetLength / LASER_STEP;
    m_state = herrchenLocalizing;
    // Add a little overshoot to survey beyond the target and make sure the gradient changes
    m_goalOffset = m_amountOfReadingsToSkip + 5;
}

ObjectIdentifier::~ObjectIdentifier() {}

ArActionDesired *ObjectIdentifier::fire(ArActionDesired current)
{
    // Make sure the robot doesn't attempt anything silly or has dirty direct actions once its rotated
    // Important safety to guarantee a stable gradient sampling
    if (myRobot->isHeadingDone())
        myRobot->clearDirectMotion();
    // Initialize and make sure we have everything we need
    if (!m_laser)
    {
        deactivate();
        return 0;
    }
    // Attempt to find the target by sampling the whole sensor area and rotate if it's not in sight
    if (m_state == herrchenLocalizing && myRobot->isHeadingDone())
    {

        m_laser->lockDevice();
        int angle = detectTarget(false);
        // Angle is only 900 if target isn't found
        if (angle != 900)
        {
            myRobot->clearDirectMotion();
            m_action.reset();
            myRobot->setDeltaHeading(angle);
            m_state = herrchenFound;
            m_laser->unlockDevice();
            return &m_action;
        }
        // If no target is found, rotate around the robot
        myRobot->setDeltaHeading(-20);
        m_laser->unlockDevice();
    }
    // If we face the target, detect it again to be sure we're facing it and create ReadingBoxes around it
    // Then disable this behaviour
    if (m_state == herrchenFound && myRobot->isHeadingDone())
    {
        int angle = detectTarget(true);
        // Angle is only 900 if target isn't found
        if (angle != 900)
        {
            myRobot->setDeltaHeading(angle);
            m_state = disabled;
        }
    }

    return &this->m_action;
}

int ObjectIdentifier::detectTarget(bool valid)
{
    if (myRobot->isHeadingDone())
    {
        if (m_laser->isConnected())
        {
            ArPose reading;
            // The start and end points of our ReadingBox
            CVector start(300, 21000);
            CVector end(20000, 20500);
            // The distance at each side is 20000
            int offset = 40000 / m_numReadingBoxes;
            // Used to transform ReadingBox coordinates into world space
            // Needed since the map is in world space
            ArTransform worldTransform = myRobot->getToGlobalTransform();

            for (int i = 0; i < m_numReadingBoxes; ++i)
            {
                m_laser->currentReadingBox(start.x, start.y, end.x, end.y, &reading);
                CVector relativePos(reading.getX(), reading.getY());

                // Move the box 500mm to the right
                start.y -= offset;
                end.y -= offset;

                // Empty readings return (0,0)
                if (relativePos == 0)
                    continue;

                ArPose worldPos = worldTransform.doTransform(reading);
                if (!MapUtilities::isObstacle(worldPos, m_map))
                {
                    double angle = myRobot->findDeltaHeadingTo(worldPos);
                    // Confirm the target through gradients by transforming the angle to a range of 0 - 180
                    if (!detectTargetWithGradient(abs(angle) + 92))
                        break;
                    // Track the target
                    if (valid)
                        m_tracker->SetTarget(relativePos, relativePos.length());

                    m_herrchenDetected = true;
                    return angle;
                }
            }
            return 900;
        }
        else if (m_laser->isTryingToConnect())
        {
            ArLog::log(ArLog::Normal, "Connecting to Laser %s, please wait.",
                       m_laser->getName());
            return 900;
        }
        else
        {
            m_laser->asyncConnect();
            return 900;
        }
    }
    return 900;
}

bool ObjectIdentifier::detectTargetWithGradient(int start)
{
    std::vector<ArSensorReading> *readings;
    readings = m_laser->getRawReadingsAsVector();
    int size = readings->size();

    double initialLength, length, lengthN, lengthBack, refLength;
    int right, back, neighbor, ref;
    ArPose robo = myRobot->getPose();

    // Get current reading
    start = start >= 179 ? 179 : start;
    ArSensorReading reading = readings->at(start);
    CVector x0(reading.getX(), reading.getY());
    initialLength = x0.length();

    // Simple calculation to determine the amount of readings to skip at this distance
    float steps = initialLength / 1000.0f;
    int indicesToSkip = m_targetLength / (LASER_STEP * steps);
    m_goalOffset = indicesToSkip + 4;
    int end = m_goalOffset + start - 1;
    end = end >= 179 ? 179 : end;

    for (int i = start - 1; i < end; i++)
    {
        // Sample the point TargetLength + tolerance after the possible target
        right = i + m_goalOffset >= 179 ? 179 : i + m_goalOffset;
        length = getGradientLengthAtReading(readings, right, m_goalOffset, x0);

        // Repeat for the other two sample points
        // Next up is the one directly after the current one, to have a reference gradient
        neighbor = i + m_neighborOffset >= 179 ? 179 : i + m_neighborOffset;
        lengthN = getGradientLengthAtReading(readings, neighbor, m_neighborOffset, x0);

        // Now the one directly before it
        back = i - m_backOffset < 0 ? 0 : i - m_backOffset;
        lengthBack = getGradientLengthAtReading(readings, back, -m_backOffset, x0);

        // Check if both sampled points have a steep enough gradient to be considered away
        if (length > lengthN * 2 && lengthBack > lengthN * 2)
        {
            // Safety procedure to be absolutely sure
            // Samples every reading in the range m_goalOffset - tolerance to be sure no gradient changes
            int limit = m_goalOffset - (m_goalOffset * 0.4) - 4;
            for (int c = 1; c < limit; ++c)
            {
                int indices = c + i;
                indices = indices >= 179 ? 179 : indices;
                indices = indices < 0 ? 0 : indices;
                double refLen = getGradientLengthAtReading(readings, indices, c, x0);
                if (refLen >= 0 && refLen <= lengthN * 3)
                {
                    continue;
                }
                else
                {
                    return false;
                    break;
                }
            }
            std::cout<<length<<'\n';
            std::cout<<lengthN<<'\n';
            return true;
        }
    }
    return false;
}

double ObjectIdentifier::getGradientLengthAtReading(std::vector<ArSensorReading> *readings, int index, int skippedIndex, CVector &x0) const
{
    ArSensorReading indexReading = readings->at(index);

    CVector x1(indexReading.getX(), indexReading.getY());
    CVector gradient;
    getGradient(skippedIndex, x0, x1, gradient);
    // Get the Length of the gradient
    return gradient.length();
}

void ObjectIdentifier::getGradient(int h, CVector &x0, CVector &x1, CVector &gradient) const
{
    gradient.x = (x1.x - x0.x) / h;
    gradient.y = (x1.y - x0.y) / h;
}
void ObjectIdentifier::setState(HerrchenState s)
{
    m_state = s;
}
