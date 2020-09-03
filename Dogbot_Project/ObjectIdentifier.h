#ifndef DISTANCEREADER_H
#define DISTANCEREADER_H

#include <Aria.h>
#include <ArAction.h>
#include "CUtils.h"
#include "ObjectTracker.h"
#include "MapUtilities.h"
#include <ArMutex.h>

// Amount of mm that lie between each laser sensor reading step in a circle with 1000mm radius
#define LASER_STEP 17

/**
 * Class that handles the detection of the target object
 * Employs a map and gradient based procedure to identify an object of roughly the given size
 * Symbolizes the entry point of the program and can rotate the robot to survey the surroundings
 */
class ObjectIdentifier : public ArAction
{
public:
    /*
   * Central constructor of the class 
   * @param l The ArLaser object since all functions work with the far more precise laser
   * @param tracker The ObjectTracker since the ObjectIdentifier also triggers the creation of reading boxes once the target is found
   * @param map The current ArMap since the detection requires it
   */
    ObjectIdentifier(ArLaser *l, ObjectTracker *tracker, ArMap *map);

    ~ObjectIdentifier();

    ArActionDesired *fire(ArActionDesired currentAction);

    void getSensorReading(int index, CVector &position);

    // Helper functions
    bool isHerrchenDetected() { return m_herrchenDetected; }

    /**
     * Returns the length of the gradient at the desired reading
     * @param readings The Vector of all raw LaserSensorReadings
     * @param index The index of the reading to compute the gradient for
     * @param skippedIndex The amount of skipped indices to the original reading
     * @param x0 The positions of the original reading
     */
    double getGradientLengthAtReading(std::vector<ArSensorReading> *readings, int index, int skippedIndex, CVector &x0) const;

    /**
     * Computes the gradient between two given positions
     * @param h The amount of readings between the two positions
     * @param x0 The first position
     * @param x1 The second position
     * @param gradient The CVector to write the gradient into
     */
    void getGradient(int h, CVector &x0, CVector &x1, CVector &gradient) const;

    // Central methods to detect the target
    /**
     * Detects the target by comparing different ReadingBox positions with the map and returns the angle to rotate the robot by to face it directly
     * Returns 900 if the target isn't found
     * @param valid A bool to indicate whether ReadingBoxes should be placed at the found Target or simply return the angle
     */
    int detectTarget(bool valid);
     /**
     * Detects the target by trying to determine its rough shape and size through gradients, mainly used to double check the map based procedure
     * Returns true if the found reading is the target
     * @param start The rough index of the raw SensorReading the supposed target is located at
     */
    bool detectTargetWithGradient(int start);
    void setState(HerrchenState s);

private:
    ArLaser *m_laser;
    ArActionDesired m_action;
    ObjectTracker *m_tracker;
    ArMap *m_map;

    bool m_initialized;
    bool m_herrchenDetected;

    ArPose m_readingPose;

    // Length of target in mm
    float m_targetLength = 600;
    const int m_numReadingBoxes = 100;
    // Helper variables for the gradient procedure
    int m_amountOfReadingsToSkip;
    const int m_neighborOffset = 2;
    const int m_backOffset = 5;
    int m_goalOffset;

    // Used to control the sequence of events
    HerrchenState m_state;
};

#endif
