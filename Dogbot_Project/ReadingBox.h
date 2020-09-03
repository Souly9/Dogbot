// Utility class to store ReadingBox data
class ReadingBox
{
public:
    ReadingBox()
    {
        x1 = 0;
        x2 = 0;
        y1 = 0;
        y2 = 0;
    }
    ReadingBox(double x1, double y1, double x2, double y2)
    {
        this->x1 = x1;
        this->x2 = x2;
        this->y1 = y1;
        this->y2 = y2;
    }

    ~ReadingBox() {}

    void moveUp(double diff)
    {
        x1 += diff;
        x2 += diff;
    }

    double x1, x2, y1, y2;
};
