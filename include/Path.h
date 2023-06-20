#ifndef _WAYPOINT_H_
#define _WAYPOINT_H_

class Path {
    protected:
        int from = 0;
        int to = 0;
        double distance = 0;
    
    public:
        bool completed = false;
        Path() = default;
        Path(const int from, const int to, const double distance);
        int start();
        int end();
        double getDistance();
        Path &operator=(const Path &source);
};

Path::Path(const int from, const int to, const double distance) {
    this->from = from;
    this->to = to;
    this->distance = distance;
    this->completed = false;
}

int Path::start() {
    return this->from;
}

int Path::end() { 
    return this->to; 
}

double Path::getDistance() {
    return this->distance;
}

Path &Path::operator=(const Path &source) {
    this->from = source.from;
    this->to = source.to;
    this->distance = source.distance;
    this->completed = source.completed;

    return *this;
}
#endif