#ifndef ATTACK_GNSS_CONVERT_HPP
#define ATTACK_GNSS_CONVERT_HPP

#include <string>

class GNSSCoords
{
public:
    GNSSCoords(double lat, double lon, double alt, int coordinate_system = 3, int height_system = 1, double origin_lat = 0.0, double origin_lon = 0.0, double origin_alt = 0.0, int plane = 2);

    inline double x() { return _x; }
    void x(double newX);
    inline double y() { return _y; }
    void y(double newY);
    inline double z() { return _z; }
    void z(double newZ);
    inline double lat() { return _lat; }
    void lat(double newLat);
    inline double lon() { return _lon; }
    void lon(double newLon);
    inline double alt() { return _alt; }
    void alt(double newAlt);

    void convert2XYZ();
    void convert2LLA();

private:
    double EllipsoidHeight2OrthometricHeight(double lat, double lon, double alt);
    double OrthometricHeight2EllipsoidHeight(double lat, double lon, double alt);
    void setPlane(int plane);

    int plane;

    int zone = 0;
    bool east_north_up = true;

    int coordinate_system;
    int height_system;
    double _x;
    double _y;
    double _z;
    double _lat;
    double _lon;
    double _alt;

    double origin_lat = 0.0;
    double origin_lon = 0.0;
    double origin_alt = 0.0;
};

#endif