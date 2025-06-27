#include "attack_gnss/gnss_convert.hpp"
#include <iostream>

#include <GeographicLib/LocalCartesian.hpp>
#include <GeographicLib/MGRS.hpp>
#include <GeographicLib/UTMUPS.hpp>
#include <GeographicLib/Geoid.hpp>

#include <geo_pos_conv/geo_pos_conv.hpp>

static const std::string MAP_TYPE_MGRS = "MGRS";
static const std::string MAP_TYPE_UTM = "UTM";
static const std::string MAP_TYPE_LOCAL = "local";

GNSSCoords::GNSSCoords(double lat, double lon, double alt, int coordinate_system, int height_system, double origin_lat, double origin_lon, double origin_alt, int plane)
{
    _lat = lat;
    _lon = lon;
    _alt = alt;

    this->plane = plane;
    
    this->coordinate_system = coordinate_system;
    this->height_system = height_system;

    this->origin_lat = origin_lat;
    this->origin_lon = origin_lon;

    if (coordinate_system == 4 && height_system == 0)
    {
        this->origin_alt = EllipsoidHeight2OrthometricHeight(origin_lat, origin_lon, 0.0);
    } 
    else {
        this->origin_alt = origin_alt;
    }
}

void GNSSCoords::x(double newX)
{
    _x = newX;
}

void GNSSCoords::y(double newY)
{
    _y = newY;
}

void GNSSCoords::z(double newZ)
{
    _z = newZ;
}

void GNSSCoords::lat(double newLat)
{
    _lat = newLat;

    convert2XYZ();
}

void GNSSCoords::lon(double newLon)
{
    _lon = newLon;
}

void GNSSCoords::alt(double newAlt)
{
    _alt = newAlt;
}

void GNSSCoords::convert2XYZ()
{
    if (coordinate_system == 0)
    {
        std::cerr << "GPS Attack converter not implemented for UTM" << std::endl;
        // GeographicLib::UTMUPS::Forward(_lat, _lon, zone, east_north_up, _x, _y);
        // if (height_system == 0) {
        //     _z = EllipsoidHeight2OrthometricHeight(_lat, _lon, _alt);
        // } else {
        //     _z = _alt;
        // }
    }
    else if (coordinate_system == 1)
    {
        std::cerr << "GPS Attack converter not implemented for MGRS" << std::endl;
    }
    else if (coordinate_system == 2)
    {
        setPlane(plane);
        try {
            GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
            localCartesian_origin.Forward(_lat, _lon, _alt, _x, _y, _z);
        } catch (const GeographicLib::GeographicErr & err) {
            std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        }
    }
    else if (coordinate_system == 3)
    {
        try {
            GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
            localCartesian_origin.Forward(_lat, _lon, _alt, _x, _y, _z);
        } catch (const GeographicLib::GeographicErr & err) {
            std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        }
    }
    else if (coordinate_system == 4)
    {
        std::cerr << "GPS Attack converter not implemented for local cartesian UTM" << std::endl;
        // try {
        //     GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
        //     localCartesian_origin.Forward(_lat, _lon, _alt, _x, _y, _z);
        // } catch (const GeographicLib::GeographicErr & err) {
        //     std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        // }
    }
    else {
        std::cerr << "Unknown gnss coordinate system" << std::endl;
    }
}

void GNSSCoords::convert2LLA()
{
    if (coordinate_system == 0)
    {
        std::cerr << "GPS Attack converter not implemented for UTM" << std::endl;
        // GeographicLib::UTMUPS::Reverse(zone, east_north_up, _x, _y, _lat, _lon);
        // if (height_system == 0) {
        //     _alt = OrthometricHeight2EllipsoidHeight(_lat, _lon, _z);
        // } 
    }
    else if (coordinate_system == 1)
    {
        std::cerr << "GPS Attack converter not implemented for MGRS" << std::endl;
    }
    else if (coordinate_system == 2)
    {
        setPlane(plane);
        try {
            GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
            localCartesian_origin.Reverse(_x, _y, _z, _lat, _lon, _alt);
        } catch (const GeographicLib::GeographicErr & err) {
            std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        }
        _z = EllipsoidHeight2OrthometricHeight(_lat, _lon, _alt);
    }
    else if (coordinate_system == 3)
    {
        try {
            GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
            localCartesian_origin.Reverse(_x, _y, _z, _lat, _lon, _alt);
        } catch (const GeographicLib::GeographicErr & err) {
            std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        }
    }
    else if (coordinate_system == 4)
    {
        std::cerr << "GPS Attack converter not implemented for local cartesian UTM" << std::endl;
        // try {
        //     GeographicLib::LocalCartesian localCartesian_origin(origin_lat, origin_lon, origin_alt);
        //     localCartesian_origin.Forward(_x, _y, _z, _lat, _lon, _alt);
        // } catch (const GeographicLib::GeographicErr & err) {
        //     std::cerr << "Failed to convert NavSatFix to LocalCartesian" << err.what() << std::endl;
        // }
    }
    else {
        std::cerr << "Unknown gnss coordinate system" << std::endl;
    }
}

double GNSSCoords::EllipsoidHeight2OrthometricHeight(double lat, double lon, double alt)
{
  double OrthometricHeight{0.0};
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    OrthometricHeight = egm2008.ConvertHeight(
      lat, lon, alt,
      GeographicLib::Geoid::ELLIPSOIDTOGEOID);
  } catch (const GeographicLib::GeographicErr & err) {
    std::cerr << "Failed to convert Height from Ellipsoid to Orthometric" << err.what() << std::endl;
  }
  return OrthometricHeight;
}

double GNSSCoords::OrthometricHeight2EllipsoidHeight(double lat, double lon, double alt)
{
  double OrthometricHeight{0.0};
  try {
    GeographicLib::Geoid egm2008("egm2008-1");
    OrthometricHeight = egm2008.ConvertHeight(
      lat, lon, alt,
      GeographicLib::Geoid::GEOIDTOELLIPSOID);
  } catch (const GeographicLib::GeographicErr & err) {
    std::cerr << "Failed to convert Height from Ellipsoid to Orthometric" << err.what() << std::endl;
  }
  return OrthometricHeight;
}

void GNSSCoords::setPlane(int num)
{
  int lon_deg, lon_min, lat_deg, lat_min;  // longitude and latitude of origin of each plane in Japan
  if (num == 1) {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 129;
    lat_min = 30;
  } else if (num == 2) {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  } else if (num == 3) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 132;
    lat_min = 10;
  } else if (num == 4) {
    lon_deg = 33;
    lon_min = 0;
    lat_deg = 133;
    lat_min = 30;
  } else if (num == 5) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 134;
    lat_min = 20;
  } else if (num == 6) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  } else if (num == 7) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 137;
    lat_min = 10;
  } else if (num == 8) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 138;
    lat_min = 30;
  } else if (num == 9) {
    lon_deg = 36;
    lon_min = 0;
    lat_deg = 139;
    lat_min = 50;
  } else if (num == 10) {
    lon_deg = 40;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 50;
  } else if (num == 11) {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 140;
    lat_min = 15;
  } else if (num == 12) {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 15;
  } else if (num == 13) {
    lon_deg = 44;
    lon_min = 0;
    lat_deg = 144;
    lat_min = 15;
  } else if (num == 14) {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 142;
    lat_min = 0;
  } else if (num == 15) {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 127;
    lat_min = 30;
  } else if (num == 16) {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 124;
    lat_min = 0;
  } else if (num == 17) {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 131;
    lat_min = 0;
  } else if (num == 18) {
    lon_deg = 20;
    lon_min = 0;
    lat_deg = 136;
    lat_min = 0;
  } else if (num == 19) {
    lon_deg = 26;
    lon_min = 0;
    lat_deg = 154;
    lat_min = 0;
  } else {
    lon_deg = 0;
    lon_min = 0;
    lat_deg = 0;
    lat_min = 0;
  }

  origin_lon = lon_deg;
  origin_lat = lat_deg; 
}
