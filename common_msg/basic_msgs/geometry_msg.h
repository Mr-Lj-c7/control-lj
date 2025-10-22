#ifndef GEOMETRY_MSG_H
#define GEOMETRY_MSG_H

namespace control {
namespace common_msg {

typedef struct POINT_ENU
{
  double x;  // East from the origin, in meters.
  double y;  // North from the origin, in meters.
  double z;  // Up from the WGS-84 ellipsoid, in meters.
} PointENU;

typedef struct POINT_LLH
{
  // Longitude in degrees, ranging from -180 to 180.
 double lon;
  // Latitude in degrees, ranging from -90 to 90.
 double lat;
  // WGS-84 ellipsoid height in meters.
 double height;
} PointLLH;

typedef struct POINT_2D
{
  double x;  
  double y;  
} Point2D;

typedef struct POINT_3D
{
  double x;  
  double y;  
  double z;  
} Point3D;

typedef struct QUATERNION
{
  double qx;
  double qy;
  double qz;
  double qw;
} Quaternion;

// A general polygon, points are counter clockwise
typedef struct POLYGON {
  Point3D point;
} Polygon;


} // common_msg
} // control

#endif