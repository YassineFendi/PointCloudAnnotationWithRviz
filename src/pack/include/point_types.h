/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne_pointcloud
{
  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  /*
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    uint16_t    intensity;                 ///< laser intensity reading
    float    distanceXY;                  ///< laser distance
    uint16_t ring;                     ///< laser theta
    uint16_t label;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment

    inline PointXYZIR ()
    {
      label = ring = distanceXY = intensity = x = y = z = 0.0f;
    }

   inline PointXYZIR (float _x, float _y, float _z)
    {
      x = _x; y = _y; z = _z;
      distanceXY = sqrt(pow(_x,2)+pow(_y,2));
      intensity = label = ring = 0;
    }

  } EIGEN_ALIGN16;
  */

  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIR
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    uint16_t    intensity;                 ///< laser intensity reading
    float    distanceXY;                  ///< laser distance
    uint16_t ring;                     ///< laser theta
    uint8_t dynamic;                  /// if dynamic != 0, it's dynamic (can't add bool value)
    uint16_t label;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment

    inline PointXYZIR ()
    {
      label = ring = distanceXY = dynamic = intensity = x = y = z = 0.0f;
    }

   inline PointXYZIR (float _x, float _y, float _z)
    {
      x = _x; y = _y; z = _z;
      distanceXY = sqrt(pow(_x,2)+pow(_y,2));
      intensity = label = dynamic = ring = 0;
    }

  } EIGEN_ALIGN16;

  /** XYZ, RXY, Ith (in point_mat_tp), Jth (in point_mat_tp), T (Intensity). */
  struct Point8D
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    rxy;                       /// XY distance
    uint16_t    ith;                    // row number in point_mat_tp
    uint16_t    jth;                    // column number in point_mat_tp
    uint16_t    intensity;                 ///< laser intensity reading
    uint16_t    id;                     //id of the clustering
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment

    inline Point8D()
    {
      rxy = x = y = z =  0.0f;
      jth = ith = intensity = id = 0;
    }

   inline Point8D (float _x, float _y, float _z)
    {
      x = _x; y = _y; z = _z;
      rxy = sqrt(pow(_x,2)+pow(_y,2));
      intensity = ith = jth = 0;
      id = 0;
    }

   inline Point8D (float _x, float _y, float _z, float _rxy, int _ith, int _jth, int _intensity, int _id)
    {
      x = _x; y = _y; z = _z;
      rxy = _rxy;
      intensity = _intensity;
      ith = _ith;
      jth = _jth;
      id = _id;
    }
  } EIGEN_ALIGN16;


  /** Euclidean Velodyne coordinate, including intensity and ring number. */
  struct PointXYZIL
  {
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float    intensity;                 ///< laser intensity reading
    uint16_t ring;                      ///< laser ring number
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
  } EIGEN_ALIGN16;
}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIL,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, distanceXY, distanceXY)
                                  (uint16_t, intensity, intensity)
                                  (uint16_t, label, label)
                                  (uint8_t, dynamic, dynamic)
                                  (uint16_t, ring, ring))

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::Point8D,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, rxy, rxy)
                                  (uint16_t, ith, ith)
                                  (uint16_t, jth, jth)
                                  (uint16_t, intensity, intensity)
                                  (uint16_t, id, id))

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

