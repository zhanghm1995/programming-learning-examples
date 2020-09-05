
#ifndef PROJECT_POINT_TYPE_DEFINE_H
#define PROJECT_POINT_TYPE_DEFINE_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>

#define PCL_NO_PRECOMPILE

namespace perception {

struct EIGEN_ALIGN16 _PointXYZR
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    uint32_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};

/**
 * @brief A point structure representing Euclidean xyz coordinates, and the ring value.
 */
struct EIGEN_ALIGN16 PointXYZR : public _PointXYZR
{
    inline PointXYZR (const _PointXYZR &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        ring = p.ring;
    }

    inline PointXYZR ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        ring = 0;
    }

    inline PointXYZR (float _x, float _y, float _z, uint32_t ring)
    {
        x = _x; y = _y; z = _z;
        data[3] = 1.0f;
        ring = ring;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZR& p)
    {
        os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.ring << ")";
        return (os);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

struct EIGEN_ALIGN16 _PointXYZIR
{
    PCL_ADD_POINT4D;                  // preferred way of adding a XYZ+padding
    float intensity;
    uint32_t ring;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
};

/**
 * @brief A point structure representing Euclidean xyz coordinates, and the ring value.
 */
struct EIGEN_ALIGN16 PointXYZIR : public _PointXYZIR
{
    inline PointXYZIR (const _PointXYZIR &p)
    {
        x = p.x; y = p.y; z = p.z; data[3] = 1.0f;
        intensity = p.intensity;
        ring = p.ring;
    }

    inline PointXYZIR ()
    {
        x = y = z = 0.0f;
        data[3] = 1.0f;
        ring = 0;
        ring = 0;
    }

    inline PointXYZIR (float _x, float _y, float _z, float _intensity, uint32_t _ring)
    {
        x = _x; y = _y; z = _z; 
        data[3] = 1.0f;
        intensity = _intensity;
        ring = _ring;
    }

    friend std::ostream& operator << (std::ostream& os, const PointXYZIR& p)
    {
        os << "(" << p.x << "," << p.y << "," << p.z << " - " << p.ring << ")";
        return (os);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace perception

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::PointXYZR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (uint32_t, ring, ring)
)

POINT_CLOUD_REGISTER_POINT_STRUCT(perception::PointXYZIR,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (uint32_t, ring, ring)
)
#endif //PROJECT_POINT_TYPE_DEFINE_H
