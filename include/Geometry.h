/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#pragma once
#ifndef VISUALGROUPCOVERAGEPLANNER_GEOMETRY_H
#define VISUALGROUPCOVERAGEPLANNER_GEOMETRY_H

#include "Logging.h"
#include <Eigen/Geometry>
#include <array>
#include <functional>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>

#define EPSILON 0.00001
#define MAX_FACES_PER_POLYGON 5000

namespace geometry
{
    using namespace Eigen;

    // Geometry typedefs
    typedef Affine2d Pose2D;
    typedef Vector2d Point2D;

    typedef Affine3d Pose3D;
    typedef Vector3d Point3D;
    struct Plane3D
    {
        Point3D n;
        Point3D p;
    };

    // Pose3D helpers
    namespace pose3d
    {
        inline void rotateX(Pose3D & pose, double radians)
        {
            pose.rotate(AngleAxisd(radians, Vector3d::UnitX()));
        };
        inline void rotateY(Pose3D & pose, double radians)
        {
            pose.rotate(AngleAxisd(radians, Vector3d::UnitY()));
        };
        inline void rotateZ(Pose3D & pose, double radians)
        {
            pose.rotate(AngleAxisd(radians, Vector3d::UnitZ()));
        };

        double distance(const Pose3D a, const Pose3D b);
        double distance(const Point3D a, const Point3D b);

        Pose3D fromPositionHeading(Vector3d position, double theta);

        Pose3D fromXYZRPY(std::array<double, 6> xyzrpy);
    } // namespace pose3d

    // Pose2D helpers
    namespace pose2d
    {
        Pose2D fromPose3DXY(const Pose3D pose3D);
    }

    // Default Planes
    inline Plane3D xyPlane()
    {
        return Plane3D({0, 0, 1}, {0, 0, 0});
    };

    inline Plane3D yzPlane()
    {
        return Plane3D({1, 0, 0}, {0, 0, 0});
    };

    inline Plane3D zxPlane()
    {
        return Plane3D({0, 1, 0}, {0, 0, 0});
    };

    // Ray
    struct Ray2D
    {
        Point2D point;
        Point2D direction;
        bool operator==(const Ray2D & rhs) const
        {
            return point == rhs.point && direction == rhs.direction;
        }
        static bool FuzzyEq(const Ray2D & a, const Ray2D & b, double epsilon = EPSILON);
    };

    std::ostream & operator<<(std::ostream & os, const Ray2D & ray);

    struct Ray3D
    {
        Point3D point;
        Point3D direction;
        bool operator==(const Ray3D & rhs) const
        {
            return point == rhs.point && direction == rhs.direction;
        }
        static bool FuzzyEq(const Ray3D & a, const Ray3D & b, double epsilon = EPSILON);
    };

    std::ostream & operator<<(std::ostream & os, const Ray3D & ray);

    // Line
    struct Line2D
    {
        Point2D start;
        Point2D end;
    };

    struct Line3D
    {
        Point3D start;
        Point3D end;
    };

    // Basic Geometry Math Helpers
    std::optional<Point3D> rayPlaneIntersect(const Ray3D & ray, const Plane3D plane);
    double triangleSurfaceArea(std::array<Point3D, 3> points);

    // All polygon and mesh related info
    namespace polygon
    {

        struct Polygon2D
        {
            Matrix2Xd vertices;
            std::vector<Line2D> edges() const;
            std::vector<Line2D> edges(Pose2D worldPose) const;
        };
        struct RegularPolygon2D : Polygon2D
        {
            RegularPolygon2D(double radius, int edgeCount);
        };

        struct Rectangle2D : Polygon2D
        {
            Rectangle2D(double width, double height);
        };

        // FaceId is a pair of ActorId and FaceIndex
        typedef std::array<int, 2> FaceId;

        // Hasher to use FaceId as a key for unordered_map
        struct FaceIdHasher
        {
            std::size_t operator()(const std::array<int, 2> & a) const
            {
                std::size_t h = 0;

                for(auto e : a)
                {
                    h ^= std::hash<int>{}(e) + 0x9e3779b9 + (h << 6) + (h >> 2);
                }
                return h;
            }
        };

        // Face struct container
        struct Face
        {
            const FaceId id;           // actor and face unique id
            const double _surfaceArea; // face SA of face

            static const Face Invalid; // instance of invalid face
            static std::unordered_map<int, int> faceIdCounter;
            static std::unordered_map<FaceId, Face, FaceIdHasher> faceMap;

            // get Face from rgb bytes
            static const Face & fromByte(std::array<unsigned char, 3> byte);
            // create new face and add to lookup list
            static const Face & addFace(int actorId, double surfaceArea);

            Face();
            Face(const Face &) = delete;
            Face(Face &&) = default;
            Face(int actorId, double surfaceArea);

            inline bool isInvalid() const
            {
                return id[0] == -1;
            }
            // Get rgb color from face id
            std::array<float, 3> toRGBf() const;
        };

        enum ColorMode
        {
            BLACK,
            WHITE,
            PICKING,
            RANDOM,
            REDSHADE,
            GRAYSCALE,
            BLUEID

        };

        // TODO should be moved to a color header
        typedef struct
        {
            double r; // a fraction between 0 and 1
            double g; // a fraction between 0 and 1
            double b; // a fraction between 0 and 1
        } rgb;

        typedef struct
        {
            double h; // angle in degrees
            double s; // a fraction between 0 and 1
            double v; // a fraction between 0 and 1
        } hsv;

        static hsv rgb2hsv(rgb in);
        static rgb hsv2rgb(hsv in);

        hsv rgb2hsv(rgb in)
        {
            hsv out;
            double min, max, delta;

            min = in.r < in.g ? in.r : in.g;
            min = min < in.b ? min : in.b;

            max = in.r > in.g ? in.r : in.g;
            max = max > in.b ? max : in.b;

            out.v = max; // v
            delta = max - min;
            if(delta < 0.00001)
            {
                out.s = 0;
                out.h = 0; // undefined, maybe nan?
                return out;
            }
            if(max > 0.0)
            { // NOTE: if Max is == 0, this divide would cause a crash
                out.s = (delta / max); // s
            }
            else
            {
                // if max is 0, then r = g = b = 0
                // s = 0, h is undefined
                out.s = 0.0;
                out.h = NAN; // its now undefined
                return out;
            }
            if(in.r >= max)                    // > is bogus, just keeps compilor happy
                out.h = (in.g - in.b) / delta; // between yellow & magenta
            else if(in.g >= max)
                out.h = 2.0 + (in.b - in.r) / delta; // between cyan & yellow
            else
                out.h = 4.0 + (in.r - in.g) / delta; // between magenta & cyan

            out.h *= 60.0; // degrees

            if(out.h < 0.0)
                out.h += 360.0;

            return out;
        }

        rgb hsv2rgb(hsv in)
        {
            double hh, p, q, t, ff;
            long i;
            rgb out;

            if(in.s <= 0.0)
            { // < is bogus, just shuts up warnings
                out.r = in.v;
                out.g = in.v;
                out.b = in.v;
                return out;
            }
            hh = in.h;
            if(hh >= 360.0)
                hh = 0.0;
            hh /= 60.0;
            i = (long)hh;
            ff = hh - i;
            p = in.v * (1.0 - in.s);
            q = in.v * (1.0 - (in.s * ff));
            t = in.v * (1.0 - (in.s * (1.0 - ff)));

            switch(i)
            {
            case 0:
                out.r = in.v;
                out.g = t;
                out.b = p;
                break;
            case 1:
                out.r = q;
                out.g = in.v;
                out.b = p;
                break;
            case 2:
                out.r = p;
                out.g = in.v;
                out.b = t;
                break;

            case 3:
                out.r = p;
                out.g = q;
                out.b = in.v;
                break;
            case 4:
                out.r = t;
                out.g = p;
                out.b = in.v;
                break;
            case 5:
            default:
                out.r = in.v;
                out.g = p;
                out.b = q;
                break;
            }

            if (out.r < 0)
                out.r = 1+out.r;
            if (out.g < 0)
                out.g = 1+out.g;
            if (out.b < 0)
                out.b = 1+out.b;

            return out;
        }
        struct Polygon3D
        {
            Matrix<float, -1, 3> vertices;
            Matrix<unsigned int, -1, 3, RowMajor> faces;
            Matrix<float, -1, 3> colors;
            const ColorMode _colorMode;

            Polygon3D(const Polygon3D &) = default;
            Polygon3D(Polygon3D &&) = default;
            Polygon3D(ColorMode colorMode)
              : _colorMode(colorMode){};

            virtual Polygon2D get2DPolygon() const = 0;

            std::vector<float> getVertexData();
        };

        struct RectangularPrism : Polygon3D
        {
            std::tuple<float, float, float> dimensions; // width, length, height
            RectangularPrism(int entityId, float width, float length, float height,
                             ColorMode mode);
            Polygon2D get2DPolygon() const;
        };
    } // namespace polygon

} // namespace geometry

#endif // VISUALGROUPCOVERAGEPLANNER_GEOMETRY_H