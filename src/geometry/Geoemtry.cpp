/*******************************************************************************
 * Last modified on 1 July 2023
 * Author: Aditya Rauniyar, Krishna Suresh
 * Email: arauniya@andrew.cmu.edu, ksuresh2@andrew.cmu.edu
 * Affiliation: The Robotics Institute, Carnegie Mellon University, Pittsburgh.
 *
 * Copyright (c) 2023.  Aditya Rauniyar, Krishna Suresh.
 ******************************************************************************/

#include "Geometry.h"
#include <iostream>
#include <ranges>

namespace geometry
{

    Pose3D pose3d::fromPositionHeading(Vector3d position, double theta)
    {
        return Translation3d(position) * AngleAxisd(theta, Vector3d::UnitZ());
    }

    Pose3D pose3d::fromXYZRPY(std::array<double, 6> xyzrpy)
    {
        Quaterniond q = Eigen::AngleAxisd(xyzrpy[3], Eigen::Vector3d::UnitX())
                        * Eigen::AngleAxisd(xyzrpy[4], Eigen::Vector3d::UnitY())
                        * Eigen::AngleAxisd(xyzrpy[5], Eigen::Vector3d::UnitZ());

        // Now create the transformation matrix
        Translation3d trans(xyzrpy[0], xyzrpy[1], xyzrpy[2]);
        return trans * q;
    }

    double pose3d::distance(const Pose3D a, const Pose3D b)
    {
        return distance(a.translation(), b.translation());
    }

    double pose3d::distance(const Point3D a, const Point3D b)
    {
        return std::sqrt(std::pow(b.x() - a.x(), 2) + std::pow(b.y() - a.y(), 2)
                         + std::pow(b.z() - a.z(), 2));
    }

    Pose2D pose2d::fromPose3DXY(const Pose3D pose3D)
    {
        Pose2D pose2D;
        pose2D.setIdentity();
        pose2D.linear() = pose3D.linear().block<2, 2>(0, 0);
        pose2D.translation() = pose3D.translation().head<2>();
        return pose2D;
    }

    std::optional<Point3D> rayPlaneIntersect(const Ray3D & ray, const Plane3D plane)
    {
        double denominator = plane.n.dot(ray.direction);
        if(std::abs(denominator) < std::numeric_limits<double>::epsilon())
            return std::nullopt; // ray is parallel

        double t = (plane.n.dot(plane.p - ray.point)) / denominator;
        if(t < 0)
            spdlog::warn("T is negative");
        // TODO t is negative for some reason
        return ray.point + t * ray.direction;
    }

    double triangleSurfaceArea(std::array<Point3D, 3> points)
    {
        double sideA = pose3d::distance(points[0], points[1]);
        double sideB = pose3d::distance(points[1], points[2]);
        double sideC = pose3d::distance(points[2], points[0]);

        double s = (sideA + sideB + sideC) / 2.0;

        return std::sqrt(s * (s - sideA) * (s - sideB) * (s - sideC));
    }

    std::ostream & operator<<(std::ostream & os, const Ray2D & ray)
    {
        os << "Point: " << ray.point.transpose() << "\n";
        os << "Direction: " << ray.direction.transpose() << "\n";
        return os;
    }

    std::ostream & operator<<(std::ostream & os, const Ray3D & ray)
    {
        os << "Point: " << ray.point.transpose() << "\n";
        os << "Direction: " << ray.direction.transpose() << "\n";
        return os;
    }

    bool Ray3D::FuzzyEq(const Ray3D & a, const Ray3D & b, double epsilon)
    {
        return (a.point - b.point).norm() <= epsilon
               && (a.direction - b.direction).norm() <= epsilon;
    }

    bool Ray2D::FuzzyEq(const Ray2D & a, const Ray2D & b, double epsilon)
    {
        return (a.point - b.point).norm() <= epsilon
               && (a.direction - b.direction).norm() <= epsilon;
    }
    namespace polygon
    {
        std::vector<Line2D> Polygon2D::edges() const
        {
            return edges(Pose2D::Identity());
        }

        std::vector<Line2D> Polygon2D::edges(Pose2D pose) const
        {
            Matrix3Xd verticesInPoseFrame(3, vertices.cols());
            verticesInPoseFrame << vertices, Eigen::MatrixXd::Ones(1, vertices.cols());
            verticesInPoseFrame = pose * verticesInPoseFrame;
            std::vector<Line2D> polygonEdges;
            for(int i = 0; i < verticesInPoseFrame.cols(); ++i)
            {
                polygonEdges.push_back(
                    {verticesInPoseFrame.col(i).head<2>(),
                     verticesInPoseFrame.col((i + 1) % verticesInPoseFrame.cols())
                         .head<2>()});
            }
            return polygonEdges;
        }
        // Since these are regular they can be simplified a ton
        // but this works for now
        Rectangle2D::Rectangle2D(double width, double height)
        {
            double halfWidth = width / 2;
            double halfHeight = height / 2;
            vertices = Matrix2Xd(2, 4);
            // clang-format off
    vertices
        << -halfWidth, halfWidth, halfWidth,-halfWidth, 
            halfHeight, halfHeight, -halfHeight, -halfHeight;
            // clang-format on
        }

        RegularPolygon2D::RegularPolygon2D(double radius, int edgeCount)
        {
            vertices = Matrix2Xd(2, edgeCount);

            // Define the six vertices of the hexagon in its own frame
            for(int i = 0; i < 6; ++i)
            {
                double angle =
                    i * M_PI / 3; // Six vertices equally spaced around the circle
                vertices(0, i) = radius * cos(angle); // x-coordinate
                vertices(1, i) = radius * sin(angle); // y-coordinate
            }
        }

        // Face
        Face::Face(int actorId, double surfaceArea)
          : id({actorId, faceIdCounter[actorId]++})
          , _surfaceArea(surfaceArea)
        {
            spdlog::debug("Entity [{}] adding face {} for with surface area {:.3f}.",
                          actorId,
                          faceIdCounter[actorId],
                          surfaceArea);
            if(faceIdCounter[actorId] >= MAX_FACES_PER_POLYGON)
            {
                spdlog::warn(
                    "Polygon [{}] exceeded max face count {} / {}, coverage estimates "
                    "may not be "
                    "accurate.",
                    actorId,
                    faceIdCounter[actorId],
                    MAX_FACES_PER_POLYGON);
            }
        };

        Face::Face()
          : id({-1, -1})
          , _surfaceArea(0){};

        std::unordered_map<int, int> Face::faceIdCounter = {};
        std::unordered_map<FaceId, Face, FaceIdHasher> Face::faceMap = {};
        const Face Face::Invalid = Face();

        const Face & Face::addFace(int actorId, double surfaceArea)
        {
            Face face(actorId, surfaceArea);
            FaceId id = face.id;
            faceMap.insert(std::make_pair(id, std::move(face)));
            return faceMap.at(id);
        }

        // Color picking system inspired by:
        // http://www.opengl-tutorial.org/miscellaneous/clicking-on-objects/picking-with-an-opengl-hack/
        const Face & Face::fromByte(std::array<unsigned char, 3> byte)
        {
            // clang-format off
            int pickedID = byte[0] + 
                        byte[1] * 256 +
                        byte[2] * 256*256;
            // clang-format on
            int actorId = pickedID / MAX_FACES_PER_POLYGON;
            int face = pickedID % MAX_FACES_PER_POLYGON;
            if(Face::faceMap.find({actorId, face}) == Face::faceMap.end())
                return Face::Invalid;
            return Face::faceMap.at({actorId, face});
        }

        std::array<float, 3> Face::toRGBf() const
        {
            int i = id[0] * MAX_FACES_PER_POLYGON + id[1];
            int r = (i & 0x000000FF) >> 0;
            int g = (i & 0x0000FF00) >> 8;
            int b = (i & 0x00FF0000) >> 16;
            return {(float)r / 255.0f, (float)g / 255.0f, (float)b / 255.0f};
        }

        std::vector<float> Polygon3D::getVertexData()
        {
            // for every face get 3 vertex and the associated color
            Matrix<float, -1, -1, RowMajor> vertexData(faces.size(),
                                                       vertices.cols() + colors.cols());
            for(int i = 0; i < faces.rows(); i++)
            {
                auto faceColor = colors.row(i);
                for(int j = 0; j < 3; j++)
                    vertexData.row(i * 3 + j) << vertices.row(faces(i, j)), faceColor;
            }
            std::size_t sz = vertexData.innerSize() * vertexData.outerSize();
            std::vector<float> ret(sz);
            float * tmp = vertexData.data();
            std::copy(tmp, tmp + sz, ret.begin());
            return ret;
        };
        RectangularPrism::RectangularPrism(int entityId, float width, float length,
                                           float height, ColorMode mode)
          : Polygon3D(mode)
        {
            dimensions = {width, length, height};
            vertices = Matrix<float, -1, 3>(8, 3);
            faces = Matrix<unsigned int, -1, 3>(12, 3);
            colors = Matrix<float, -1, 3>(12, 3);
            colors.setZero();
            // clang-format off
        vertices << -width/2, -length/2,  0,
                     width/2, -length/2,  0,
                     width/2,  length/2,  0,
                    -width/2,  length/2,  0,
                    -width/2, -length/2,  height,
                     width/2, -length/2,  height,
                     width/2,  length/2,  height,
                    -width/2,  length/2,  height;
        faces << 0, 1, 2,
                 2, 3, 0,
                 0, 4, 7,
                 7, 3, 0,
                 0, 1, 5,
                 5, 4, 0,
                 1, 5, 6,
                 6, 1, 2,
                 3, 7, 6,
                 6, 3, 2,
                 4, 5, 6,
                 6, 7, 4;
            // clang-format on
            if(_colorMode == ColorMode::WHITE)
                colors.setOnes();
            else if(_colorMode == ColorMode::RANDOM)
            {
                colors.setRandom();
                colors = (colors + Eigen::MatrixXf::Ones(12, 3)) / 2.0;
            }
            else if(_colorMode == ColorMode::BLUEID)
            {

                auto rgb = hsv2rgb(hsv(entityId == 0? 0 : 250, 0.83, 0.7));
                // auto rgb = hsv2rgb(hsv((((float)(entityId - 2)) / 3.0) * 90, 0.83, 0.7));
                    // std::cout << rgb.r << " " << rgb.g << " " <<rgb.b << " " << "\n";
                for(int i = 0; i < 12; ++i)
                {
                    colors(i, 0) = rgb.r;
                    colors(i, 1) = rgb.g;
                    colors(i, 2) = rgb.b;
                }
            }
            else if(_colorMode == ColorMode::REDSHADE)
            {
                for(int i = 0; i < 12; ++i)
                {
                    auto rgb = hsv2rgb(hsv(350, 0.83, ((float)(rand() % 100)) / 100.0));
                    colors(i, 0) = rgb.r;
                    colors(i, 1) = rgb.g;
                    colors(i, 2) = rgb.b;
                }
            }
            else if(_colorMode == ColorMode::PICKING)
            {
                for(int i = 0; i < 12; ++i)
                {
                    std::array<Point3D, 3> p;
                    for(int t_i = 0; t_i < 3; t_i++)
                        p[t_i] = vertices.row(faces.row(i)[t_i]).cast<double>();
                    const Face & face = Face::addFace(entityId, triangleSurfaceArea(p));
                    std::array<float, 3> rgb = face.toRGBf();
                    colors.row(i) << rgb[0], rgb[1], rgb[2];
                }
            }
            else
            {
                spdlog::error("ColorMode not set, value is [{}]", (int)_colorMode);
            }
        };

        Polygon2D RectangularPrism::get2DPolygon() const
        {
            return Rectangle2D(std::get<0>(dimensions), std::get<1>(dimensions));
        }
    } // namespace polygon
} // namespace geometry