/*
 * This file is part of the IndoorGraphLocalization distribution (https://github.com/LRMPUT/IndoorGraphLocalization).
 * Copyright (c) 2018 Michał Nowicki (michal.nowicki@put.poznan.pl)
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef G2O_EDGE_WALL_H
#define G2O_EDGE_WALL_H

#include "g2o/core/base_binary_edge.h"
#include "g2o/core/base_multi_edge.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"

#include "vertex_one.h"
#include "wifiLocalization.h"

using namespace std;
using namespace g2o;


namespace g2o {

    class G2O_TYPES_SLAM2D_API EdgeWall : public BaseBinaryEdge<1, Vector1, VertexSE2, VertexSE2>
    {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeWall();


        virtual void computeError();

        virtual bool read(std::istream& is);
        virtual bool write(std::ostream& os) const;

        virtual double initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate);

        std::vector< Wall > walls;

        struct Point
        {
            double x;
            double y;
        };

// Given three colinear points p, q, r, the function checks if
// point q lies on line segment 'pr'
        bool onSegment(Point p, Point q, Point r)
        {
            if (q.x <= max(p.x, r.x) && q.x >= min(p.x, r.x) &&
                q.y <= max(p.y, r.y) && q.y >= min(p.y, r.y))
                return true;

            return false;
        }

// To find orientation of ordered triplet (p, q, r).
// The function returns following values
// 0 --> p, q and r are colinear
// 1 --> Clockwise
// 2 --> Counterclockwise
        int orientation(Point p, Point q, Point r)
        {
            // See https://www.geeksforgeeks.org/orientation-3-ordered-points/
            // for details of below formula.
            double val = (q.y - p.y) * (r.x - q.x) -
                      (q.x - p.x) * (r.y - q.y);

            if (fabs(val) < 0.03) return 0;  // colinear if slope angles differ less by ~ 2 degrees

            return (val > 0)? 1: 2; // clock or counterclock wise
        }


        bool doIntersect(Wall wall, const VertexSE2 * P1, VertexSE2 * P2) {
            // Wall from start to end
            Point w1 = {wall.startX, wall.startY};
            Point w2 = {wall.endX, wall.endY};

            // Two consequitive user poses
            Point p1 = {P1->estimate()[0], P1->estimate()[1]};
            Point p2 = {P2->estimate()[0], P2->estimate()[1]};

            // User crosses the wall
            return  doIntersect(w1, w2, p1, p2);
        }

// The main function that returns true if line segment 'p1q1'
// and 'p2q2' intersect.
        bool doIntersect(Point p1, Point q1, Point p2, Point q2)
        {
//            p1.x = 67.3033;
//            p1.y = 22.3198;
//            q1.x = 54.2185;
//            q1.y = 5.59373;
//
//            p2.x = 67.6377;
//            p2.y = 23.4431;
//            q2.x = 68.5082;
//            q2.y = 23.518;

            double lineLength = sqrt(pow(p2.x - q2.x,2) + pow(p2.y - q2.y,2));
//            std::cout << "Linelength: " << lineLength << std::endl;
            if (lineLength < 0.01) {
                double distance = distanceToLine(p1, q1, p2);

                if (distance < 0.01)
                    return true;
                return false;
            }

            // Find the four orientations needed for general and
            // special cases
            int o1 = orientation(p1, q1, p2);
            int o2 = orientation(p1, q1, q2);
            int o3 = orientation(p2, q2, p1);
            int o4 = orientation(p2, q2, q1);

//            std::cout << o1 << " " << o2 << " "<< o3 << " "<< o4 << std::endl;

            // General case
            if (o1 != o2 && o3 != o4)
                return true;

            // Special Cases
            // p1, q1 and p2 are colinear and p2 lies on segment p1q1
            if (o1 == 0 && onSegment(p1, p2, q1)) return true;

            // p1, q1 and p2 are colinear and q2 lies on segment p1q1
            if (o2 == 0 && onSegment(p1, q2, q1)) return true;

            // p2, q2 and p1 are colinear and p1 lies on segment p2q2
            if (o3 == 0 && onSegment(p2, p1, q2)) return true;

            // p2, q2 and q1 are colinear and q1 lies on segment p2q2
            if (o4 == 0 && onSegment(p2, q1, q2)) return true;

            return false; // Doesn't fall in any of the above cases
        }

        double distanceToLine(Wall wall, g2o::VertexSE2 *P1, g2o::VertexSE2 *P2) {

            // Wall from start to end
            Point w1 = {wall.startX, wall.startY};
            Point w2 = {wall.endX, wall.endY};

            // Two consequitive user poses
            Point p1 = {P1->estimate()[0], P1->estimate()[1]};
            Point p2 = {P2->estimate()[0], P2->estimate()[1]};

            // Distance of first and second pose
            double d1 = distanceToLine(w1, w2, p1);
            double d2 = distanceToLine(w1, w2, p2);

            return min(d1, d2);
        }


        double distanceToLine(Point p1, Point p2, Point q) {

//            Line vector
            double px = p2.x-p1.x;
            double py = p2.y-p1.y;

            double norm = px*px + py*py;

            // Length of the projection
            double u =  ((q.x - p1.x) * px + (q.y - p1.y) * py) / norm;

            if (u > 1)
                u = 1;
            else if (u < 0)
                u = 0;

            // Crossing point
            double x = p1.x + u * px;
            double y = p1.y + u * py;

            // Distance to crossing point
            double dx = x - q.x;
            double dy = y - q.y;

            return sqrt(pow(dx,2) + pow(dy,2));
        }
    };




} // end namespace local_g2o


#endif //G2O_EDGE_WALL_H
