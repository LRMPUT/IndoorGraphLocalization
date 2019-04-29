#include "local_g2o/edge_wall.h"

namespace g2o{

    EdgeWall::EdgeWall() : BaseBinaryEdge<1, Vector1, VertexSE2, VertexSE2>() {

    }

    void EdgeWall::computeError(){

        // Assume error equal to zero
        _error[0] = 0.0;

        // For each wall check collision
        for (auto & wall : walls) {

            // Wall from start to end
            Point w1 = {wall.startX, wall.startY};
            Point w2 = {wall.endX, wall.endY};

            // Two consequitive user poses
            VertexSE2 * P1 = static_cast<VertexSE2 *> (_vertices[0]);
            VertexSE2 * P2 = static_cast<VertexSE2 *> (_vertices[1]);
            Point p1 = {P1->estimate()[0], P1->estimate()[1]};
            Point p2 = {P2->estimate()[0], P2->estimate()[1]};

            // User crosses the wall
            if ( doIntersect(w1, w2, p1, p2) ) {

                // Distance of first and second pose
                double d1 = distanceToLine(w1, w2, p1);
                double d2 = distanceToLine(w1, w2, p2);

                // Crossing increases the error
                _error[0] += _measurement[0] * min(d1,d2);
            }
        }
    }

    // EdgeWall PEN inf_matrix
    bool EdgeWall::read(std::istream& is){
        is >> _measurement[0];
        is >> information()(0,0);

        return true;
    }

    bool EdgeWall::write(std::ostream& os) const{
        os << measurement()[0] << " " << information()(0,0);

        return os.good();
    }

    double EdgeWall::initialEstimatePossible(const OptimizableGraph::VertexSet& fixed, OptimizableGraph::Vertex* toEstimate){
        // TODO: try to prevent local minimum

        return -1.0;
    }


} // end namespace local_g2o