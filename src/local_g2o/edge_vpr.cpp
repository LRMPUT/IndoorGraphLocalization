#include "local_g2o/edge_vpr.h"

namespace g2o {

    EdgeVPR::EdgeVPR() : BaseUnaryEdge<2, Vector2, VertexSE2>() {

    }

    void EdgeVPR::computeError() {

        const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);

        // Assume error equal to zero
        _error[0] = 0.0;
        _error[1] = 0.0;

//        // For each wall check collision
//        for (auto & wall : walls) {
//
//            // Wall from start to end
//            Point w1 = {wall.startX, wall.startY};
//            Point w2 = {wall.endX, wall.endY};
//
//            // Two consequitive user poses
//            VertexSE2 * P1 = static_cast<VertexSE2 *> (_vertices[0]);
//            VertexSE2 * P2 = static_cast<VertexSE2 *> (_vertices[1]);
//            Point p1 = {P1->estimate()[0], P1->estimate()[1]};
//            Point p2 = {P2->estimate()[0], P2->estimate()[1]};
//
//            // User crosses the wall
//            if ( doIntersect(w1, w2, p1, p2) ) {
//
//                // Distance of first and second pose
//                double d1 = distanceToLine(w1, w2, p1);
//                double d2 = distanceToLine(w1, w2, p2);
//
//                // Crossing increases the error
//                _error[0] += _measurement[0] * min(d1,d2);
//            }
//        }
    }

    // EdgeVPR ???? inf_matrix
    bool EdgeVPR::read(std::istream &is) {
        // Not needed for now

        return true;
    }

    bool EdgeVPR::write(std::ostream &os) const {
        os << _measurement[0] << " " << _measurement[1];

        // write information matrix
        for (unsigned int i = 0; i < 2; i++) {
            for (unsigned int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        }

        return os.good();
    }

}