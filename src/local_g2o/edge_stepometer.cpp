//
// Created by mnowicki on 09.10.18.
//

#include "local_g2o/edge_stepometer.h"


namespace g2o {

    EdgeStepometer::EdgeStepometer() : BaseMultiEdge<-1, VectorX>() {
        resize(3);
        _information.resize(2, 2);
        _error.resize(2, 1);
        _measurement.resize(1, 1);
        _dimension = 3;
    }

    void EdgeStepometer::setMeasurement(const Vector1 &m) {
        _measurement = m;
    }

    void EdgeStepometer::computeError() {

        const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
        const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
        const VertexOne *v3 = static_cast<const VertexOne *>(_vertices[2]);

        double estimatedStepLength = v3->estimate()[0];

        Eigen::Vector3d delta = (v1->estimate().inverse() * v2->estimate()).toVector();

        double averageAngle = (v1->estimate()[2] + v2->estimate()[2]) / 2.0;

        _error[0] = delta[0] - estimatedStepLength * _measurement[0] * cos(averageAngle);
        _error[1] = delta[1] - estimatedStepLength * _measurement[0] * sin(averageAngle);
    }


    bool EdgeStepometer::read(std::istream &is) {
        // Not needed for now
        return true;
    }

    bool EdgeStepometer::write(std::ostream &os) const {
        os << _measurement[0];

        // write information matrix
        for (unsigned int i = 0; i < 2; i++) {
            for (unsigned int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        }

        return os.good();
    }

    void EdgeStepometer::linearizeOplus() {

        const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
        const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
        const VertexOne *v3 = static_cast<const VertexOne *>(_vertices[2]);

        double estimatedStepLength = v3->estimate()[0];
        double averageAngle = (v1->estimate()[2] + v2->estimate()[2]) / 2.0;

        // Jacobian w.r.t. first pose (2x3)
        MatrixX Ji, Jj, Jk;
        Ji.resize(2, 3);
        Ji.fill(0);
        Jj = Ji;
        if (!v1->fixed())
        {
            Ji(0, 0) = -1.0;
            Ji(0, 2) = estimatedStepLength * _measurement[0] * sin(averageAngle) / 2;
            Ji(1, 1) = -1.0;
            Ji(1, 2) = -estimatedStepLength * _measurement[0] * cos(averageAngle) / 2;
        }
        _jacobianOplus[0] = Ji;

        // Jacobian w.r.t. second pose (2x3)
        if (!v2->fixed())
        {
            Jj(0, 0) = 1.0;
            Jj(0, 2) = estimatedStepLength * _measurement[0] * sin(averageAngle) / 2;
            Jj(1, 1) = 1.0;
            Jj(1, 2) = -estimatedStepLength * _measurement[0] * cos(averageAngle) / 2;
        }
        _jacobianOplus[1] = Jj;

        // Jacobian w.r.t. step length (2x1)
        Jk.resize(2, 1);
        if (!v3->fixed()) {
            Jk(0) = -_measurement[0] * cos(averageAngle);
            Jk(1) = -_measurement[0] * sin(averageAngle);;
        }
        _jacobianOplus[2] = Jk;
    }

    double EdgeStepometer::initialEstimatePossible(const OptimizableGraph::VertexSet &fixed,
                                            OptimizableGraph::Vertex *toEstimate) {
        return false;
    }

} // end namespace g2o
