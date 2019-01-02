#include "local_g2o/edge_wknn.h"


namespace g2o {

    EdgeWKNN::EdgeWKNN(int size) : BaseMultiEdge<-1, VectorX>() {
        setSize(size);
    }

    void EdgeWKNN::setMeasurement(const VectorX &m) {

        // Normalizing the weights
        double sumOfMeasurements = 0;
        for (unsigned int i = 0; i < _kDimension; i++) {
            sumOfMeasurements += m(i);
        }
        _measurement = m / sumOfMeasurements;
    }

    void EdgeWKNN::computeError() {

        // Current pose estimate
        VertexSE2 *pose = static_cast<VertexSE2 *> (_vertices[0]);

        // WiFi estimate with WiFi fingerprinting
        double wifiEstimate[2] = {0, 0};
        for (unsigned int i = 0; i < _kDimension; i++) {
            VertexSE2 *xy = static_cast<VertexSE2 *> (_vertices[1 + i]);

            wifiEstimate[0] += _measurement[i] * xy->estimate().translation().x();
            wifiEstimate[1] += _measurement[i] * xy->estimate().translation().y();

        }

        // Setting the error
        _error[0] = pose->estimate().translation().x() - wifiEstimate[0];
        _error[1] = pose->estimate().translation().y() - wifiEstimate[1];

        // TODO: Testing approximated deadzone
//        double errSq = _error[0]*_error[0] + _error[1]*_error[1];
//        if (errSq < 5.0 * 5.0) {
//            _error[0] = 0;
//            _error[1] = 0;
//        }
    }

    // EdgeWKNN kDiemnsion weight weight ... inf_matrix
    bool EdgeWKNN::read(std::istream &is) {
        is >> _kDimension;
        setSize(_kDimension + 1);

        // reading the measurements and normalizing them
        double sumOfMeasurements = 0;
        for (unsigned int i = 0; i < _kDimension; i++) {
            is >> _measurement[i];
            sumOfMeasurements += _measurement[i];
        }
        _measurement = _measurement / sumOfMeasurements;

        // read the information matrix
        for (unsigned int i = 0; i < 2; i++) {
            // fill the "upper triangle" part of the matrix
            for (unsigned int j = i; j < 2; j++) {
                is >> information()(i, j);
            }

            // fill the lower triangle part
            for (unsigned int j = 0; j < i; j++) {
                information()(i, j) = information()(j, i);
            }

        }

        return true;
    }

    bool EdgeWKNN::write(std::ostream &os) const {
        // write number of observed points
        os << "|| " << _kDimension;

        // write measurements
        for (unsigned int i = 0; i < _kDimension; i++) {
            os << " " << _measurement[i];
        }

        // write information matrix
        for (unsigned int i = 0; i < 2; i++) {
            for (unsigned int j = i; j < 2; j++) {
                os << " " << information()(i, j);
            }
        }

        return os.good();
    }

    void EdgeWKNN::linearizeOplus() {

        // Jacobian w.r.t. pose of the current scan (2x3)
        MatrixX Ji;
        Ji.resize(2, 3);
        Ji.fill(0);
        Ji(0, 0) = 1.0;
        Ji(1, 1) = 1.0;

        _jacobianOplus[0] = Ji;

        // Jacobians w.r.t. the poses of the anchors used in the WiFi fingerprinting
        for (unsigned int i = 0; i < _kDimension; i++) {

            MatrixX Jj;
            Jj.resize(2, 3);
            Jj.fill(0);

            g2o::OptimizableGraph::Vertex *v = dynamic_cast<g2o::OptimizableGraph::Vertex *>(_vertices[i]);

            if (!v->fixed())
                Jj.block<2, 2>(0, 0) = -_measurement[i] * Ji;
            _jacobianOplus[i + 1] = Jj;
        }
    }

    double EdgeWKNN::initialEstimatePossible(const OptimizableGraph::VertexSet &fixed,
                                             OptimizableGraph::Vertex *toEstimate) {
        return false;
    }

    void EdgeWKNN::setSize(int vertices) {
        resize(vertices);
        _kDimension = vertices - 1;
        _information.resize(2, 2);
        _error.resize(2, 1);
        _measurement.resize(vertices - 1, 1);
        _dimension = 2;
    }


} // end namespace g2o
