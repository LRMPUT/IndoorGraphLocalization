#include "local_g2o/edge_wknn_deadzone.h"


namespace g2o {

    EdgeWKNNDeadzone::EdgeWKNNDeadzone(int size) : BaseMultiEdge<-1, VectorX>() {
        setSize(size);
    }

    void EdgeWKNNDeadzone::setMeasurement(const VectorX &m) {

        // Normalizing the weights
        double sumOfMeasurements = 0;
        for (unsigned int i = 0; i < _kDimension; i++) {
            sumOfMeasurements += m(i);
        }
        _measurement = m / sumOfMeasurements;
    }

    void EdgeWKNNDeadzone::computeError() {

        // Current pose estimate
        VertexSE2 *pose = static_cast<VertexSE2 *> (_vertices[0]);

        // WiFi estimate with WiFi fingerprinting
        double wifiEstimate[2] = {0, 0};
        for (unsigned int i = 0; i < _kDimension; i++) {
            VertexSE2 *xy = static_cast<VertexSE2 *> (_vertices[1 + i]);

            wifiEstimate[0] += _measurement[i] * xy->estimate().translation().x();
            wifiEstimate[1] += _measurement[i] * xy->estimate().translation().y();

        }

        // Computing the error
        double errorX = pose->estimate().translation().x() - wifiEstimate[0];
        double errorY = pose->estimate().translation().y() - wifiEstimate[1];
        double errorDist = sqrt(errorX*errorX + errorY*errorY);

        // Deadzone
        if (errorDist < 3.0)
            _error[0] = 0;
        else
            _error[0] = errorDist - 3.0;
    }

    // EdgeWKNNDeadzone kDiemnsion weight weight ... inf_matrix
    bool EdgeWKNNDeadzone::read(std::istream &is) {
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
        is >> information()(0, 0);

        return true;
    }

    bool EdgeWKNNDeadzone::write(std::ostream &os) const {
        // write number of observed points
        os << "|| " << _kDimension;

        // write measurements
        for (unsigned int i = 0; i < _kDimension; i++) {
            os << " " << _measurement[i];
        }

        // write information matrix
        os << " " << information()(0,0);

        return os.good();
    }


    double EdgeWKNNDeadzone::initialEstimatePossible(const OptimizableGraph::VertexSet &fixed,
                                             OptimizableGraph::Vertex *toEstimate) {
        return false;
    }

    void EdgeWKNNDeadzone::setSize(int vertices) {
        resize(vertices);
        _kDimension = vertices - 1;
        _information.resize(1, 1);
        _error.resize(1, 1);
        _measurement.resize(vertices - 1, 1);
        _dimension = 1;
    }


} // end namespace g2o
