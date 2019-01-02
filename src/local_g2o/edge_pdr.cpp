#include "local_g2o/edge_pdr.h"


namespace g2o {

	EdgePDR::EdgePDR() : BaseMultiEdge<-1, VectorX>() {
		resize(3);
		_information.resize(3, 3);
		_error.resize(3, 1);
		_measurement.resize(2, 1);
		_dimension = 3;
	}

	void EdgePDR::setMeasurement(const Vector2 &m) {
		_measurement = m;
	}

	void EdgePDR::computeError() {

		const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
		const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
		const VertexOne *v3 = static_cast<const VertexOne *>(_vertices[2]);

		double estimatedStepLength = v3->estimate()[0];

		Eigen::Vector3d delta = (v1->estimate().inverse() * v2->estimate()).toVector();

//		Eigen::Vector3d delta;
//		delta[0] = v2->estimate()[0] - v1->estimate()[0];
//		delta[1] = v2->estimate()[1] - v1->estimate()[1];
//		delta[2] = v2->estimate()[2] - v1->estimate()[2];

//		Eigen::Vector3d v1mv2;
//		v1mv2[0] = v2->estimate()[0] - v1->estimate()[0];
//		v1mv2[1] = v2->estimate()[1] - v1->estimate()[1];
//		v1mv2[2] = v2->estimate()[2] - v1->estimate()[2];

//		std::cout << "PDR: v1.inv() * v2:  " << delta[0] << " " << delta[1] << " " << normalize_theta(delta[2])*180.0/3.1415265 << " |" << delta[0]*delta[0] + delta[1]*delta[1] << std::endl;
//		std::cout << "PDR: -v1 + v2:  " << v1mv2[0] << " " << v1mv2[1] << " " << normalize_theta(v1mv2[2])*180.0/3.1415265 << " |" << v1mv2[0]*v1mv2[0] + v1mv2[1]*v1mv2[1] << std::endl;
//		std::cout << " ------ " << std::endl;

		double averageAngle = (v1->estimate()[2] + v2->estimate()[2]) / 2.0;
//		double averageAngle = normalize_theta(-v1->estimate()[2] + v2->estimate()[2]) / 2.0;
//        double averageAngle = normalize_theta(v1->estimate()[2] + v2->estimate()[2]) / 2.0;
//        double averageAngle = normalize_theta(delta[2]) / 2.0;
//		double averageAngle = v1->estimate()[2];


//		std::cout << "PDR: " << v1->estimate()[2] *180.0/3.1415265 << " " << v2->estimate()[2] *180.0/3.1415265
//		    << " avg: " << averageAngle*180.0/3.1415265 <<  std::endl;

//		_error[0] = 0;//delta[0] - estimatedStepLength * _measurement[0] * cos(v2->estimate()[2]);
//		_error[1] = 0;//delta[1] - estimatedStepLength * _measurement[0] * sin(v2->estimate()[2]);
//		_error[2] = normalize_theta(delta[2] - _measurement[1]);

		_error[0] = delta[0] - estimatedStepLength * _measurement[0] * cos(averageAngle);
		_error[1] = delta[1] - estimatedStepLength * _measurement[0] * sin(averageAngle);
        _error[2] = normalize_theta(delta[2] - _measurement[1]);
	}


	bool EdgePDR::read(std::istream &is) {
		// Not needed for now
		return true;
	}

	bool EdgePDR::write(std::ostream &os) const {
        os << _measurement[0] << " " << _measurement[1];

        // write information matrix
        for (unsigned int i = 0; i < 3; i++) {
            for (unsigned int j = i; j < 3; j++) {
                os << " " << information()(i, j);
            }
        }

        return os.good();
	}

	void EdgePDR::linearizeOplus() {

		const VertexSE2 *v1 = static_cast<const VertexSE2 *>(_vertices[0]);
		const VertexSE2 *v2 = static_cast<const VertexSE2 *>(_vertices[1]);
		const VertexOne *v3 = static_cast<const VertexOne *>(_vertices[2]);

		double estimatedStepLength = v3->estimate()[0];
		double averageAngle = (v1->estimate()[2] + v2->estimate()[2]) / 2.0;

		Eigen::Vector3d delta;
		delta[0] = v2->estimate()[0] - v1->estimate()[0];
		delta[1] = v2->estimate()[1] - v1->estimate()[1];
		delta[2] = v2->estimate()[2] - v1->estimate()[2];

        double cosv1 = cos(v1->estimate()[2]);
        double sinv1 = sin(v1->estimate()[2]);

        // Jacobian w.r.t. first pose (3x3)
        MatrixX Ji, Jj, Jk;
        Ji.resize(3, 3);
        Ji.fill(0);
        Jj = Ji;
        if (!v1->fixed())
        {
            Ji(0, 0) = -cosv1;
            Ji(0, 1) = -sinv1;
            Ji(0, 2) = -delta[0] * sinv1 + delta[1] * cosv1 + estimatedStepLength * _measurement[0] * sin(averageAngle) / 2;
            Ji(1, 0) = sinv1;
            Ji(1, 1) = -cosv1;
            Ji(1, 2) = -delta[0] * cosv1 - delta[1] * sinv1 - estimatedStepLength * _measurement[0] * cos(averageAngle) / 2;
            Ji(2, 2) = -1.0;
        }
        _jacobianOplus[0] = Ji;

        // Jacobian w.r.t. second pose (3x3)
        if (!v2->fixed())
        {
            Jj(0, 0) = cosv1;
            Jj(0, 1) = sinv1;
            Jj(0, 2) = estimatedStepLength * _measurement[0] * sin(averageAngle) / 2;
            Jj(1, 0) = -sinv1;
            Jj(1, 1) = cosv1;
            Jj(1, 2) = -estimatedStepLength * _measurement[0] * cos(averageAngle) / 2;
            Jj(2, 2) = 1.0;
        }
        _jacobianOplus[1] = Jj;

        // Jacobian w.r.t. step length (3x1)
        Jk.resize(3, 1);
        Jk.fill(0);
        if (!v3->fixed()) {
            Jk(0) = -_measurement[0] * cos(averageAngle);
            Jk(1) = -_measurement[0] * sin(averageAngle);
        }
        _jacobianOplus[2] = Jk;
	}

	double EdgePDR::initialEstimatePossible(const OptimizableGraph::VertexSet &fixed,
											 OptimizableGraph::Vertex *toEstimate) {
		return false;
	}

} // end namespace g2o
