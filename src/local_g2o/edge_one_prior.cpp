//
// Created by mnowicki on 26.09.18.
//

#include "local_g2o/edge_one_prior.h"

namespace g2o {

    EdgeOnePrior::EdgeOnePrior() : BaseUnaryEdge<1, Vector1, VertexOne>()
    {
        _information.setIdentity();
        _error.setZero();
    }


    bool EdgeOnePrior::read(std::istream& is)
    {
        Vector1 p;
        is >> p[0];
        setMeasurement(p);

        is >> information()(0, 0);
        return true;
    }

    bool EdgeOnePrior::write(std::ostream& os) const
    {
        Vector1 p = measurement();
        os << p.x() << " " << information()(0, 0);
        return os.good();
    }

    void EdgeOnePrior::linearizeOplus()
    {
        _jacobianOplusXi=Vector1::Identity();
    }
} // end namespace