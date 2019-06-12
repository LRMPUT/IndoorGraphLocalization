#include "local_g2o/edge_vpr.h"

namespace g2o {

    EdgeVPR::EdgeVPR() : BaseUnaryEdge<2, Vector2, VertexSE2>() {

    }

    void EdgeVPR::computeError() {

        const VertexSE2 *v = static_cast<const VertexSE2 *>(_vertices[0]);

        // Assume error equal to zero
        _error[0] = v->estimate()[0] - _measurement[0];
        _error[1] = v->estimate()[1] - _measurement[1];

        // TODO: angle contraint?
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