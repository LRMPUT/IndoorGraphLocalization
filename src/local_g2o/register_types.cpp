//
// Created by mnowicki on 01.10.18.
//

#include "local_g2o/register_types.h"

#include "g2o/core/factory.h"

#include "g2o/stuff/macros.h"

#include <iostream>

namespace g2o {

    G2O_REGISTER_TYPE(VERTEX_ONE, VertexOne);
    G2O_REGISTER_TYPE(EDGE_PDR, EdgePDR);
    G2O_REGISTER_TYPE(EDGE_STEP, EdgeStepometer);
    G2O_REGISTER_TYPE(EDGE_WKNN, EdgeWKNN);
    G2O_REGISTER_TYPE(EDGE_PRIOR_ONE, EdgeOnePrior);

} // end namespace