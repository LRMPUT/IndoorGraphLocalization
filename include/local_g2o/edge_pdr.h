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
#ifndef G2O_EDGE_PDR
#define G2O_EDGE_PDR

#include "g2o/config.h"

#include "g2o/core/base_multi_edge.h"


#include "local_g2o/vertex_one.h"
#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"

namespace g2o {

    class G2O_TYPES_SLAM2D_API EdgePDR : public BaseMultiEdge<-1, VectorX> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // Constructor
        EdgePDR();

        // Setting the measurement ([freq*dt angleChange])
        virtual void setMeasurement(const Vector2 &m);

        // Computes the error of the edge
        virtual void computeError();

        // Read & write from streams
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        // Information is initial estimate is available
        virtual double initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *);

        // Jacobians
        virtual void linearizeOplus();

    };

} // end namespace g2o

#endif    // G2O_EDGE_PDR
