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
#ifndef G2O_EDGE_WKNN
#define G2O_EDGE_WKNN

#include "g2o/config.h"

#include "g2o/core/base_multi_edge.h"

#include "g2o/types/slam2d/vertex_se2.h"
#include "g2o/types/slam2d/vertex_point_xy.h"
#include "g2o/types/slam2d/g2o_types_slam2d_api.h"

namespace g2o {

    class G2O_TYPES_SLAM2D_API EdgeWKNN : public BaseMultiEdge<-1, VectorX> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // We need to provide the number of vertices (it is the WKNN k number + 1)
        EdgeWKNN(int size=1);

        // Setting the measurement. It also normalizes the weights
        virtual void setMeasurement(const VectorX &m);

        // Computes the error of the edge
        virtual void computeError();

        // Read & write from streams
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;

        // Information if initial estimate is available
        virtual double initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *);

        // Jacobians
        virtual void linearizeOplus();

    protected:
        // Set the sizes of all involved elements
        void setSize(int vertices);

        // WKNN k parameter
        unsigned int _kDimension;
    };

} // end namespace g2o

#endif    // G2O_EDGE_WKNN
