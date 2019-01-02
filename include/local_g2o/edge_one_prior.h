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
#ifndef GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H
#define GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H

#include "vertex_one.h"
#include "g2o/core/base_unary_edge.h"

namespace g2o {

    /**
     * \brief Prior for a single parameters
     */
    class EdgeOnePrior : public BaseUnaryEdge<1, Vector1, VertexOne> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeOnePrior();

        virtual void setMeasurement(const Vector1 &m) {
            _measurement = m;
        }

        virtual bool setMeasurementData(const number_t *d) {
            _measurement[0] = d[0];
            return true;
        }

        virtual bool getMeasurementData(number_t *d) const {
            d[0] = _measurement[0];
            return true;
        }

        virtual int measurementDimension() const { return 1; }

        virtual void linearizeOplus();

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;

        virtual void computeError() {
            const VertexOne *v = static_cast<const VertexOne *>(_vertices[0]);
//            if ( v->estimate() > 0)
                _error = v->estimate() - _measurement;
//            else
//                _error[0] = 99999999999.0;
        }

        virtual number_t initialEstimatePossible(const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *) { return 0; }
    };

} // end namespace

#endif //GRAPHLOCALIZATION_EDGE_ONE_PRIOR_H
