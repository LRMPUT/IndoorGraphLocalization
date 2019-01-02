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
#ifndef G2O_VERTEX_ONE_H
#define G2O_VERTEX_ONE_H

#include "g2o/config.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/hyper_graph_action.h"

#include <Eigen/Core>

namespace g2o {


    typedef Eigen::Matrix<number_t, 1, 1, Eigen::ColMajor> Vector1;

    class VertexOne : public BaseVertex<1, Vector1> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        VertexOne();

        virtual void setToOriginImpl() {
            _estimate.setZero();
        }

        virtual bool setEstimateDataImpl(const number_t *est) {
            _estimate[0] = est[0];
            return true;
        }

        virtual bool getEstimateData(number_t *est) const {
            est[0] = _estimate[0];
            return true;
        }

        virtual int estimateDimension() const {
            return 1;
        }

        virtual bool setMinimalEstimateDataImpl(const number_t *est) {
            return setEstimateData(est);
        }

        virtual bool getMinimalEstimateData(number_t *est) const {
            return getEstimateData(est);
        }

        virtual int minimalEstimateDimension() const {
            return 1;
        }

        virtual void oplusImpl(const number_t *update) {
            _estimate[0] += update[0];
        }

        virtual bool read(std::istream &is);

        virtual bool write(std::ostream &os) const;

    };


}

#endif