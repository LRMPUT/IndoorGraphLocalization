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
#ifndef GRAPHLOCALIZATION_REGISTER_TYPES_H
#define GRAPHLOCALIZATION_REGISTER_TYPES_H

#include "g2o/config.h"

// Custom edges
#include "local_g2o/edge_pdr.h"
#include "local_g2o/edge_stepometer.h"
#include "local_g2o/edge_wknn.h"
#include "local_g2o/vertex_one.h"
#include "local_g2o/edge_one_prior.h"
#include "local_g2o/edge_wknn_deadzone.h"
#include "local_g2o/edge_wall.h"


#endif //GRAPHLOCALIZATION_REGISTER_TYPES_H
