/**
 * @file   manual_restart.hh
 *
 * @author David Simon Kammer <david.kammer@epfl.ch>
 *
 * @date creation: Fri Mar 16 2018
 * @date last modification: Fri Mar 16 2018
 *
 * @brief  Restart tools header
 *
 *
 * @section LICENSE
 *
 * Copyright (©) 2015-2021 EPFL (Ecole Polytechnique Fédérale de Lausanne)
 * Laboratory (LSMS - Laboratoire de Simulation en Mécanique des Solides)
 *
 * Akantu is free software: you can redistribute it and/or modify it under the
 * terms of the GNU Lesser General Public License as published by the Free
 * Software Foundation, either version 3 of the License, or (at your option) any
 * later version.
 *
 * Akantu is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
 * details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with Akantu. If not, see <http://www.gnu.org/licenses/>.
 *
 */

/**
 * @file   manual_restart.hh
 * @author Dana Christen <dana.christen@epfl.ch>
 * @date   May 15, 2013
 */

/* -------------------------------------------------------------------------- */
#include "aka_array.hh"
#include "solid_mechanics_model.hh"

void dumpArray(const akantu::Array<akantu::Real> & array,
               const std::string & fname);

void loadArray(akantu::Array<akantu::Real> & array, const std::string & fname);
void loadRestart(akantu::SolidMechanicsModel & model, const std::string & fname,
                 akantu::UInt prank);
void loadRestart(akantu::SolidMechanicsModel & model,
                 const std::string & fname);
void dumpRestart(akantu::SolidMechanicsModel & model, const std::string & fname,
                 akantu::UInt prank);
void dumpRestart(akantu::SolidMechanicsModel & model,
                 const std::string & fname);
