/**
 * @file   predefined_bc.cc
 *
 * @author Zineb Fouad <zineb.fouad@epfl.ch>
 * @author Nicolas Richart <nicolas.richart@epfl.ch>
 *
 * @date creation: Wed Aug 04 2010
 * @date last modification: Wed Feb 06 2019
 *
 * @brief  Example showing how to set boundary conditions
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

/* -------------------------------------------------------------------------- */
#include "solid_mechanics_model.hh"
/* -------------------------------------------------------------------------- */
#include <iostream>
/* -------------------------------------------------------------------------- */

using namespace akantu;

int main(int argc, char * argv[]) {
  initialize("material.dat", argc, argv);

  Mesh mesh(2);
  mesh.read("square.msh");

  // model initialization
  SolidMechanicsModel model(mesh);
  model.initFull();

  // Dirichlet boundary conditions
  model.applyBC(BC::Dirichlet::FixedValue(0.0, _x), "Fixed_x");
  model.applyBC(BC::Dirichlet::FixedValue(0.0, _y), "Fixed_y");

  // output in a paraview file
  model.setBaseName("plate");
  model.addDumpFieldVector("displacement");
  model.addDumpField("blocked_dofs");
  model.addDumpField("external_force");
  model.dump();

  finalize();
  return EXIT_SUCCESS;
}
