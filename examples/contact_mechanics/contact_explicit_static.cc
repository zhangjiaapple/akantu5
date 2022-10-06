/**
 * @file   contact_explicit_static.cc
 *
 * @author Mohit Pundir <mohit.pundir@epfl.ch>
 *
 * @date creation: Thu Feb 21 2013
 * @date last modification: Wed Jun 23 2021
 *
 * @brief  Example of contact mechanics in static
 *
 *
 * @section LICENSE
 *
 * Copyright (©) 2014-2021 EPFL (Ecole Polytechnique Fédérale de Lausanne)
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
#include "contact_mechanics_model.hh"
#include "coupler_solid_contact.hh"
#include "non_linear_solver.hh"
#include "solid_mechanics_model.hh"
#include "surface_selector.hh"
/* -------------------------------------------------------------------------- */

using namespace akantu;

/* -------------------------------------------------------------------------- */
int main(int argc, char * argv[]) {

  const UInt spatial_dimension = 2;
  initialize("material.dat", argc, argv);

  Mesh mesh(spatial_dimension);
  mesh.read("hertz.msh");

  CouplerSolidContact coupler(mesh);

  auto & solid = coupler.getSolidMechanicsModel();
  auto & contact = coupler.getContactMechanicsModel();

  auto && selector = std::make_shared<MeshDataMaterialSelector<std::string>>(
      "physical_names", solid);
  solid.setMaterialSelector(selector);

  coupler.initFull(_analysis_method = _static);

  auto && surface_selector = std::make_shared<PhysicalSurfaceSelector>(mesh);
  contact.getContactDetector().setSurfaceSelector(surface_selector);

  coupler.applyBC(BC::Dirichlet::FixedValue(0.0, _x), "fixed");
  coupler.applyBC(BC::Dirichlet::FixedValue(0.0, _y), "fixed");
  coupler.applyBC(BC::Dirichlet::FixedValue(0.0, _x), "loading");
  coupler.applyBC(BC::Dirichlet::FixedValue(0.0, _x), "symmetry");

  coupler.setBaseName("contact-explicit-static");
  coupler.addDumpFieldVector("displacement");
  coupler.addDumpFieldVector("normals");
  coupler.addDumpFieldVector("contact_force");
  coupler.addDumpFieldVector("external_force");
  coupler.addDumpFieldVector("internal_force");
  coupler.addDumpField("gaps");
  coupler.addDumpField("areas");
  coupler.addDumpField("blocked_dofs");
  coupler.addDumpField("grad_u");
  coupler.addDumpField("stress");

  auto max_steps = 100u;

  for (auto _ [[gnu::unused]] : arange(max_steps)) {

    auto increment = 1e-4;
    coupler.applyBC(BC::Dirichlet::IncrementValue(-increment, _y), "loading");

    coupler.solveStep();
    coupler.dump();
  }

  finalize();
  return EXIT_SUCCESS;
}
