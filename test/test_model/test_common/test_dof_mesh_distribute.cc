/**
 * @file   test_dof_mesh_distribute.cc
 *
 * @author Philip Müller <phimuell@ethz.ch>
 *
 * @brief  Test if the DOFManager can handle the distribution event of teh mesh.
 *
 * This test is based on the file `test_dof_synchronizer.cc`.
 *
 * @section LICENSE
 *
 * Copyright (©) 2010-2021 EPFL (Ecole Polytechnique Fédérale de Lausanne)
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
#include "communicator.hh"
#include "dof_synchronizer.hh"
#include "element_synchronizer.hh"
#include "mesh_io.hh"
#include "mesh_partition_scotch.hh"

#include <gtest/gtest.h>
#include <string>

/* -------------------------------------------------------------------------- */
#include "io_helper.hh"

/* -------------------------------------------------------------------------- */
using namespace akantu;

TEST(TestMesh, TestMeshDistribute) {

  const UInt spatial_dimension = 3;

  int argc = 0;
  char ** argv = nullptr;
  initialize(argc, argv);

  const auto & comm = akantu::Communicator::getStaticCommunicator();
  Int prank = comm.whoAmI();

  Mesh mesh1(spatial_dimension);
  Mesh mesh2(spatial_dimension);

  if (prank == 0) {
    mesh1.read("mesh.msh");
    mesh2.read("mesh.msh");
  }

  /* This should fail since the mesh is distributed after the
   *  DOFManager is created. */
  DOFManagerDefault dof_manager(mesh1, "test_dof_manager_1__failing");
  EXPECT_THROW(mesh1.distribute(), debug::Exception);

  /* This will succeed since the mesh is distributed before the manager is
   * created. */
  mesh2.distribute();
  EXPECT_NO_THROW(
      DOFManagerDefault dof_manager2(mesh2, "test_dof_manager_2__succeeding"));

  finalize();
}
