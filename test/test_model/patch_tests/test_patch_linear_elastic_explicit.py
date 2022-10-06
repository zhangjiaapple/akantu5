#!/usr/bin/env python3

"""test_patch_linear_elastic_explicit.py: linear elatic patch test in python"""

__author__ = "Guillaume Anciaux"
__credits__ = [
    "Guillaume Anciaux <guillaume.anciaux@epfl.ch>",
]
__copyright__ = "Copyright (©) 2016-2021 EPFL (Ecole Polytechnique Fédérale" \
                " de Lausanne) Laboratory (LSMS - Laboratoire de Simulation" \
                " en Mécanique des Solides)"
__license__ = "LGPLv3"

import sys
from patch_test_linear_solid_mechanics_fixture import TestPatchTestSMMLinear
import akantu


def foo(self):

    filename = "material_check_stress_plane_stress.dat"
    if self.plane_strain:
        filename = "material_check_stress_plane_strain.dat"

    self.initModel(akantu._explicit_lumped_mass, filename)

    coordinates = self.mesh.getNodes()
    displacement = self.model.getDisplacement()
    # set the position of all nodes to the static solution
    self.setLinearDOF(displacement, coordinates)

    for s in range(0, 100):
        self.model.solveStep()

    ekin = self.model.getEnergy("kinetic")
    self.assertAlmostEqual(0, ekin, delta=1e-16)
    self.checkAll()


def test():
    TestPatchTestSMMLinear.TYPED_TEST(foo, "Explicit")


if 'pytest' not in sys.modules:
    test()
