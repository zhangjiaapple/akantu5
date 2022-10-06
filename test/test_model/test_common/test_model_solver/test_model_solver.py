#!/usr/bin/env python3

""" test_model_solver.py: Test the model solver in python"""

__author__ = "Nicolas Richart"
__credits__ = [
    "Nicolas Richart <nicolas.richart@epfl.ch>",
]
__copyright__ = "Copyright (©) 2016-2021 EPFL (Ecole Polytechnique Fédérale" \
                " de Lausanne) Laboratory (LSMS - Laboratoire de Simulation" \
                " en Mécanique des Solides)"
__license__ = "LGPLv3"

import numpy as np
import numpy.linalg as npl
import python_fe as pfe

sim_u = np.genfromtxt("disp.csv", delimiter=",", names=True)

Ne = int(sim_u['node'][-1])
F = sim_u['force']
blocked = sim_u['blocked']
u = sim_u['disp']

trusses = pfe.TrussFE(Ne=Ne,
                      F={f: [np.where(F == f)] for f in np.unique(F)},
                      blocked=(np.where(blocked == 1),
                               u[blocked == 1]))

solver = pfe.StaticSolver(trusses)
solver.solveStep()

upy = trusses.u

n = npl.norm(upy - u) / npl.norm(upy)

if n > 1e-14:
    raise ValueError('Something went wrong in the simulation')
