
""" __init__.py: akantu python module"""

__author__ = "Guillaume Anciaux and Nicolas Richart"
__credits__ = [
    "Guillaume Anciaux <guillaume.anciaux@epfl.ch>",
    "Nicolas Richart <nicolas.richart@epfl.ch>",
]
__copyright__ = "Copyright (©) 2018-2021 EPFL (Ecole Polytechnique Fédérale" \
                " de Lausanne) Laboratory (LSMS - Laboratoire de Simulation" \
                " en Mécanique des Solides)"
__license__ = "LGPLv3"


import warnings as _aka_warn
import scipy.sparse as _aka_sparse
import numpy as _aka_np
from . import py11_akantu as _py11_akantu

private_keys = set(dir(_py11_akantu)) - set(dir())

for k in private_keys:
    globals()[k] = getattr(_py11_akantu, k)

if _py11_akantu.has_mpi():
    try:
        from mpi4py import MPI  # noqa: F401
    except Exception:
        pass


def initialize(*args, **kwargs):
    raise RuntimeError("No need to call initialize,"
                       " use parseInput to read an input file")


def finalize(*args, **kwargs):
    _aka_warn.warn("No need to call finalize", DeprecationWarning)


class AkantuSparseMatrix (_aka_sparse.coo_matrix):

    def __init__(self, aka_sparse):

        self.aka_sparse = aka_sparse
        matrix_type = self.aka_sparse.getMatrixType()
        sz = self.aka_sparse.size()
        row = self.aka_sparse.getIRN()[:, 0] - 1
        col = self.aka_sparse.getJCN()[:, 0] - 1
        data = self.aka_sparse.getA()[:, 0]

        row = row.copy()
        col = col.copy()
        data = data.copy()

        if matrix_type == _py11_akantu._symmetric:
            non_diags = (row != col)
            row_sup = col[non_diags]
            col_sup = row[non_diags]
            data_sup = data[non_diags]
            col = _aka_np.concatenate((col, col_sup))
            row = _aka_np.concatenate((row, row_sup))
            data = _aka_np.concatenate((data, data_sup))

        _aka_sparse.coo_matrix.__init__(
            self, (data, (row, col)), shape=(sz, sz), dtype=data.dtype)


FromStress = _py11_akantu.FromHigherDim
FromTraction = _py11_akantu.FromSameDim
_py11_akantu.__initialize()

__version__ = _py11_akantu.getVersion()
