#===============================================================================
# @file   embedded.cmake
#
# @author Lucas Frerot <lucas.frerot@epfl.ch>
# @author Nicolas Richart <nicolas.richart@epfl.ch>
#
# @date creation: Tue Oct 16 2012
# @date last modification: Fri Mar 16 2018
#
# @brief  package descrition for embedded model use
#
#
# @section LICENSE
#
# Copyright (©) 2010-2021 EPFL (Ecole Polytechnique Fédérale de Lausanne)
# Laboratory (LSMS - Laboratoire de Simulation en Mécanique des Solides)
#
# Akantu is free software: you can redistribute it and/or modify it under the
# terms of the GNU Lesser General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option) any
# later version.
# 
# Akantu is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
# A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
# details.
# 
# You should have received a copy of the GNU Lesser General Public License along
# with Akantu. If not, see <http://www.gnu.org/licenses/>.
#
#===============================================================================


package_declare(embedded 
  DESCRIPTION "Add support for the embedded solid mechanics model"
  DEPENDS CGAL)

package_declare_sources(embedded
  model/solid_mechanics/solid_mechanics_model_embedded_interface/embedded_interface_intersector.cc
  model/solid_mechanics/solid_mechanics_model_embedded_interface/embedded_interface_intersector.hh
  model/solid_mechanics/solid_mechanics_model_embedded_interface/embedded_interface_model.cc
  model/solid_mechanics/solid_mechanics_model_embedded_interface/embedded_interface_model.hh

  model/solid_mechanics/materials/material_embedded/material_embedded_includes.hh
  model/solid_mechanics/materials/material_embedded/material_reinforcement.hh
  model/solid_mechanics/materials/material_embedded/material_reinforcement_tmpl.hh
  )

package_declare_material_infos(embedded
  LIST AKANTU_EMBEDDED_MATERIAL_LIST
  INCLUDE material_embedded_includes.hh
  )

