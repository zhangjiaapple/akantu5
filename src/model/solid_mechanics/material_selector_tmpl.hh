/**
 * @file   material_selector_tmpl.hh
 *
 * @author Lucas Frerot <lucas.frerot@epfl.ch>
 * @author Nicolas Richart <nicolas.richart@epfl.ch>
 *
 * @date creation: Wed Nov 13 2013
 * @date last modification: Fri Apr 09 2021
 *
 * @brief  Implementation of the template MaterialSelector
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
#include "material_selector.hh"
/* -------------------------------------------------------------------------- */

#ifndef AKANTU_MATERIAL_SELECTOR_TMPL_HH_
#define AKANTU_MATERIAL_SELECTOR_TMPL_HH_

namespace akantu {

/* -------------------------------------------------------------------------- */
template <>
inline UInt
ElementDataMaterialSelector<std::string>::operator()(const Element & element) {
  try {
    std::string material_name = this->elementData(element);
    return model.getMaterialIndex(material_name);
  } catch (std::exception & e) {
    return MaterialSelector::operator()(element);
  }
}

/* -------------------------------------------------------------------------- */
template <>
inline UInt
ElementDataMaterialSelector<UInt>::operator()(const Element & element) {
  try {
    return this->elementData(element) - first_index;
  } catch (...) {
    return MaterialSelector::operator()(element);
  }
}

/* -------------------------------------------------------------------------- */
template <typename T>
inline UInt
ElementDataMaterialSelector<T>::operator()(const Element & element) {
  return MaterialSelector::operator()(element);
}

/* -------------------------------------------------------------------------- */
template <typename T>
MeshDataMaterialSelector<T>::MeshDataMaterialSelector(
    const std::string & name, const SolidMechanicsModel & model,
    UInt first_index)
    : ElementDataMaterialSelector<T>(model.getMesh().getData<T>(name), model,
                                     first_index) {}

} // namespace akantu

#endif /* AKANTU_MATERIAL_SELECTOR_TMPL_HH_ */
