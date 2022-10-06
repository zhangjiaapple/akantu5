/**
 * @file   material_lister.cc
 *
 * @author Nicolas Richart <nicolas.richart@epfl.ch>
 *
 * @date creation: Mon Jun 14 2010
 * @date last modification: Fri Mar 16 2018
 *
 * @brief  Small code to generate a list of activated materials
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
#include "material_list.hh"

#include <boost/preprocessor.hpp>
#include <iostream>

int main(__attribute__((unused)) int argc,
         __attribute__((unused)) char * argv[]) {
#define PRINT_OUT_OPTIONS(r, data, i, elem)                                    \
  << ":" << BOOST_PP_STRINGIZE(BOOST_PP_TUPLE_ELEM(2, 0, elem))

#define PRINT_OUT(r, data, elem)                                               \
  std::cout << BOOST_PP_STRINGIZE(BOOST_PP_ARRAY_ELEM(0, elem)) << ":"         \
            << BOOST_PP_STRINGIZE(BOOST_PP_ARRAY_ELEM(1, elem)) BOOST_PP_IF(   \
                   BOOST_PP_EQUAL(3, BOOST_PP_ARRAY_SIZE(elem)),               \
                   BOOST_PP_SEQ_FOR_EACH_I(PRINT_OUT_OPTIONS, _,               \
                                           BOOST_PP_ARRAY_ELEM(2, elem)), )    \
            << std::endl;

  BOOST_PP_SEQ_FOR_EACH(PRINT_OUT, "toto", AKANTU_MATERIAL_LIST);

  return 0;
}
