/**
 * @file   ntn_friclaw_linear_slip_weakening.hh
 *
 * @author David Simon Kammer <david.kammer@epfl.ch>
 *
 * @date creation: Fri Jun 18 2010
 * @date last modification: Tue Sep 29 2020
 *
 * @brief  linear slip weakening
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
#ifndef AST_NTN_FRICLAW_LINEAR_SLIP_WEAKENING_HH_
#define AST_NTN_FRICLAW_LINEAR_SLIP_WEAKENING_HH_

/* -------------------------------------------------------------------------- */
// simtools
#include "ntn_friclaw_coulomb.hh"

namespace akantu {

/* -------------------------------------------------------------------------- */
template <class Regularisation = NTNFricRegNoRegularisation>
class NTNFricLawLinearSlipWeakening : public NTNFricLawCoulomb<Regularisation> {
  /* ------------------------------------------------------------------------ */
  /* Constructors/Destructors                                                 */
  /* ------------------------------------------------------------------------ */
public:
  NTNFricLawLinearSlipWeakening(NTNBaseContact & contact,
                                const ID & id = "linear_slip_weakening");
  ~NTNFricLawLinearSlipWeakening() override = default;

  /* ------------------------------------------------------------------------ */
  /* Methods                                                                  */
  /* ------------------------------------------------------------------------ */
public:
  /// register synchronizedarrays for sync
  void registerSynchronizedArray(SynchronizedArrayBase & array) override;

  /// dump restart file
  void dumpRestart(const std::string & file_name) const override;

  /// read restart file
  void readRestart(const std::string & file_name) override;

  /// function to print the contain of the class
  void printself(std::ostream & stream, int indent = 0) const override;

protected:
  /// compute frictional strength according to friction law
  void computeFrictionalStrength() override;
  /// computes the friction coefficient as a function of slip
  virtual void computeFrictionCoefficient();

  /* ------------------------------------------------------------------------ */
  /* Dumpable                                                                 */
  /* ------------------------------------------------------------------------ */
public:
  void addDumpFieldToDumper(const std::string & dumper_name,
                            const std::string & field_id) override;

  /* ------------------------------------------------------------------------ */
  /* Accessors                                                                */
  /* ------------------------------------------------------------------------ */

  /* ------------------------------------------------------------------------ */
  /* Class Members                                                            */
  /* ------------------------------------------------------------------------ */
protected:
  // static coefficient of friction
  SynchronizedArray<Real> mu_s;

  // kinetic coefficient of friction
  SynchronizedArray<Real> mu_k;

  // Dc the length over which slip weakening happens
  SynchronizedArray<Real> d_c;
};

/* -------------------------------------------------------------------------- */
/* inline functions                                                           */
/* -------------------------------------------------------------------------- */

/// standard output stream operator
template <class Regularisation>
inline std::ostream &
operator<<(std::ostream & stream,
           const NTNFricLawLinearSlipWeakening<Regularisation> & _this) {
  _this.printself(stream);
  return stream;
}

} // namespace akantu

#include "ntn_friclaw_linear_slip_weakening_tmpl.hh"

#endif /* AST_NTN_FRICLAW_LINEAR_SLIP_WEAKENING_HH_ */
