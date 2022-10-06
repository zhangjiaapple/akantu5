/**
 * @file   coupler_solid_contact.hh
 *
 * @author Mohit Pundir <mohit.pundir@epfl.ch>
 * @author Nicolas Richart <nicolas.richart@epfl.ch>
 *
 * @date creation: Fri Jun 18 2010
 * @date last modification: Sat Jun 26 2021
 *
 * @brief  class for coupling of solid mechanics and conatct mechanics
 * model in explicit
 *
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
#include "contact_mechanics_model.hh"
#include "solid_mechanics_model.hh"
#if defined(AKANTU_COHESIVE_ELEMENT)
#include "solid_mechanics_model_cohesive.hh"
#endif
/* -------------------------------------------------------------------------- */

#ifndef __AKANTU_COUPLER_SOLID_CONTACT_HH__
#define __AKANTU_COUPLER_SOLID_CONTACT_HH__

/* ------------------------------------------------------------------------ */
/* Coupling : Solid Mechanics / Contact Mechanics                           */
/* ------------------------------------------------------------------------ */
namespace akantu {

/* -------------------------------------------------------------------------- */
template <class SolidMechanicsModelType>
class CouplerSolidContactTemplate : public Model,
                                    public DataAccessor<Element>,
                                    public DataAccessor<UInt> {
  static_assert(
      std::is_base_of<SolidMechanicsModel, SolidMechanicsModelType>::value,
      "SolidMechanicsModelType should be derived from SolidMechanicsModel");
  /* ------------------------------------------------------------------------ */
  /* Constructor/Destructor                                                   */
  /* ------------------------------------------------------------------------ */
public:
  CouplerSolidContactTemplate(
      Mesh & mesh, UInt dim = _all_dimensions,
      const ID & id = "coupler_solid_contact",
      std::shared_ptr<DOFManager> dof_manager = nullptr);

  ~CouplerSolidContactTemplate() override;

  /* ------------------------------------------------------------------------ */
  /* Methods                                                                  */
  /* ------------------------------------------------------------------------ */
protected:
  /// initialize completely the model
  void initFullImpl(const ModelOptions & options) override;

  /// get some default values for derived classes
  std::tuple<ID, TimeStepSolverType>
  getDefaultSolverID(const AnalysisMethod & method) override;

  /* ------------------------------------------------------------------------ */
  /* Solver Interface                                                         */
  /* ------------------------------------------------------------------------ */
public:
  /// assembles the contact stiffness matrix
  virtual void assembleStiffnessMatrix();

  /// assembles the contant internal forces
  virtual void assembleInternalForces();

#if defined(AKANTU_COHESIVE_ELEMENT)
  template <class Model_ = SolidMechanicsModelType,
            std::enable_if_t<std::is_same<
                Model_, SolidMechanicsModelCohesive>::value> * = nullptr>
  UInt checkCohesiveStress() {
    return solid->checkCohesiveStress();
  }
#endif

  template <typename FunctorType>
  inline void applyBC(const FunctorType & func) {
    solid->applyBC(func);
  }

  template <class FunctorType>
  inline void applyBC(const FunctorType & func,
                      const std::string & group_name) {
    solid->applyBC(func, group_name);
  }

  template <class FunctorType>
  inline void applyBC(const FunctorType & func,
                      const ElementGroup & element_group) {
    solid->applyBC(func, element_group);
  }

protected:
  /// callback for the solver, this adds f_{ext} - f_{int} to the residual
  void assembleResidual() override;

  /// callback for the solver, this adds f_{ext} or  f_{int} to the residual
  void assembleResidual(const ID & residual_part) override;
  bool canSplitResidual() const override { return true; }

  /// get the type of matrix needed
  MatrixType getMatrixType(const ID & matrix_id) const override;

  /// callback for the solver, this assembles different matrices
  void assembleMatrix(const ID & matrix_id) override;

  /// callback for the solver, this assembles the stiffness matrix
  void assembleLumpedMatrix(const ID & matrix_id) override;

  /// callback for the solver, this is called at beginning of solve
  void predictor() override;

  /// callback for the solver, this is called at end of solve
  void corrector() override;

  /// callback for the solver, this is called at beginning of solve
  void beforeSolveStep() override;

  /// callback for the solver, this is called at end of solve
  void afterSolveStep(bool converged = true) override;

  /// callback for the model to instantiate the matricess when needed
  void initSolver(TimeStepSolverType time_step_solver_type,
                  NonLinearSolverType non_linear_solver_type) override;

  /* ------------------------------------------------------------------------ */
  /* Mass matrix for solid mechanics model                                    */
  /* ------------------------------------------------------------------------ */
public:
  /// assemble the lumped mass matrix
  void assembleMassLumped();

  /// assemble the mass matrix for consistent mass resolutions
  void assembleMass();

protected:
  /// assemble the lumped mass matrix for local and ghost elements
  void assembleMassLumped(GhostType ghost_type);

  /// assemble the mass matrix for either _ghost or _not_ghost elements
  void assembleMass(GhostType ghost_type);

protected:
  /* ------------------------------------------------------------------------ */
  TimeStepSolverType getDefaultSolverType() const override;
  /* ------------------------------------------------------------------------ */
  ModelSolverOptions
  getDefaultSolverOptions(const TimeStepSolverType & type) const override;

public:
  bool isDefaultSolverExplicit() { return method == _explicit_lumped_mass; }

  /* ------------------------------------------------------------------------ */
public:
  // DataAccessor<Element>
  UInt getNbData(const Array<Element> & /*elements*/,
                 const SynchronizationTag & /*tag*/) const override {
    return 0;
  }
  void packData(CommunicationBuffer & /*buffer*/,
                const Array<Element> & /*elements*/,
                const SynchronizationTag & /*tag*/) const override {}
  void unpackData(CommunicationBuffer & /*buffer*/,
                  const Array<Element> & /*elements*/,
                  const SynchronizationTag & /*tag*/) override {}

  // DataAccessor<UInt> nodes
  UInt getNbData(const Array<UInt> & /*nodes*/,
                 const SynchronizationTag & /*tag*/) const override {
    return 0;
  }
  void packData(CommunicationBuffer & /*buffer*/, const Array<UInt> & /*nodes*/,
                const SynchronizationTag & /*tag*/) const override {}
  void unpackData(CommunicationBuffer & /*buffer*/,
                  const Array<UInt> & /*nodes*/,
                  const SynchronizationTag & /*tag*/) override {}

  /* ------------------------------------------------------------------------ */
  /* Accessors                                                                */
  /* ------------------------------------------------------------------------ */
public:
  /// get the solid mechanics model
#if defined(AKANTU_COHESIVE_ELEMENT)
  template <class Model_ = SolidMechanicsModelType,
            std::enable_if_t<std::is_same<
                Model_, SolidMechanicsModelCohesive>::value> * = nullptr>
  SolidMechanicsModelCohesive & getSolidMechanicsModelCohesive() {
    return *solid;
  }
#endif
  template <class Model_ = SolidMechanicsModelType,
            std::enable_if_t<
                std::is_same<Model_, SolidMechanicsModel>::value> * = nullptr>
  SolidMechanicsModelType & getSolidMechanicsModel() {
    return *solid;
  }

  /// get the contact mechanics model
  AKANTU_GET_MACRO(ContactMechanicsModel, *contact, ContactMechanicsModel &)

  /* ------------------------------------------------------------------------ */
  /* Dumpable interface                                                       */
  /* ------------------------------------------------------------------------ */
public:
  std::shared_ptr<dumpers::Field>
  createNodalFieldReal(const std::string & field_name,
                       const std::string & group_name,
                       bool padding_flag) override;

  std::shared_ptr<dumpers::Field>
  createNodalFieldUInt(const std::string & field_name,
                       const std::string & group_name,
                       bool padding_flag) override;

  std::shared_ptr<dumpers::Field>
  createNodalFieldBool(const std::string & field_name,
                       const std::string & group_name,
                       bool padding_flag) override;

  std::shared_ptr<dumpers::Field>
  createElementalField(const std::string & field_name,
                       const std::string & group_name, bool padding_flag,
                       UInt spatial_dimension, ElementKind kind) override;

  void dump(const std::string & dumper_name) override;
  void dump(const std::string & dumper_name, UInt step) override;
  void dump(const std::string & dumper_name, Real time, UInt step) override;

  void dump() override;

  void dump(UInt step) override;
  void dump(Real time, UInt step) override;

  /* ------------------------------------------------------------------------ */
  /* Members                                                                  */
  /* ------------------------------------------------------------------------ */
private:
  /// solid mechanics model
  std::unique_ptr<SolidMechanicsModelType> solid;

  /// contact mechanics model
  std::unique_ptr<ContactMechanicsModel> contact;

  UInt step;
};

using CouplerSolidContact = CouplerSolidContactTemplate<SolidMechanicsModel>;

} // namespace akantu

#include "coupler_solid_contact_tmpl.hh"

#endif /* __COUPLER_SOLID_CONTACT_HH__  */
