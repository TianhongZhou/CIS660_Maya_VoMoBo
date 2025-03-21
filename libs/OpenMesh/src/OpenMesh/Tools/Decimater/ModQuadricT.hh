/* ========================================================================= *
 *                                                                           *
 *                               OpenMesh                                    *
 *           Copyright (c) 2001-2015, RWTH-Aachen University                 *
 *           Department of Computer Graphics and Multimedia                  *
 *                          All rights reserved.                             *
 *                            www.openmesh.org                               *
 *                                                                           *
 *---------------------------------------------------------------------------*
 * This file is part of OpenMesh.                                            *
 *---------------------------------------------------------------------------*
 *                                                                           *
 * Redistribution and use in source and binary forms, with or without        *
 * modification, are permitted provided that the following conditions        *
 * are met:                                                                  *
 *                                                                           *
 * 1. Redistributions of source code must retain the above copyright notice, *
 *    this list of conditions and the following disclaimer.                  *
 *                                                                           *
 * 2. Redistributions in binary form must reproduce the above copyright      *
 *    notice, this list of conditions and the following disclaimer in the    *
 *    documentation and/or other materials provided with the distribution.   *
 *                                                                           *
 * 3. Neither the name of the copyright holder nor the names of its          *
 *    contributors may be used to endorse or promote products derived from   *
 *    this software without specific prior written permission.               *
 *                                                                           *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS       *
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED *
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A           *
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER *
 * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,  *
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,       *
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR        *
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF    *
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING      *
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS        *
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.              *
 *                                                                           *
 * ========================================================================= */

//=============================================================================
//
//  CLASS ModQuadricT
//
//=============================================================================

#ifndef OSG_MODQUADRIC_HH
#define OSG_MODQUADRIC_HH


//== INCLUDES =================================================================

#include <float.h>
#include <OpenMesh/Tools/Decimater/ModBaseT.hh>
#include <OpenMesh/Core/Utils/Property.hh>
#include <OpenMesh/Core/Utils/vector_cast.hh>
#include <OpenMesh/Core/Geometry/QuadricT.hh>
#include <qpOASES.hpp>
#include <maya/MGlobal.h>

typedef OpenMesh::TriMesh_ArrayKernelT<>  MyMesh;


//== NAMESPACE ================================================================

namespace OpenMesh  {
namespace Decimater {


//== CLASS DEFINITION =========================================================


/** \brief Mesh decimation module computing collapse priority based on error quadrics.
 *
 *  This module can be used as a binary and non-binary module.
 */
template <class MeshT>
class ModQuadricT : public ModBaseT<MeshT>
{
public:

  // Defines the types Self, Handle, Base, Mesh, and CollapseInfo
  // and the memberfunction name()
  DECIMATING_MODULE( ModQuadricT, MeshT, Quadric );

public:

  /** Constructor
   *  \internal
   */
  explicit ModQuadricT( MeshT &_mesh )
    : Base(_mesh, false)
  {
    unset_max_err();
    Base::mesh().add_property( quadrics_ );
  }


  /// Destructor
  virtual ~ModQuadricT()
  {
    Base::mesh().remove_property(quadrics_);
  }


public: // inherited

  /// Initalize the module and prepare the mesh for decimation.
  virtual void initialize(void) override;

  /** Compute collapse priority based on error quadrics.
   *
   *  \see ModBaseT::collapse_priority() for return values
   *  \see set_max_err()
   */
  virtual float collapse_priority(const CollapseInfo& _ci) override
  {
    // CQEM based-on "Bounding Proxies for Shape Approximation Supplemental Materials"
    
    using namespace OpenMesh;

    typedef Geometry::QuadricT<double> Q;

    Q q = Base::mesh().property(quadrics_, _ci.v0);
    q += Base::mesh().property(quadrics_, _ci.v1);

    // Generate Q4x4
    Eigen::Matrix4d Q4x4;
    Q4x4(0, 0) = q.xx();  Q4x4(0, 1) = q.xy();  Q4x4(0, 2) = q.xz();  Q4x4(0, 3) = q.xw();
    Q4x4(1, 0) = q.xy();  Q4x4(1, 1) = q.yy();  Q4x4(1, 2) = q.yz();  Q4x4(1, 3) = q.yw();
    Q4x4(2, 0) = q.xz();  Q4x4(2, 1) = q.yz();  Q4x4(2, 2) = q.zz();  Q4x4(2, 3) = q.zw();
    Q4x4(3, 0) = q.xw();  Q4x4(3, 1) = q.yw();  Q4x4(3, 2) = q.zw();  Q4x4(3, 3) = q.ww();

    std::vector<Eigen::Vector3d> triangle_normals;
    std::vector<double> triangle_d_values;

    // Loop through all faces involve v0 using half-edge struct
    for (MyMesh::VertexFaceIter vf_it = Base::mesh().vf_iter(_ci.v0); vf_it.is_valid(); ++vf_it) {
        MyMesh::FaceHandle fh = *vf_it;
        
        // Get all three vertices
        MyMesh::FaceVertexIter fv_it = Base::mesh().fv_iter(fh);
        Eigen::Vector3d v0_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);
        ++fv_it;
        Eigen::Vector3d v1_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);
        ++fv_it;
        Eigen::Vector3d v2_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);

        // Compute normal
        Eigen::Vector3d normal = (v1_pos - v0_pos).cross(v2_pos - v0_pos);
        normal.normalize();

        // Compute d
        double d = -normal.dot(v0_pos);

        triangle_normals.push_back(normal);
        triangle_d_values.push_back(d);
    }

    // Loop through all faces involve v1 using half-edge struct
    for (MyMesh::VertexFaceIter vf_it = Base::mesh().vf_iter(_ci.v1); vf_it.is_valid(); ++vf_it) {
        MyMesh::FaceHandle fh = *vf_it;

        // Get all three vertices
        MyMesh::FaceVertexIter fv_it = Base::mesh().fv_iter(fh);
        Eigen::Vector3d v0_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);
        ++fv_it;
        Eigen::Vector3d v1_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);
        ++fv_it;
        Eigen::Vector3d v2_pos(Base::mesh().point(*fv_it)[0], Base::mesh().point(*fv_it)[1], Base::mesh().point(*fv_it)[2]);

        // Compute normal
        Eigen::Vector3d normal = (v1_pos - v0_pos).cross(v2_pos - v0_pos);
        normal.normalize();

        // Compute d
        double d = -normal.dot(v0_pos);

        triangle_normals.push_back(normal);
        triangle_d_values.push_back(d);
    }

    // Compute A and b for QP
    int num_constraints = (int) triangle_normals.size();
    Eigen::MatrixXd A(num_constraints, 4);

    for (int i = 0; i < num_constraints; i++) {
        A.row(i).head<3>() = triangle_normals[i].transpose();
        A(i, 3) = triangle_d_values[i];
    }

    // qpOASES variables
    int total_constraints = num_constraints + 1;
    qpOASES::SQProblem qp(4, total_constraints);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_LOW;
    qp.setOptions(options);

    qpOASES::real_t H[16], g[4] = {0, 0, 0, 0};
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            H[i * 4 + j] = Q4x4(i, j);
        }
    }

    // Constraints
    std::vector<qpOASES::real_t> A_qp(total_constraints * 4);
    std::vector<qpOASES::real_t> lb(total_constraints);
    std::vector<qpOASES::real_t> ub(total_constraints);
    qpOASES::real_t* A_qp_ptr = A_qp.data();
    qpOASES::real_t* lb_ptr = lb.data();
    qpOASES::real_t* ub_ptr = ub.data();

    for (int i = 0; i < num_constraints; i++) {
        for (int j = 0; j < 4; j++) {
            A_qp[i * 4 + j] = A(i, j);
        }
        lb[i] = -1e-6;
        ub[i] = qpOASES::INFTY;
    }

    int extraRow = num_constraints;
    A_qp[extraRow * 4 + 0] = 0;
    A_qp[extraRow * 4 + 1] = 0;
    A_qp[extraRow * 4 + 2] = 0;
    A_qp[extraRow * 4 + 3] = 1;
    lb[extraRow] = 1;
    ub[extraRow] = 1;

    // Initialize solver
    int nWSR = 100;
    qp.init(H, g, A_qp_ptr, nullptr, nullptr, lb_ptr, ub_ptr, nWSR);

    // Solve QP
    qpOASES::real_t v_opt_qp[4];
    qp.getPrimalSolution(v_opt_qp);

    // Compute final cost
    
    // CQEM only
    const_cast<OpenMesh::VectorT<float, 3>&>(_ci.p1) = OpenMesh::VectorT<float, 3>(v_opt_qp[0], v_opt_qp[1], v_opt_qp[2]);
    double err_qem = q(_ci.p1);

    // Combine both CQEM and QEM
    //double err_qem = q(_ci.p1);
    //OpenMesh::VectorT<float, 3> cqem_p(v_opt_qp[0], v_opt_qp[1], v_opt_qp[2]);
    //double err_cqem = q(cqem_p);

    //if (err_cqem < err_qem) {
    //    const_cast<OpenMesh::VectorT<float, 3>&>(_ci.p1) = cqem_p;
    //    return float(err_cqem < max_err_ ? err_cqem : float(Base::ILLEGAL_COLLAPSE));
    //}

    return float(err_qem < max_err_ ? err_qem : float(Base::ILLEGAL_COLLAPSE));
  }


  /// Post-process halfedge collapse (accumulate quadrics)
  virtual void postprocess_collapse(const CollapseInfo& _ci) override
  {
    Base::mesh().property(quadrics_, _ci.v1) +=
      Base::mesh().property(quadrics_, _ci.v0);
  }

  /// set the percentage of maximum quadric error
  void set_error_tolerance_factor(double _factor) override;



public: // specific methods

  /** Set maximum quadric error constraint and enable binary mode.
   *  \param _err    Maximum error allowed
   *  \param _binary Let the module work in non-binary mode in spite of the
   *                 enabled constraint.
   *  \see unset_max_err()
   */
  void set_max_err(double _err, bool _binary=true)
  {
    max_err_ = _err;
    Base::set_binary(_binary);
  }

  /// Unset maximum quadric error constraint and restore non-binary mode.
  /// \see set_max_err()
  void unset_max_err(void)
  {
    max_err_ = DBL_MAX;
    Base::set_binary(false);
  }

  /// Return value of max. allowed error.
  double max_err() const { return max_err_; }


private:

  // maximum quadric error
  double max_err_;

  // this vertex property stores a quadric for each vertex
  VPropHandleT< Geometry::QuadricT<double> >  quadrics_;
};

//=============================================================================
} // END_NS_DECIMATER
} // END_NS_OPENMESH
//=============================================================================
#if defined(OM_INCLUDE_TEMPLATES) && !defined(OPENMESH_DECIMATER_MODQUADRIC_CC)
#define OSG_MODQUADRIC_TEMPLATES
#include "ModQuadricT_impl.hh"
#endif
//=============================================================================
#endif // OSG_MODQUADRIC_HH defined
//=============================================================================

