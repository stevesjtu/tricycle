#ifndef TRIANGLE_MESH_H
#define TRIANGLE_MESH_H

#include"predef.h"
#include<set>
#include<array>
#include<string>
#include<fstream>
#include<iostream>

namespace tricycle {


class TRICYCLE_API TriangleMesh {
public:
  virtual ~TriangleMesh() {};
  TriangleMesh(unsigned nofe = 3) : num_node_of_element_(nofe) {}
  void SetMesh(const std::vector<std::array<unsigned, 3>> &elemin, const std::vector<Eigen::Vec3> &nodein);
  void SetMeshFromPlainText(const std::string &file);

  void SetDisplacement(Eigen::Ref<Eigen::MatrixXd> node_coordniates) {
    *rnodes_ = rnodes0_ + node_coordniates;
  }

  void SetRnode(const Eigen::MatrixXd &node_coordniates) {
    *rnodes_ = node_coordniates;
  }
  void SetRnode(const Eigen::Ref<const Eigen::MatrixXd> &node_coordniates) {
    *rnodes_ = node_coordniates;
  }
  void SetRnode(unsigned nid, Eigen::Vec3 coordinate) {
    GetRnode(nid) = coordinate;
  }

  void SetRnode(const double *node_coordinates) {
    for (unsigned i = 0; i < num_node_ * 3; ++i) {
      (*rnodes_)(i) = node_coordinates[i];
    }
  }
  void SetRnode(unsigned nid, const double *node_coordinates) {
    GetRnode(nid) << node_coordinates[0], node_coordinates[1], node_coordinates[2];
  }

  void UpdateSurfaceNormal();
  Eigen::Vec3 GetNodeNormal(unsigned int id);
  Eigen::Vec3 GetEdgeNormal(unsigned id);
  Eigen::Refv3 GetNormal(unsigned id); 
  double GetArea(unsigned id);
  Eigen::Refv3 GetRnode(unsigned id);
  void GetElemRnode(unsigned id, Eigen::Refv3 r1, Eigen::Refv3 r2, Eigen::Refv3 r3); 
  void MakeElementBoundary();

public:
  std::vector<std::array<unsigned, 3>> elems_bynodes_;
  std::vector<std::array<unsigned, 3>> elems_byedges_;
  std::vector<std::array<unsigned, 2>> edges_bynodes_;
  std::vector<std::vector<unsigned>> nodes_byelems_;  
  std::vector<std::vector<unsigned>> edges_byelems_;
  std::vector<Eigen::Vec3> nodes_;

  std::vector<Eigen::Vec3> normals_ofelems_;
  std::vector<double> areas_;
  Eigen::MatrixXd rnodes0_;
  sptr<Eigen::Map<Eigen::MatrixXd>> rnodes_;

  unsigned num_node_of_element_;
  unsigned num_elem_, num_node_, num_edge_;

};


}

#endif
