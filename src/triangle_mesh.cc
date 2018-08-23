#include"triangle_mesh.h"

namespace tricycle {


void TriangleMesh::SetMesh(const std::vector<std::array<unsigned, 3>> &elemin, const std::vector<Eigen::Vec3> &nodein) {
  elems_bynodes_ = elemin;
  nodes_ = nodein;
  
  num_elem_ = static_cast<unsigned>(elems_bynodes_.size());
  num_node_ = static_cast<unsigned>(nodes_.size());

  normals_ofelems_.resize(num_elem_);
  areas_.resize(num_elem_);

  this->MakeElementBoundary();
}

void TriangleMesh::SetMeshFromPlainText(const std::string &file) {
  std::ifstream infile(file);
  if (!infile.is_open()) {
    std::cout << "Error in opening file: " << file << std::endl;
    return;
  }

  infile >> num_elem_
    >> num_node_
    >> num_node_of_element_;

  elems_bynodes_.resize(num_elem_);
  nodes_.resize(num_node_);

  normals_ofelems_.resize(num_elem_);
  areas_.resize(num_elem_);

  for (unsigned elid = 0; elid < num_elem_; ++elid) {
    infile >> elems_bynodes_[elid][0]
      >> elems_bynodes_[elid][1]
      >> elems_bynodes_[elid][2];
  }

  for (unsigned i = 0; i < num_node_; ++i) {
    infile >> nodes_[i].x()
      >> nodes_[i].y()
      >> nodes_[i].z();
  }

  infile.close();
  this->MakeElementBoundary();
}

void TriangleMesh::MakeElementBoundary() {
  // edgeSet should be a set of edges without duplicated edges.
  typedef std::set<unsigned> edge;
  edge oneEdge;
  std::set<edge> edgeSet;
  unsigned ind1;
  for (unsigned e = 0; e < num_elem_; ++e) {
    for (unsigned ind = 0; ind < num_node_of_element_; ++ind) {
      ind == num_node_of_element_ - 1 ? ind1 = 0 : ind1 = ind + 1;
      oneEdge.insert(elems_bynodes_[e][ind]);
      oneEdge.insert(elems_bynodes_[e][ind1]);
      edgeSet.insert(oneEdge);
      oneEdge.clear();
    }
  }

  // fill the edges_bynodes_
  num_edge_ = static_cast<unsigned>(edgeSet.size());
  edges_bynodes_.resize(num_edge_);
  unsigned index = 0;
  for (std::set<edge>::iterator it = edgeSet.begin(); it != edgeSet.end(); it++) {
    edge::iterator nodeit = it->begin();
    edges_bynodes_[index][0] = *nodeit++;
    edges_bynodes_[index++][1] = *nodeit;
  }

  // make an indices list of edges
  elems_byedges_.resize(num_elem_);
  for (unsigned e = 0; e < num_elem_; ++e) {
    for (unsigned ind = 0; ind < num_node_of_element_; ++ind) {
      ind == num_node_of_element_ - 1 ? ind1 = 0 : ind1 = ind + 1;
      oneEdge.insert(elems_bynodes_[e][ind]);
      oneEdge.insert(elems_bynodes_[e][ind1]);
      index = 0;
      std::set<edge>::iterator it = edgeSet.begin();
      while (*it++ != oneEdge) ++index;
      elems_byedges_[e][ind] = index;
      oneEdge.clear();
    }
  }
  
  //////////////////////////////////////////////////////////////////////////////
  // process nodes_byelems_
  nodes_byelems_.resize(num_node_);
  for (unsigned e = 0; e < num_elem_; ++e) {
    for (unsigned i = 0; i < 3; ++i)
      nodes_byelems_[elems_bynodes_[e][i]].push_back(e);
  }

  // process edges_byelems_

  edges_byelems_.resize(num_edge_);
  for (unsigned e = 0; e < num_elem_; ++e) {
    for (unsigned i = 0; i < 3; ++i) {
      edges_byelems_[elems_byedges_[e][i]].push_back(e);
    } 
  }
  
} // MakeElementBoundary


void TriangleMesh::UpdateSurfaceNormal() {
  Eigen::Vector3d r1, r2, r3, v3;
  for (unsigned int e = 0; e < num_elem_; ++e) {
    r1 = rnodes_->col(elems_bynodes_[e][0]);
    r2 = rnodes_->col(elems_bynodes_[e][1]);
    r3 = rnodes_->col(elems_bynodes_[e][2]);
    v3 = (r2 - r1).cross(r3 - r1);
    v3.normalize();
    normals_ofelems_[e] = v3;
  }
}

Eigen::Vec3 TriangleMesh::GetNodeNormal(unsigned int id) {
  auto &node_byelems = nodes_byelems_[id];
  unsigned neighbor_num = (unsigned)node_byelems.size();
  Eigen::Vec3 normal = Eigen::ZV3;
  for (unsigned i = 0; i < neighbor_num; ++i)
    normal += normals_ofelems_[node_byelems[i]];
  normal.normalize();

  return normal;
}

Eigen::Vec3 TriangleMesh::GetEdgeNormal(unsigned id) {
  auto &edge_byelems = edges_byelems_[id];
  unsigned neighbor_num = (unsigned)edge_byelems.size();
  Eigen::Vec3 normal = Eigen::ZV3;
  for (unsigned i = 0; i < neighbor_num; ++i)
    normal += normals_ofelems_[edge_byelems[i]];
  normal.normalize();
  return normal;
}

Eigen::Refv3 TriangleMesh::GetNormal(unsigned id) { return normals_ofelems_[id]; }
double TriangleMesh::GetArea(unsigned id) { return areas_[id]; }
Eigen::Refv3 TriangleMesh::GetRnode(unsigned id) { return rnodes_->col(id); }

void TriangleMesh::GetElemRnode(unsigned id, Eigen::Refv3 r1, Eigen::Refv3 r2, Eigen::Refv3 r3) {
  unsigned n1 = elems_bynodes_[0][id];
  unsigned n2 = elems_bynodes_[1][id];
  unsigned n3 = elems_bynodes_[2][id];

  r1 = rnodes_->col(n1);
  r2 = rnodes_->col(n2);
  r3 = rnodes_->col(n3);
}


}
