#ifndef CONTACT_SENSOR_H
#define CONTACT_SENSOR_H

#include "predef.h"
#include "triangle_mesh.h"
#include "opcode/Opcode.h"

#define NODE_TRIANGLE 0x0001
#define TRIANGLE_NODE 0x0002
#define EDGE_EDGE	    0x0004
#define NODE_EDGE	    0x0008
#define EDGE_NODE	    0x0010
#define NODE_NODE	    0x0020

namespace tricycle {


class OpcodeWrapper {
public:
  OpcodeWrapper() {};
  virtual~OpcodeWrapper() {};

  void SetModel(unsigned modelid, const sptr<TriangleMesh> &trimesh);
  void Initialize();
  
  const sptr<Opcode::AABBTreeCollider> &GetCollider() const { return collider_; }
  const sptr<Opcode::BVTCache> &GetCache() const { return cache_; }
  const sptr<Opcode::Model> &GetModel(unsigned i) const { return helpers_[i].model; }
  const uptr<IceMaths::Point[]> &GetVertices(int id) { return helpers_[id].vertices; }

private:
  void TransformMeshFrom(unsigned modelid,
                          const std::vector<std::array<unsigned, 3>> &element_indices,
                          const std::vector<Eigen::Vec3> &nodes_coordinates);
  
  void MapNodalCoordniates(int id, const sptr<TriangleMesh> &trimesh);

private:
  sptr<Opcode::BVTCache> cache_;
  sptr<Opcode::AABBTreeCollider> collider_;

  struct OpcodeWrapperHelper {
    uptr<IceMaths::IndexedTriangle[]> indices;
    uptr<IceMaths::Point[]> vertices;
    sptr<Opcode::MeshInterface> meshinterface;
    sptr<Opcode::Model> model;
  };

  std::array<OpcodeWrapperHelper,2> helpers_;
};


class TRICYCLE_API ContactSensor {
protected:

  OpcodeWrapper opcodewrapper_;
  
  sptr<TriangleMesh> triangle_mesh0_;
  sptr<TriangleMesh> triangle_mesh1_;

  int surf_direction0_, surf_direction1_;
  
  // contacted triangle pairs
  
  typedef std::vector<std::pair<unsigned, unsigned>> pairsCollection;
  pairsCollection contacted_triangles_;
  std::vector<IceCore::contactStruct> contacted_types_;

  bool contact_found_;

  pairsCollection node_triangle_;
  pairsCollection triangle_node_;
  pairsCollection edge_edge_;
  pairsCollection node_node_;  // special case
  pairsCollection node_edge_;  // special case
  pairsCollection edge_node_;  // special case
  pairsCollection node_triangle_missed_;
  pairsCollection triangle_node_missed_;  
  
  // find pairs
  inline bool FindPair(const std::vector<std::pair<unsigned, unsigned>> &vec, const unsigned a, const unsigned b) {
    return std::find(vec.begin(), vec.end(), std::pair<unsigned, unsigned>(a, b)) != vec.end();
  }

  inline bool FindPairNodeFirst(const std::vector<std::pair<unsigned, unsigned>> &vec, const unsigned nodeid) {
    return std::find_if(vec.begin(), vec.end(),
      [nodeid](std::pair<unsigned, unsigned> const& obj) {return obj.first == nodeid; }) != vec.end();
  }

  inline bool FindPairNodeSecond(const std::vector<std::pair<unsigned, unsigned>> &vec, const unsigned nodeid) {
    return std::find_if(vec.begin(), vec.end(),
      [nodeid](std::pair<unsigned, unsigned> const& obj) {return obj.second == nodeid; }) != vec.end();
  }

  // distance calculation
  void GetDistanceNodeTriangle(const Eigen::Vec3 &rnode0, const Eigen::Vec3 &rnode1, const Eigen::Vec3 &rnode2,
                                const Eigen::Vec3 &rtri0, const Eigen::Vec3 &rtri1, const Eigen::Vec3 &rtri2,
                                Eigen::Vec3 &penetration, int surfdir);

  void GetDistanceLineLine(Eigen::Vec3 &r0, Eigen::Vec3 &r1, Eigen::Vec3 &r2, Eigen::Vec3 &r3, double &penetration);

  void AcceptEdgeEdgebyXi(const Eigen::Vec3 &r1, const Eigen::Vec3 &r2, const Eigen::Vec3 &r3, const Eigen::Vec3 &r4,
    unsigned edge0, unsigned edge1, int &primitiveType, unsigned &id0, unsigned &id1);

  bool AcceptEdgeEdgebyNormalDirection(const Eigen::Vec3 &r12, const Eigen::Vec3 &r34, const unsigned &edge0, const unsigned &edge1);

  void AcceptNodeTrianglebyXi(const Eigen::Vec3 &r31, const Eigen::Vec3 &r32, const Eigen::Vec3 &r3s, bool nt, int &primitiveType, int &lid1);

  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // get contact pairs, restore them
  bool RestorePairs();
  void RestorePrimitives();
  
public:
  ContactSensor() :contact_found_(0) {};
  virtual ~ContactSensor() {};

  static double xi_epsilon;
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  const sptr<Opcode::AABBTreeCollider> &GetCollider() const { return opcodewrapper_.GetCollider(); };
  const sptr<Opcode::BVTCache> &GetCache() const { return opcodewrapper_.GetCache(); }
  // input two model information that are potential contact pair  
  
  void SetMeshes(const sptr<TriangleMesh> &trimesh0, const sptr<TriangleMesh> &trimesh1, int ps, int ns);
  void Initialize();
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  // check if contacted or not

  BOOL CheckContact();

  bool GetIsContact() { return contact_found_; }
  unsigned GetPairNum() { return GetCollider()->GetNbPairs(); }
  std::vector<std::pair<unsigned, unsigned>> &GetPrimitivesPairs() { return contacted_triangles_; }
  std::vector<std::pair<unsigned, unsigned>> &GetPrimitivesPairs(int type) {
    switch (type) {
    case NODE_TRIANGLE:
      return node_triangle_;
    case TRIANGLE_NODE:
      return triangle_node_;
    case EDGE_EDGE:
      return edge_edge_;
    case NODE_EDGE:
      return node_edge_;
    case EDGE_NODE:
      return edge_node_;
    case NODE_NODE:
      return node_node_;
    default:
      break;
    }
    return contacted_triangles_;
  }
  
};


}



#endif // !CONTACTSENSOR_H
