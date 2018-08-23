#include "contact_sensor.h"

namespace tricycle {


double ContactSensor::xi_epsilon = 1e-6;

void OpcodeWrapper::SetModel(unsigned modelid, const sptr<TriangleMesh> &trimesh) {
  // mesh interface
  helpers_[modelid].meshinterface = New<Opcode::MeshInterface>();
  auto &meshinterface = helpers_[modelid].meshinterface;
  meshinterface->SetNbTriangles(trimesh->num_elem_);
  meshinterface->SetNbVertices(trimesh->num_node_);

  this->TransformMeshFrom(modelid, trimesh->elems_bynodes_, trimesh->nodes_);

  this->MapNodalCoordniates(modelid, trimesh);

  meshinterface->SetPointers(helpers_[modelid].indices.get(), helpers_[modelid].vertices.get());

  unsigned degeneratedFaces = meshinterface->CheckTopology();
  assert(degeneratedFaces == 0);
  assert(meshinterface->IsValid());

  // model
  Opcode::BuildSettings buildSettings;
  buildSettings.mLimit = 1;
  buildSettings.mRules = Opcode::SplittingRules::SPLIT_BEST_AXIS | Opcode::SplittingRules::SPLIT_SPLATTER_POINTS | Opcode::SplittingRules::SPLIT_GEOM_CENTER;

  Opcode::OPCODECREATE data;
  data.mCanRemap = false;
  data.mKeepOriginal = false;
  data.mNoLeaf = true;    // for refit function, must be a NoLeaf tree
  data.mQuantized = false;   // for refit function, Quantizing must be discarded
  data.mSettings = buildSettings;
  data.mIMesh = meshinterface.get();

  helpers_[modelid].model = New<Opcode::Model>();
  auto &model = helpers_[modelid].model;
  bool modelCreated = model->Build(data);
  assert(modelCreated);

}

void OpcodeWrapper::Initialize() {
  cache_ = New<Opcode::BVTCache>();
  cache_->Model0 = helpers_[0].model.get();
  cache_->Model1 = helpers_[1].model.get();

  collider_ = New<Opcode::AABBTreeCollider>();
}

void OpcodeWrapper::TransformMeshFrom(unsigned modelid,
                                      const std::vector<std::array<unsigned, 3>> &element_indices,
                                      const std::vector<Eigen::Vec3> &nodes_coordinates) {

  unsigned num_elem = (unsigned)element_indices.size();
  unsigned num_node = (unsigned)nodes_coordinates.size();

  helpers_[modelid].indices.reset(new IceMaths::IndexedTriangle[num_elem]);
  helpers_[modelid].vertices.reset(new IceMaths::Point[num_node]);

  for (unsigned e = 0; e < num_elem; ++e) {
    helpers_[modelid].indices[e] = IceMaths::IndexedTriangle(element_indices[e][0], element_indices[e][1], element_indices[e][2]);
  }

  for (unsigned n = 0; n < num_node; ++n) {
    helpers_[modelid].vertices[n] = IceMaths::Point(nodes_coordinates[n].data());
  }

}

void OpcodeWrapper::MapNodalCoordniates(int id, const sptr<TriangleMesh> &trimesh) {
  double *const coordinateOrgin = &(helpers_[id].vertices[0].x);
  trimesh->rnodes_ = New<Eigen::Map<Eigen::MatrixXd>>(coordinateOrgin, 3, trimesh->num_node_);
  trimesh->rnodes0_.resize(3, trimesh->num_node_);

  for (unsigned i = 0; i < trimesh->num_node_; ++i) {
    trimesh->rnodes_->col(i) = trimesh->nodes_[i];
    trimesh->rnodes0_.col(i) = trimesh->nodes_[i];
  }

}

void ContactSensor::SetMeshes(const sptr<TriangleMesh> &trimesh0, const sptr<TriangleMesh> &trimesh1, int ps, int ns) {
  triangle_mesh0_ = trimesh0;
  triangle_mesh1_ = trimesh1;
  surf_direction0_ = ps;
  surf_direction1_ = ns;
}

void ContactSensor::Initialize() {
  opcodewrapper_.SetModel(0, triangle_mesh0_);
  opcodewrapper_.SetModel(1, triangle_mesh1_);
  opcodewrapper_.Initialize();

}

void ContactSensor::GetDistanceNodeTriangle(const Eigen::Vec3 &rnode0, const Eigen::Vec3 &rnode1, const Eigen::Vec3 &rnode2,
	                                          const Eigen::Vec3 &rtri0, const Eigen::Vec3 &rtri1, const Eigen::Vec3 &rtri2,
	                                          Eigen::Vec3 &penetration, int surfdir) {
	Eigen::Vec3 norml = (rtri1 - rtri0).cross(rtri2 - rtri0);
	norml.normalize();
	norml *= surfdir;
	penetration(0) = norml.dot(rnode0 - rtri0);
	penetration(1) = norml.dot(rnode1 - rtri0);
	penetration(2) = norml.dot(rnode2 - rtri0);
}

void ContactSensor::GetDistanceLineLine(Eigen::Vec3 &r0, Eigen::Vec3 &r1, Eigen::Vec3 &r2, Eigen::Vec3 &r3, double &penetration) {
	Eigen::Vec3 norml = (r1 - r0).cross(r3 - r2);
	norml.normalize();
	penetration = norml.dot(r0 - r2);
}

void ContactSensor::AcceptEdgeEdgebyXi(const Eigen::Vec3 &r1, const Eigen::Vec3 &r2, const Eigen::Vec3 &r3, const Eigen::Vec3 &r4,
	unsigned edge0, unsigned edge1, int &primitiveType, unsigned &id0, unsigned &id1) {
	Eigen::MatrixXd Jmat(3, 2);
	Jmat.leftCols(1) = r2 - r1;
	Jmat.rightCols(1) = r3 - r4;
  Eigen::Matrix2d A2 = Jmat.transpose()* Jmat;
  Eigen::Vector2d b2 = Jmat.transpose()* (r3 - r1);
  Eigen::Vector2d xi;

	solve22(A2, b2, xi);

	id0 = id1 = 2;
	primitiveType = 0;

	const double upLimit = 1.0 + xi_epsilon;
	const double downLimit = -xi_epsilon;

	if (xi(0) > upLimit || xi(1) > upLimit || xi(0) < downLimit || xi(1) < downLimit) return;

	Eigen::Vec3 rsm = (1.0 - xi(1))* r3 + xi(1)* r4 - (1.0 - xi(0))* r1 - xi(0)* r2;

	if (-surf_direction0_* rsm.dot(triangle_mesh0_->GetEdgeNormal(edge0)) < 0.0) return;
	if (surf_direction1_* rsm.dot(triangle_mesh1_->GetEdgeNormal(edge1)) < 0.0) return;

	if (equals(xi(0), 0.0, xi_epsilon)) {  // a = 0, r1 attach to r34
		primitiveType |= NODE_EDGE;
		id0 = 0;
	}
	else if (equals(xi(0), 1.0, xi_epsilon)) { // a = 1, r2 attach to r34
		primitiveType |= NODE_EDGE;
		id0 = 1;
	}

	if (equals(xi(1), 0.0, xi_epsilon)) { // b= 0, r3 attach to r12
		primitiveType |= EDGE_NODE;
		id1 = 0;
	}
	else if (equals(xi(1), 1.0, xi_epsilon)) { // b = 1, r4 attach to r12
		primitiveType |= EDGE_NODE;
		id1 = 1;
	}

	if ((primitiveType & NODE_EDGE) && (primitiveType & EDGE_NODE)) {
		primitiveType = NODE_NODE;
	}

	if (primitiveType == 0)
		primitiveType = EDGE_EDGE;

}


bool ContactSensor::AcceptEdgeEdgebyNormalDirection(const Eigen::Vec3 &r12, const Eigen::Vec3 &r34, const unsigned &edge0, const unsigned &edge1) {
	
	Eigen::Vec3 normal = r12.cross(r34);
	normal.normalize();

	if (abs(normal.dot(triangle_mesh0_->GetEdgeNormal(edge0))) < 0.70710678118) return false;
	if (abs(normal.dot(triangle_mesh1_->GetEdgeNormal(edge1))) < 0.70710678118) return false;
			
	return true;
}

void ContactSensor::AcceptNodeTrianglebyXi(const Eigen::Vec3 &r31, const Eigen::Vec3 &r32, const Eigen::Vec3 &r3s, bool nt, int &primitiveType, int &lid1) {
  
	Eigen::MatrixXd Jmat(3, 2);
	Jmat.leftCols(1) = r31;
	Jmat.rightCols(1) = r32;
	Eigen::Matrix2d A1 = Jmat.transpose()* Jmat;
	Eigen::Vector2d b1 = Jmat.transpose()* r3s;
	Eigen::Vector2d xi;

	solve22(A1, b1, xi);

	primitiveType = 0;

	const double upLimit = 1.0 + ContactSensor::xi_epsilon;
	const double downLimit = -ContactSensor::xi_epsilon;

	if (xi(0) > upLimit || xi(1) > upLimit || xi(0) < downLimit || xi(1) < downLimit || xi.sum() > upLimit || xi.sum() < downLimit)
		return;

	// nt = true node_triangle_
	// nt = false triangle_node_
	int nodeedge, nodetriangle;
	if (nt) {
		nodeedge = NODE_EDGE;
		nodetriangle = NODE_TRIANGLE;
	}
	else {
		nodeedge = EDGE_NODE;
		nodetriangle = TRIANGLE_NODE;
	}

	int type = 0;
	if (equals(xi(0), 0.0, ContactSensor::xi_epsilon)) {
		type += 123;
		primitiveType = nodeedge;
		lid1 = 1;
	}

	if (equals(xi(1), 0.0, ContactSensor::xi_epsilon)) {
		type += 131;
		primitiveType = nodeedge;
		lid1 = 2;
	}

	if (equals(xi(0) + xi(1), 1.0, ContactSensor::xi_epsilon)) {
		type += 112;
		primitiveType = nodeedge;
		lid1 = 0;
	}

	//
	if (type > 200) {
		primitiveType = NODE_NODE;
		switch (type)
		{
		case 254:
			lid1 = 2;
			break;
		case 235:
			lid1 = 1;
			break;
		case 243:
			lid1 = 0;
			break;
		default:
			break;
		}
	}

	if (type == 0)
		primitiveType = nodetriangle;

}

bool ContactSensor::RestorePairs() {
	const unsigned pairNum = GetCollider()->GetNbPairs();
	const IceCore::Pair *p = GetCollider()->GetPairs();

	contacted_triangles_.clear();
	for (unsigned int i = 0; i < pairNum; ++i) {
		contacted_triangles_.push_back(std::make_pair(p->id0, p->id1));
		p++;
	}
	contacted_types_ = GetCollider()->GetContactArgs();
	return !contacted_triangles_.empty();

}

// triID1 in pBody, triID2 in nBody
void ContactSensor::RestorePrimitives() {
	unsigned truePairNum = (unsigned)contacted_triangles_.size();
	auto &elem0 = triangle_mesh0_->elems_bynodes_;
	auto &elem1 = triangle_mesh1_->elems_bynodes_;

	auto &elem_edges0 = triangle_mesh0_->elems_byedges_;
	auto &elem_edges1 = triangle_mesh1_->elems_byedges_;

	auto &edge0bynode = triangle_mesh0_->edges_bynodes_;
	auto &edge1bynode = triangle_mesh1_->edges_bynodes_;
  
	IceCore::contactStruct contactArgs;

	unsigned Node0, Node1, Node2, triangleID, triangleID0, triangleID1, n0, n1, n2;
	int primitiveType = 0;
	Eigen::Vec3 penetration0, penetration, penetration1;
	Eigen::Vec3 rnode0, rnode1, rnode2, rtri0, rtri1, rtri2;
	Eigen::Vec3 r1, r2, r3, r4;

	// filt the primitive type into specific container
	std::vector<unsigned> eeid, ntid, tnid;
	for (unsigned i = 0; i < truePairNum; ++i) {
		contactArgs = contacted_types_[i];
		if (contactArgs.primitiveType & NODE_TRIANGLE)
			ntid.push_back(i);
		else if (contactArgs.primitiveType & TRIANGLE_NODE)
			tnid.push_back(i);
		else if (contactArgs.primitiveType & EDGE_EDGE)
			eeid.push_back(i);
	}

	// clear the primitive pairs
	node_triangle_.clear();
	triangle_node_.clear();
	edge_edge_.clear();

	node_edge_.clear();
	edge_node_.clear();
	node_node_.clear();

	// first, filt the edge_edge_ contact
	// some of edge-edge contacts at corner should convert to node_triangle_ or triangle_node_ contact
	for (auto &i : eeid) {
		contactArgs = contacted_types_[i];
		//////////////////////////////////////////////////////////////
		// get number of nodes_ and nodes_ in edges ////////////////////
		triangleID0 = contacted_triangles_[i].first;
		triangleID1 = contacted_triangles_[i].second;
		const unsigned &edge0 = elem_edges0[triangleID0][contactArgs.edgePair[0]];
		const unsigned &edge1 = elem_edges1[triangleID1][contactArgs.edgePair[1]];

		if (FindPair(edge_edge_, edge0, edge1)) continue;

		r1 = triangle_mesh0_->GetRnode(edge0bynode[edge0][0]);
		r2 = triangle_mesh0_->GetRnode(edge0bynode[edge0][1]);
		r3 = triangle_mesh1_->GetRnode(edge1bynode[edge1][0]);
		r4 = triangle_mesh1_->GetRnode(edge1bynode[edge1][1]);

		unsigned lid0, lid1, nodeid0, nodeid1;
		AcceptEdgeEdgebyXi(r1, r2, r3, r4, edge0, edge1, primitiveType, lid0, lid1);
		if (primitiveType & EDGE_EDGE) {
			if (AcceptEdgeEdgebyNormalDirection(r2 - r1, r4 - r3, edge0, edge1))
				edge_edge_.push_back(std::pair<unsigned, unsigned>(edge0, edge1));
		}
		else {

			lid0 != 2 ? nodeid0 = edge0bynode[edge0][lid0] : 0;
			lid1 != 2 ? nodeid1 = edge1bynode[edge1][lid1] : 0;

			if (primitiveType & NODE_NODE) {
				if (!FindPairNodeFirst(node_node_, nodeid0)
					&& !FindPairNodeSecond(node_node_, nodeid1)
					&& !FindPairNodeFirst(node_edge_, nodeid0))
				{
					node_node_.push_back(std::pair<unsigned, unsigned>(nodeid0, nodeid1));
				}
			}
			else if (primitiveType & NODE_EDGE) {
				if (!FindPairNodeFirst(node_node_, nodeid0)
					&& !FindPairNodeFirst(node_edge_, nodeid0))
				{
					node_edge_.push_back(std::pair<unsigned, unsigned>(nodeid0, edge1));
				}
			}
			else if (primitiveType & EDGE_NODE) {
				if (!FindPairNodeSecond(node_node_, nodeid1)
					&& !FindPairNodeSecond(edge_node_, nodeid1))
				{
					edge_node_.push_back(std::pair<unsigned, unsigned>(edge0, nodeid1));
				}
			}
		}
	}

	// second, filt the node_triangle_ contact
	int lid;
	bool isduplicated0, isduplicated1, isduplicated2;
	bool nopenetr0, nopenetr1, nopenetr2;

	for (auto &i : ntid) {
		//////////////////////////////////////////////////////////////
		// get number of nodes_ and nodes_ in triangle /////////////////

		Node0 = elem0[contacted_triangles_[i].first][0];
		Node1 = elem0[contacted_triangles_[i].first][1];
		Node2 = elem0[contacted_triangles_[i].first][2];

		isduplicated0 = isduplicated1 = isduplicated2 = false;
		if (FindPairNodeFirst(node_triangle_, Node0)
			|| FindPairNodeFirst(node_edge_, Node0)
			|| FindPairNodeFirst(node_node_, Node0)) isduplicated0 = true;

		if (FindPairNodeFirst(node_triangle_, Node1)
			|| FindPairNodeFirst(node_edge_, Node1)
			|| FindPairNodeFirst(node_node_, Node1)) isduplicated1 = true;

		if (FindPairNodeFirst(node_triangle_, Node2)
			|| FindPairNodeFirst(node_edge_, Node2)
			|| FindPairNodeFirst(node_node_, Node2)) isduplicated2 = true;

		if (isduplicated0 && isduplicated1 && isduplicated2) continue;

		triangleID = contacted_triangles_[i].second;
		n0 = elem1[triangleID][0];
		n1 = elem1[triangleID][1];
		n2 = elem1[triangleID][2];

		///////////////////////////////////////////////////////////////
		// get position of contact nodes_ //////////////////////////////

		rnode0 = triangle_mesh0_->GetRnode(Node0);
		rnode1 = triangle_mesh0_->GetRnode(Node1);
		rnode2 = triangle_mesh0_->GetRnode(Node2);
		rtri0 = triangle_mesh1_->GetRnode(n0);
		rtri1 = triangle_mesh1_->GetRnode(n1);
		rtri2 = triangle_mesh1_->GetRnode(n2);

		///////////////////////////////////////////////////////////////
		// get 3 nodes_ distances to the triangle surface. /////////////
		GetDistanceNodeTriangle(rnode0, rnode1, rnode2, rtri0, rtri1, rtri2, penetration1, surf_direction1_);
		///////////////////////////////////////////////////////////////
		// judge the pair if it is the contacted pair.
		nopenetr0 = nopenetr1 = nopenetr2 = false;

		if (!isduplicated0) {
			if (penetration1(0) < 0.0) {
				//if (std::find(pExtNodes.begin(), pExtNodes.end(), Node0) != pExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode0 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node0, triangleID));
				}
				else if ((primitiveType & NODE_EDGE)) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node0, elem_edges1[triangleID][lid]));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node0, elem1[triangleID][lid]));
				}
				else if (!primitiveType) {
					nopenetr0 = true;
				}
			}
			else {
				nopenetr0 = true;
			}
		}

		if (!isduplicated1) {
			if (penetration1(1) < 0.0) {
				//if (std::find(pExtNodes.begin(), pExtNodes.end(), Node1) != pExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode1 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node1, triangleID));
				}
				else if (primitiveType & NODE_EDGE) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node1, elem_edges1[triangleID][lid] ));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node1, elem1[triangleID][lid] ));
				}
				else if (!primitiveType) {
					nopenetr1 = true;
				}
			}
			else {
				nopenetr1 = true;
			}
		}

		if (!isduplicated2) {
			if (penetration1(2) < 0.0) {
				//if (std::find(pExtNodes.begin(), pExtNodes.end(), Node2) != pExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode2 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node2, triangleID));
				}
				else if (primitiveType & NODE_EDGE) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node2, elem_edges1[triangleID][lid]));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node2, elem1[triangleID][lid]));
				}
				else if (!primitiveType) {
					nopenetr2 = true;
				}
			}
			else {
				nopenetr2 = true;
			}
		}

		if (nopenetr0 && nopenetr1 && nopenetr2) {
			triangle_node_missed_.push_back(std::pair<unsigned, unsigned>(contacted_triangles_[i].first, contacted_triangles_[i].second));
		}


	} //for


	  // third, filt the triangle_node_ contact
	for (auto &i : tnid) {
		//////////////////////////////////////////////////////////////
		// get number of nodes_ and nodes_ in triangle /////////////////
		Node0 = elem1[contacted_triangles_[i].second][0];
		Node1 = elem1[contacted_triangles_[i].second][1];
		Node2 = elem1[contacted_triangles_[i].second][2];

		isduplicated0 = isduplicated1 = isduplicated2 = false;
		if (FindPairNodeSecond(triangle_node_, Node0)
			|| FindPairNodeSecond(edge_node_, Node0)
			|| FindPairNodeSecond(node_node_, Node0)) isduplicated0 = true;

		if (FindPairNodeSecond(triangle_node_, Node1)
			|| FindPairNodeSecond(edge_node_, Node1)
			|| FindPairNodeSecond(node_node_, Node1)) isduplicated1 = true;

		if (FindPairNodeSecond(triangle_node_, Node2)
			|| FindPairNodeSecond(edge_node_, Node2)
			|| FindPairNodeSecond(node_node_, Node2)) isduplicated2 = true;

		if (isduplicated0 && isduplicated1 && isduplicated2) continue;

		triangleID = contacted_triangles_[i].first;
		n0 = elem0[triangleID][0];
		n1 = elem0[triangleID][1];
		n2 = elem0[triangleID][2];
		///////////////////////////////////////////////////////////////
		// get position of contact nodes_ //////////////////////////////

		rnode0 = triangle_mesh1_->GetRnode(Node0);
		rnode1 = triangle_mesh1_->GetRnode(Node1);
		rnode2 = triangle_mesh1_->GetRnode(Node2);
		rtri0 = triangle_mesh0_->GetRnode(n0);
		rtri1 = triangle_mesh0_->GetRnode(n1);
		rtri2 = triangle_mesh0_->GetRnode(n2);
		///////////////////////////////////////////////////////////////
		// get 3 nodes_ distances to the triangle surface. /////////////
		GetDistanceNodeTriangle(rnode0, rnode1, rnode2, rtri0, rtri1, rtri2, penetration1, surf_direction0_);
		///////////////////////////////////////////////////////////////
		// judge the pair if it is the contacted pair.
		nopenetr0 = nopenetr1 = nopenetr2 = false;

		if (!isduplicated0) {
			if (penetration1(0) < 0.0) {
				//if (std::find(nExtNodes.begin(), nExtNodes.end(), Node0) != nExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode0 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node0));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node0));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node0));
				}
				else if (!primitiveType) {
					nopenetr0 = true;
				}
			}
			else {
				nopenetr0 = true;
			}
		}

		if (!isduplicated1) {
			if (penetration1(1) < 0.0) {
				//if (std::find(nExtNodes.begin(), nExtNodes.end(), Node1) != nExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode1 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node1));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node1));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node1));
				}
				else if (!primitiveType) {
					nopenetr1 = true;
				}
			}
			else {
				nopenetr1 = true;
			}
		}

		if (!isduplicated2) {
			if (penetration1(2) < 0.0) {
				//if (std::find(nExtNodes.begin(), nExtNodes.end(), Node2) != nExtNodes.end()) continue;

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode2 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node2));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node2));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node2));
				}
				else if (!primitiveType) {
					nopenetr2 = true;
				}
			}
			else {
				nopenetr2 = true;
			}
		}

		if (nopenetr0 && nopenetr1 && nopenetr2) {
			node_triangle_missed_.push_back(std::pair<unsigned, unsigned>(contacted_triangles_[i].first, contacted_triangles_[i].second));
		}

	}// for

	for (auto &p : triangle_node_missed_) {
		// get number of nodes_ and nodes_ in triangle /////////////////
		Node0 = elem1[p.second][0];
		Node1 = elem1[p.second][1];
		Node2 = elem1[p.second][2];

		isduplicated0 = isduplicated1 = isduplicated2 = false;
		if (FindPairNodeSecond(triangle_node_, Node0)
			|| FindPairNodeSecond(edge_node_, Node0)
			|| FindPairNodeSecond(node_node_, Node0)) isduplicated0 = true;

		if (FindPairNodeSecond(triangle_node_, Node1)
			|| FindPairNodeSecond(edge_node_, Node1)
			|| FindPairNodeSecond(node_node_, Node1)) isduplicated1 = true;

		if (FindPairNodeSecond(triangle_node_, Node2)
			|| FindPairNodeSecond(edge_node_, Node2)
			|| FindPairNodeSecond(node_node_, Node2)) isduplicated2 = true;

		if (isduplicated0 && isduplicated1 && isduplicated2) continue;

		triangleID = p.first;
		n0 = elem0[triangleID][0];
		n1 = elem0[triangleID][1];
		n2 = elem0[triangleID][2];

		// get position of contact nodes_ //////////////////////////////

		rnode0 = triangle_mesh1_->GetRnode(Node0);
		rnode1 = triangle_mesh1_->GetRnode(Node1);
		rnode2 = triangle_mesh1_->GetRnode(Node2);
		rtri0 = triangle_mesh0_->GetRnode(n0);
		rtri1 = triangle_mesh0_->GetRnode(n1);
		rtri2 = triangle_mesh0_->GetRnode(n2);
		///////////////////////////////////////////////////////////////
		// get 3 nodes_ distances to the triangle surface. /////////////

		GetDistanceNodeTriangle(rnode0, rnode1, rnode2, rtri0, rtri1, rtri2, penetration1, surf_direction0_);
		///////////////////////////////////////////////////////////////
		// judge the pair if it is the contacted pair.

		if (!isduplicated0) {
			if (penetration1(0) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode0 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node0));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node0));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node0));
				}
			}
		}

		if (!isduplicated1) {
			if (penetration1(1) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode1 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node1));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node1));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node1));
				}
			}
		}

		if (!isduplicated2) {
			if (penetration1(2) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode2 - rtri2, false, primitiveType, lid);
				if (primitiveType & TRIANGLE_NODE) {
					triangle_node_.push_back(std::pair<unsigned, unsigned>(triangleID, Node2));
				}
				else if (primitiveType & EDGE_NODE) {
					edge_node_.push_back(std::pair<unsigned, unsigned>(elem_edges0[triangleID][lid], Node2));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(elem0[triangleID][lid], Node2));
				}
			}
		}

	} // fro triangle_node_missed_

	  //
	for (auto &p : node_triangle_missed_) {
		// get number of nodes_ and nodes_ in triangle /////////////////
		Node0 = elem0[p.first][0];
		Node1 = elem0[p.first][1];
		Node2 = elem0[p.first][2];

		isduplicated0 = isduplicated1 = isduplicated2 = false;
		if (FindPairNodeFirst(node_triangle_, Node0)
			|| FindPairNodeFirst(node_edge_, Node0)
			|| FindPairNodeFirst(node_node_, Node0)) isduplicated0 = true;

		if (FindPairNodeFirst(node_triangle_, Node1)
			|| FindPairNodeFirst(node_edge_, Node1)
			|| FindPairNodeFirst(node_node_, Node1)) isduplicated1 = true;

		if (FindPairNodeFirst(node_triangle_, Node2)
			|| FindPairNodeFirst(node_edge_, Node2)
			|| FindPairNodeFirst(node_node_, Node2)) isduplicated2 = true;

		if (isduplicated0 && isduplicated1 && isduplicated2) continue;

		triangleID = p.second;
		n0 = elem1[triangleID][0];
		n1 = elem1[triangleID][1];
		n2 = elem1[triangleID][2];

		///////////////////////////////////////////////////////////////
		// get position of contact nodes_ //////////////////////////////

		rnode0 = triangle_mesh0_->GetRnode(Node0);
		rnode1 = triangle_mesh0_->GetRnode(Node1);
		rnode2 = triangle_mesh0_->GetRnode(Node2);
		rtri0 = triangle_mesh1_->GetRnode(n0);
		rtri1 = triangle_mesh1_->GetRnode(n1);
		rtri2 = triangle_mesh1_->GetRnode(n2);
		///////////////////////////////////////////////////////////////
		// get 3 nodes_ distances to the triangle surface. /////////////

		GetDistanceNodeTriangle(rnode0, rnode1, rnode2, rtri0, rtri1, rtri2, penetration1, surf_direction1_);
		///////////////////////////////////////////////////////////////
		// judge the pair if it is the contacted pair.
		nopenetr0 = nopenetr1 = nopenetr2 = false;

		if (!isduplicated0) {
			if (penetration1(0) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode0 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node0, triangleID));
				}
				else if ((primitiveType & NODE_EDGE)) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node0, elem_edges1[triangleID][lid]));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node0, elem1[triangleID][lid]));
				}
			}
		}

		if (!isduplicated1) {
			if (penetration1(1) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode1 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node1, triangleID));
				}
				else if (primitiveType & NODE_EDGE) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node1, elem_edges1[triangleID][lid]));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node1, elem1[triangleID][lid]));
				}
			}
		}

		if (!isduplicated2) {
			if (penetration1(2) < 0.0) {

				AcceptNodeTrianglebyXi(rtri0 - rtri2, rtri1 - rtri2, rnode2 - rtri2, true, primitiveType, lid);
				if (primitiveType & NODE_TRIANGLE) {
					node_triangle_.push_back(std::pair<unsigned, unsigned>(Node2, triangleID));
				}
				else if (primitiveType & NODE_EDGE) {
					node_edge_.push_back(std::pair<unsigned, unsigned>(Node2, elem_edges1[triangleID][lid]));
				}
				else if (primitiveType & NODE_NODE) {
					node_node_.push_back(std::pair<unsigned, unsigned>(Node2, elem1[triangleID][lid]));
				}
			}
		}

	} // for node_triangle_missed_

}

BOOL ContactSensor::CheckContact()
{

  opcodewrapper_.GetModel(0)->Refit();
  opcodewrapper_.GetModel(1)->Refit();
  
	bool check_done = GetCollider()->Collide(*GetCache(), NULL, NULL);
	assert(check_done);

	contact_found_ = false;
	if (GetCollider()->GetContactStatus()) {
    contact_found_ = RestorePairs();
		if (contact_found_) {
      RestorePrimitives();
		}
	}
	if (!contact_found_) {
		node_triangle_.clear();
		triangle_node_.clear();
		edge_edge_.clear();

		node_node_.clear();
		node_edge_.clear();
		edge_node_.clear();
	}

	return contact_found_;
}


}