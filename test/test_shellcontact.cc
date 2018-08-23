#include"contact_sensor.h"

using namespace tricycle;

template<typename T>
std::ostream & operator<<(std::ostream &ss, std::vector<T>c) {
  unsigned i;
  for (i = 0; i < c.size() - 1; ++i)
    ss << c[i] << ", ";
  ss << c[i];
  return ss;
}

void WriteVector(std::ofstream &outfile, const std::vector<unsigned>& matrix) {
  outfile.write((char*)matrix.data(), sizeof(*matrix.data())*matrix.size());
};

void RetrievePairs(std::vector<std::pair<unsigned, unsigned>>& pairs, std::vector<unsigned> &outpair_vect) {
  unsigned pairNum = (unsigned)pairs.size();
  outpair_vect.push_back(pairNum);
  for (auto &p : pairs) {
    outpair_vect.push_back(p.first);
    outpair_vect.push_back(p.second);
  }
}

void WritePairs(const sptr<ContactSensor> &cs, std::ofstream &outfile) {

  std::vector<unsigned> outcontact0;
  RetrievePairs(cs->GetPrimitivesPairs(NODE_TRIANGLE), outcontact0);
  //std::cout << outcontact0 << std::endl;
  WriteVector(outfile, outcontact0);

  std::vector<unsigned> outcontact1;
  RetrievePairs(cs->GetPrimitivesPairs(TRIANGLE_NODE), outcontact1);
  //std::cout << outcontact1 << std::endl;
  WriteVector(outfile, outcontact1);

  std::vector<unsigned> outcontact2;
  RetrievePairs(cs->GetPrimitivesPairs(EDGE_EDGE), outcontact2);
  //std::cout << outcontact2 << std::endl;
  WriteVector(outfile, outcontact2);

  std::vector<unsigned> outcontact3;
  RetrievePairs(cs->GetPrimitivesPairs(NODE_EDGE), outcontact3);
  //std::cout << outcontact3 << std::endl;
  WriteVector(outfile, outcontact3);

  std::vector<unsigned> outcontact4;
  RetrievePairs(cs->GetPrimitivesPairs(EDGE_NODE), outcontact4);
  //std::cout << outcontact4 << std::endl;
  WriteVector(outfile, outcontact4);

  std::vector<unsigned> outcontact5;
  RetrievePairs(cs->GetPrimitivesPairs(NODE_NODE), outcontact5);
  //std::cout << outcontact5 << std::endl;
  WriteVector(outfile, outcontact5);

  //WriteVector(outfile, outcontact1);
}

int main() {

  sptr<TriangleMesh> cloth = New<TriangleMesh>();
  sptr<TriangleMesh> ball = New<TriangleMesh>();

  cloth->SetMeshFromPlainText("test_tricycle/cloth_mesh.txt");
  ball->SetMeshFromPlainText("test_tricycle/ball_mesh.txt");

  sptr<ContactSensor> cs = New<ContactSensor>();

  cs->SetMeshes(cloth, ball, -1, 1);
  cs->Initialize();

  std::ifstream displacement_file;
  std::string dispfile = "test_tricycle/out_clothhemiballfine2_d.dat";
  displacement_file.open(dispfile, std::ios::in | std::ios::binary);
  if (!displacement_file.is_open()) {
    std::cout << "displacement file error" << std::endl;
    return -1;
  }

  std::ofstream outpair_file;
  std::string pairfile = dispfile.substr(0, dispfile.size() - 4) + "_c.dat";
  outpair_file.open(pairfile, std::ios::out | std::ios::binary);

  WriteVector(outpair_file, { 1, 0, 1 });

  unsigned dofs = (cloth->num_node_ + ball->num_node_) * 3;
  double current_time;
  Eigen::MatrixXd datavec(3, dofs / 3);

  while (true) {
    displacement_file.read((char*)&current_time, sizeof(double));
    displacement_file.read((char*)datavec.data(), sizeof(double)* dofs);
    if (displacement_file.fail()) break;

    cloth->SetDisplacement(datavec.leftCols(cloth->num_node_));
    ball->SetDisplacement(datavec.rightCols(ball->num_node_));

    cloth->UpdateSurfaceNormal();
    ball->UpdateSurfaceNormal();

    cs->CheckContact();

    printf("Current Time: %10.6f; contact pair number: %d.\n", current_time, cs->GetPairNum());

    WritePairs(cs, outpair_file);
  }

  displacement_file.close();
  outpair_file.close();

  std::cout << std::endl;
  std::cout << "Finish searching contact, check the result file:\n" << pairfile << std::endl;
  return 0;
}

