## Tricycle: A general contact detection tool 

### 1. Introduction
This is based on an open source code [OPCODE](http://www.codercorner.com/Opcode.htm), which is originally for computer graphics purpose.

The linear algebraic module [Eigen](http://eigen.tuxfamily.org/) is used.

To generate contact detection procedure for **multibody dynamics**, some modification is made.

    1) float precision was converted to double precision,
    2) redundant codes having nothing to do with multibody dynamics were removed,
    3) adding feature that extracts contact pairs and primitives for further processing with contact force model,
    4) carefully extracting cotnact pairs and primitives without any misjudgement.

### 2. Usage
A typical procedure can refer to test/test_shellcontact.cc, you can find the input file used in test code from [cloth-ball example](https://mega.nz/#!vkdXAKBC!IKyLXkbKBQBtf8y86kjdhoxfNZ9Hs0vWd7rS_1hIhAU).

The general routine is very simple as follows

1) generate two potential contacted meshes
```
sptr<TriangleMesh> mesh1 = New<TriangleMesh>(); // create triangle mesh objects
sptr<TriangleMesh> mesh2 = New<TriangleMesh>();
mesh->SetMeshFromPlainText("file"); // set mesh from txt file, or you can overload the function as you wish.
mesh->SetMesh(elem, node);          // or set mesh from hard code
```
where `sptr<Object>` is a smart pointer of `Object`.

2) generate contact sensor for two meshes
```
sptr<ContactSensor> contact_sensor = New<TriangleMesh>();  // create triangle mesh objects
contact_sensor->SetMeshes(mesh1, mesh2, -1, 1);            // import meshes to generate internal AABB tree
contact_sensor->Initialize();                              // initialize
```

3) update nodal position and run contact detection
```
mesh->SetDisplacement(disp);      // update nodal positions
mesh->UpdateSurfaceNormal();      // calcuate surface normal
contact_sensor->CheckContact();   // contact detection
contact_sensor->GetPrimitivesPairs(CONTACT_PAIR_TYPE);  // extract contact types
```

This project is a part of *msdtk*, a general purpose multibody system dynamics tool kit for general finite element, rigid body dynamics, robot dynamics and control, and any multibody dynamics application. Of course, the main frame is still in part-time developing.
