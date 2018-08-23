// detail contact information
#define NODE_TRIANGLE 0x0001
#define TRIANGLE_NODE 0x0002
#define EDGE_EDGE	  0x0004

//! if OPC_TRITRI_EPSILON_TEST is true then we do a check (if |dv|<EPSILON then dv=0.0;) else no check is done (which is less robust, but faster)
#define LOCAL_EPSILON 1e-15

//! sort so that a<=b
#define SORT(a,b)			\
	if(a>b)					\
	{						\
		const double c=a;	\
		a=b;				\
		b=c;				\
	}

//! Edge to edge test based on Franlin Antonio's gem: "Faster Line Segment Intersection", in Graphics Gems III, pp. 199-202
#define EDGE_EDGE_TEST(V0, U0, U1)						\
	Bx = U0[i0] - U1[i0];								\
	By = U0[i1] - U1[i1];								\
	Cx = V0[i0] - U0[i0];								\
	Cy = V0[i1] - U0[i1];								\
	f  = Ay*Bx - Ax*By;									\
	d  = By*Cx - Bx*Cy;									\
	if((f>0.0 && d>=0.0 && d<=f) || (f<0.0 && d<=0.0 && d>=f))	\
	{													\
		const double e=Ax*Cy - Ay*Cx;					\
		if(f>0.0)										\
		{												\
			if(e>=0.0 && e<=f) return TRUE;			\
		}												\
		else											\
		{												\
			if(e<=0.0 && e>=f) return TRUE;			\
		}												\
	}

//! TO BE DOCUMENTED
#define EDGE_AGAINST_TRI_EDGES(V0, V1, U0, U1, U2)		\
{														\
	double Bx,By,Cx,Cy,d,f;								\
	const double Ax = V1[i0] - V0[i0];					\
	const double Ay = V1[i1] - V0[i1];					\
	/* test edge U0,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U0, U1);							\
	/* test edge U1,U2 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U1, U2);							\
	/* test edge U2,U1 against V0,V1 */					\
	EDGE_EDGE_TEST(V0, U2, U0);							\
}

//! TO BE DOCUMENTED
#define POINT_IN_TRI(V0, U0, U1, U2)					\
{														\
	/* is T1 completly inside T2? */					\
	/* check if V0 is inside tri(U0,U1,U2) */			\
	double a  = U1[i1] - U0[i1];							\
	double b  = -(U1[i0] - U0[i0]);						\
	double c  = -a*U0[i0] - b*U0[i1];					\
	double d0 = a*V0[i0] + b*V0[i1] + c;					\
														\
	a  = U2[i1] - U1[i1];								\
	b  = -(U2[i0] - U1[i0]);							\
	c  = -a*U1[i0] - b*U1[i1];							\
	const double d1 = a*V0[i0] + b*V0[i1] + c;			\
														\
	a  = U0[i1] - U2[i1];								\
	b  = -(U0[i0] - U2[i0]);							\
	c  = -a*U2[i0] - b*U2[i1];							\
	const double d2 = a*V0[i0] + b*V0[i1] + c;			\
	if(d0*d1>0.0)										\
	{													\
		if(d0*d2>0.0) return TRUE;						\
	}													\
}

//! TO BE DOCUMENTED
BOOL CoplanarTriTri(const IceMaths::Point& n, const IceMaths::Point& v0, const IceMaths::Point& v1, const IceMaths::Point& v2, const IceMaths::Point& u0, const IceMaths::Point& u1, const IceMaths::Point& u2)
{
	double A[3];
	short i0,i1;
	/* first project onto an axis-aligned plane, that maximizes the area */
	/* of the triangles, compute indices: i0,i1. */
	A[0] = fabs(n[0]);
	A[1] = fabs(n[1]);
	A[2] = fabs(n[2]);
	if(A[0]>A[1])
	{
		if(A[0]>A[2])
		{
			i0=1;      /* A[0] is greatest */
			i1=2;
		}
		else
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
	}
	else   /* A[0]<=A[1] */
	{
		if(A[2]>A[1])
		{
			i0=0;      /* A[2] is greatest */
			i1=1;
		}
		else
		{
			i0=0;      /* A[1] is greatest */
			i1=2;
		}
	}

	/* test all edges of triangle 1 against the edges of triangle 2 */
	EDGE_AGAINST_TRI_EDGES(v0, v1, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v1, v2, u0, u1, u2);
	EDGE_AGAINST_TRI_EDGES(v2, v0, u0, u1, u2);

	/* finally, test if tri1 is totally contained in tri2 or vice versa */
	POINT_IN_TRI(v0, u0, u1, u2);
	POINT_IN_TRI(u0, v0, v1, v2);

	return FALSE;
}

//! TO BE DOCUMENTED
// node 0-1 == edge 0
// node 1-2 == edge 1
// node 2-0 == edge 2
#define NEWCOMPUTE_INTERVALS(VV0, VV1, VV2, D0, D1, D2, D0D1, D0D2, A, B, C, X0, X1, edge)	\
{																						\
	if(D0D1>0.0)																		\
	{																					\
		/* here we know that D0D2<=0.0 */												\
		/* that is D0, D1 are on the same side, D2 on the other or on the plane */		\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
		edge[0] = 2; edge[1] = 1;														\
	}																					\
	else if(D0D2>0.0)																	\
	{																					\
		/* here we know that d0d1<=0.0 */												\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
		edge[0] = 0; edge[1] = 1;														\
	}																					\
	else if(D1*D2>0.0 || abs(D0)>LOCAL_EPSILON)										\
	{																					\
		/* here we know that d0d1<=0.0 or that D0!=0.0 */								\
		A=VV0; B=(VV1 - VV0)*D0; C=(VV2 - VV0)*D0; X0=D0 - D1; X1=D0 - D2;				\
		edge[0] = 0; edge[1] = 2;														\
	}																					\
	else if(abs(D1)>LOCAL_EPSILON)													\
	{																					\
		A=VV1; B=(VV0 - VV1)*D1; C=(VV2 - VV1)*D1; X0=D1 - D0; X1=D1 - D2;				\
		edge[0] = 0; edge[1] = 1;														\
	}																					\
	else if(abs(D2)>LOCAL_EPSILON)													\
	{																					\
		A=VV2; B=(VV0 - VV2)*D2; C=(VV1 - VV2)*D2; X0=D2 - D0; X1=D2 - D1;				\
		edge[0] = 2; edge[1] = 1;														\
	}																					\
	else																				\
	{																					\
		/* triangles are coplanar */													\
		return FALSE;																	\
		/*return CoplanarTriTri(N1, V0, V1, V2, U0, U1, U2);*/							\
	}																					\
}

inline void swap(int i, int j, double *sv, int *ind)
{
	double c = sv[i];
	sv[i] = sv[j];
	sv[j] = c;

	int ic = ind[i];
	ind[i] = ind[j];
	ind[j] = ic;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Triangle/triangle intersection test routine,
 *	by Tomas Moller, 1997.
 *	See article "A Fast Triangle-Triangle Intersection Test",
 *	Journal of Graphics Tools, 2(2), 1997
 *
 *	Updated June 1999: removed the divisions -- a little faster now!
 *	Updated October 1999: added {} to CROSS and SUB macros 
 *
 *	int NoDivTriTriIsect(double V0[3],double V1[3],double V2[3],
 *                      double U0[3],double U1[3],double U2[3])
 *
 *	\param		V0		[in] triangle 0, vertex 0
 *	\param		V1		[in] triangle 0, vertex 1
 *	\param		V2		[in] triangle 0, vertex 2
 *	\param		U0		[in] triangle 1, vertex 0
 *	\param		U1		[in] triangle 1, vertex 1
 *	\param		U2		[in] triangle 1, vertex 2
 *	\return		true if triangles overlap
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
inline_ BOOL Opcode::AABBTreeCollider::TriTriOverlap(const IceMaths::Point& V0, const IceMaths::Point& V1, const IceMaths::Point& V2, const IceMaths::Point& U0, const IceMaths::Point& U1, const IceMaths::Point& U2, IceCore::contactStruct &contactarg)
{
	// Stats
	mNbPrimPrimTests++;

	// Compute plane equation of triangle(V0,V1,V2)
	IceMaths::Point E1 = V1 - V0;
	IceMaths::Point E2 = V2 - V0;
	const IceMaths::Point N1 = E1 ^ E2;
	const double d1 =-N1 | V0;
	// Plane equation 1: N1.X+d1=0

	// Put U0,U1,U2 into plane equation 1 to compute signed distances to the plane
	double du0 = (N1|U0) + d1;
	double du1 = (N1|U1) + d1;
	double du2 = (N1|U2) + d1;

	// Coplanarity robustness check
#ifdef OPC_TRITRI_EPSILON_TEST
	if(abs(du0)<LOCAL_EPSILON) du0 = 0.0;
	if(abs(du1)<LOCAL_EPSILON) du1 = 0.0;
	if(abs(du2)<LOCAL_EPSILON) du2 = 0.0;
#endif
	const double du0du1 = du0 * du1;
	const double du0du2 = du0 * du2;

	if(du0du1>0.0 && du0du2>0.0)	// same sign on all of them + not equal 0 ?
		return FALSE;				// no intersection occurs

	// Compute plane of triangle (U0,U1,U2)
	E1 = U1 - U0;
	E2 = U2 - U0;
	const IceMaths::Point N2 = E1 ^ E2;
	const double d2=-N2 | U0;
	// plane equation 2: N2.X+d2=0

	// put V0,V1,V2 into plane equation 2
	double dv0 = (N2|V0) + d2;
	double dv1 = (N2|V1) + d2;
	double dv2 = (N2|V2) + d2;

#ifdef OPC_TRITRI_EPSILON_TEST
	if(abs(dv0)<LOCAL_EPSILON) dv0 = 0.0;
	if(abs(dv1)<LOCAL_EPSILON) dv1 = 0.0;
	if(abs(dv2)<LOCAL_EPSILON) dv2 = 0.0;
#endif

	const double dv0dv1 = dv0 * dv1;
	const double dv0dv2 = dv0 * dv2;

	if(dv0dv1>0.0 && dv0dv2>0.0)	// same sign on all of them + not equal 0 ?
		return FALSE;				// no intersection occurs

	// Compute direction of intersection line
	const IceMaths::Point D = N1^N2;

	// Compute and index to the largest component of D
	double max=abs(D[0]);
	short index=0;
	double bb=abs(D[1]);
	double cc=abs(D[2]);
	if(bb>max) max=bb,index=1;
	if(cc>max) max=cc,index=2;

	// This is the simplified projection onto L
	const double vp0 = V0[index];
	const double vp1 = V1[index];
	const double vp2 = V2[index];

	const double up0 = U0[index];
	const double up1 = U1[index];
	const double up2 = U2[index];

	// edge - edge 
	int edge0[2];
	int edge1[2];
	// Compute interval for triangle 1
	double a,b,c,x0,x1;
	NEWCOMPUTE_INTERVALS(vp0,vp1,vp2,dv0,dv1,dv2,dv0dv1,dv0dv2,a,b,c,x0,x1, edge0);
	// two edges that associated with the interval

	// Compute interval for triangle 2
	double d,e,f,y0,y1;
	NEWCOMPUTE_INTERVALS(up0,up1,up2,du0,du1,du2,du0du1,du0du2,d,e,f,y0,y1, edge1);
	// two edges that associated with the interval

	int edge[4] = { edge0[0], edge0[1], edge1[0], edge1[1] };

	const double xx=x0*x1;
	const double yy=y0*y1;
	const double xxyy=xx*yy;

	double isect1[2], isect2[2];

	double tmp=a*xxyy;
	isect1[0]=tmp+b*x1*yy;
	isect1[1]=tmp+c*x0*yy;

	tmp=d*xxyy;
	isect2[0]=tmp+e*xx*y1;
	isect2[1]=tmp+f*xx*y0;

	// 
	double proj[4] = { isect1[0], isect1[1], isect2[0], isect2[1] };
	int projInd[4] = { 0,1,2,3 };
	
	SORT(isect1[0],isect1[1]);
	SORT(isect2[0],isect2[1]);

	if(isect1[1]<isect2[0] || isect2[1]<isect1[0]) return FALSE;

	////////////////////////////////////////////////////////////////////////////////////////////////
	////////////////////////////////////////////////////////////////////////////////////////////////
	//
	//more details of contact
	////////////////////////////////////////////////////////////////////////////////////////////////

	// tri1 penetrate tri0
	if (isect1[0]<isect2[0] && isect1[1]> isect2[1]) {

		contactarg.primitiveType = TRIANGLE_NODE;

#ifdef STATIC_CONTACT
		// judge which node of the tri1 is on the other side of tri0 plane
		if (du0du1 < 0.0 && du0du2 < 0.0) contactarg[1] = 0;
		else if (du0du1 < 0.0 && du0du2 > 0.0) contactarg[1] = 1;
		else if (du0du1 > 0.0 && du0du2 < 0.0) contactarg[1] = 2;
		//printf("dv0=%f, dv1=%f, dv2=%f, T_N contactArg[1] = %d\n", dv0, dv1, dv2, contactarg[1]);
#endif
		
		// the method above is not completed to judge which node contacts.
		// to judge it correctly, the previous distance should be considered.
		// here just extract the node that contacts possibly.
		//if ((du0 - du1)*(du2 - du0) >= 0.0) {
		//	contactarg[1] = 1;
		//	contactarg[2] = 2;
		//}
		//else if ((du1 - du0)*(du2 - du1) >= 0.0) {
		//	contactarg[1] = 0;
		//	contactarg[2] = 2;
		//}
		//else if ((du2 - du0)*(du1 - du2) >= 0.0) {
		//	contactarg[1] = 0;
		//	contactarg[2] = 1;
		//}
		//contactarg.penetrations[0] = du0;
		//contactarg.penetrations[1] = du1;
		//contactarg.penetrations[2] = du2;

		return TRUE;
	}
	// tri0 penetrate tr1
	if (isect2[0]<isect1[0] && isect2[1]> isect1[1]) {

		contactarg.primitiveType = NODE_TRIANGLE;

#ifdef STATIC_CONTACT
		if (dv0dv1 < 0.0 && dv0dv2 < 0.0) contactarg[1] = 0;
		else if (dv0dv1 < 0.0 && dv0dv2 > 0.0) contactarg[1] = 1;
		else if (dv0dv1 > 0.0 && dv0dv2 < 0.0) contactarg[1] = 2;
		//printf("dv0=%f, dv1=%f, dv2=%f, N_T contactArg[1] = %d\n", dv0, dv1, dv2, contactarg[1]);
#endif	
		
		//if ((dv0 - dv1)*(dv2 - dv0) >= 0.0) {
		//	contactarg[1] = 1;
		//	contactarg[2] = 2;
		//}
		//else if ((dv1 - dv0)*(dv2 - dv1) >= 0.0) {
		//	contactarg[1] = 0;
		//	contactarg[2] = 2;
		//}
		//else if ((dv2 - dv0)*(dv1 - dv2) >= 0.0) {
		//	contactarg[1] = 0;
		//	contactarg[2] = 1;
		//}
		//contactarg.penetrations[0] = dv0;
		//contactarg.penetrations[1] = dv1;
		//contactarg.penetrations[2] = dv2;

		return TRUE;
	}

	// edge penetrate edge
	if (isect2[0]<isect1[1] || isect2[1]> isect1[0]) {

		contactarg.primitiveType = EDGE_EDGE;

		for (int i = 4; i > 0; --i) {
			for (int j = 0; j < i - 1; ++j)
				if (proj[j] > proj[j + 1]) swap(j, j + 1, proj, projInd);
		}

		if (projInd[0] == 0 || projInd[0] == 1) {
			contactarg.edgePair[0] = edge[projInd[2]];
			contactarg.edgePair[1] = edge[projInd[1]];
		}
		else {
			contactarg.edgePair[0] = edge[projInd[1]];
			contactarg.edgePair[1] = edge[projInd[2]];
		}
		
		return TRUE;
	}

	return FALSE;
}