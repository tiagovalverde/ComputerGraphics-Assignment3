#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include <gl/glut.h>

#include "QuadMesh.h"

const int minMeshSize = 1;

QuadMesh NewQuadMesh(int maxMeshSize)
{
	QuadMesh qm;        // The new quad mesh to be returned
	qm.numVertices = 0;
	qm.vertices = NULL;
	qm.numQuads = 0;
	qm.quads = NULL;
	qm.numFacesDrawn = 0;

	qm.maxMeshSize = maxMeshSize < minMeshSize ? minMeshSize : maxMeshSize;
	CreateMemoryQM(&qm);

	// Set up default material used for the mesh
	qm.mat_ambient[0] = 0.0;
	qm.mat_ambient[1] = 0.0;
	qm.mat_ambient[2] = 0.0;
	qm.mat_ambient[3] = 1.0;
	qm.mat_specular[0] = 0.0;
	qm.mat_specular[1] = 0.0;
	qm.mat_specular[2] = 0.0;
	qm.mat_specular[3] = 1.0;
	qm.mat_diffuse[0] = 0.75;
	qm.mat_diffuse[1] = 0.5;
	qm.mat_diffuse[2] = 0.0;
	qm.mat_diffuse[3] = 1.0;
	qm.mat_shininess[0] = 0.0;

	return qm;
}

void SetMaterialQM(QuadMesh* qm, Vector3D ambient, Vector3D diffuse, Vector3D specular, double shininess)
{
	qm->mat_ambient[0] = ambient.x;
	qm->mat_ambient[1] = ambient.y;
	qm->mat_ambient[2] = ambient.z;
	qm->mat_ambient[3] = 1.0;
	qm->mat_specular[0] = specular.x;
	qm->mat_specular[1] = specular.y;
	qm->mat_specular[2] = specular.z;
	qm->mat_specular[3] = 1.0;
	qm->mat_diffuse[0] = diffuse.x;
	qm->mat_diffuse[1] = diffuse.y;
	qm->mat_diffuse[2] = diffuse.z;
	qm->mat_diffuse[3] = 1.0;
	qm->mat_shininess[0] = (float)shininess;
}

// Allocate dynamic arrays.
bool CreateMemoryQM(QuadMesh* qm)
{
	const int maxVertices = (qm->maxMeshSize + 1) * (qm->maxMeshSize + 1);
	qm->vertices = malloc(sizeof(MeshVertex) * maxVertices);
	if (qm->vertices == NULL)
	{
		return false;
	}

	const int maxQuads = qm->maxMeshSize * qm->maxMeshSize;
	qm->quads = malloc(sizeof(MeshQuad) * maxQuads);
	if (qm->quads == NULL)
	{
		return false;
	}

	return true;
}


// Fills the array of vertices and the array of quads.
bool InitMeshQM(QuadMesh* qm, int meshSize, Vector3D origin, double meshLength, double meshWidth, Vector3D dir1, Vector3D dir2)
{
	Vector3D o;
	double sf1, sf2;

	Vector3D v1, v2;

	v1.x = dir1.x;
	v1.y = dir1.y;
	v1.z = dir1.z;

	sf1 = meshLength / meshSize;
	ScalarMul(&v1, (float)sf1, &v1);

	v2.x = dir2.x;
	v2.y = dir2.y;
	v2.z = dir2.z;
	sf2 = meshWidth / meshSize;
	ScalarMul(&v2, (float)sf2, &v2);

	Vector3D meshpt;

	// Build Vertices
	qm->numVertices = (meshSize + 1) * (meshSize + 1);
	int currentVertex = 0;

	// Starts at front left corner of mesh 
	Set(&o, origin.x, origin.y, origin.z);

	for (int i = 0; i < meshSize + 1; i++)
	{
		for (int j = 0; j < meshSize + 1; j++)
		{
			// compute vertex position along mesh row (along x direction)
			meshpt.x = o.x + j * v1.x;
			meshpt.y = o.y + j * v1.y;
			meshpt.z = o.z + j * v1.z;

			Set(&qm->vertices[currentVertex].position, meshpt.x, meshpt.y, meshpt.z);
			currentVertex++;
		}
		// go to next row in mesh (negative z direction)
		Add(&o, &v2, &o);
	}

	// Build Quad Polygons
	qm->numQuads = (meshSize) * (meshSize);
	int currentQuad = 0;

	for (int j = 0; j < meshSize; j++)
	{
		for (int k = 0; k < meshSize; k++)
		{
			// Counterclockwise order
			qm->quads[currentQuad].vertices[0] = &qm->vertices[j * (meshSize + 1) + k];
			qm->quads[currentQuad].vertices[1] = &qm->vertices[j * (meshSize + 1) + k + 1];
			qm->quads[currentQuad].vertices[2] = &qm->vertices[(j + 1) * (meshSize + 1) + k + 1];
			qm->quads[currentQuad].vertices[3] = &qm->vertices[(j + 1) * (meshSize + 1) + k];
			currentQuad++;
		}
	}

	ComputeNormalsQM(qm);

	return true;
}

// Draw the mesh by drawing all quads.
void DrawMeshQM(QuadMesh* qm, int meshSize)
{
	int currentQuad = 0;

	glMaterialfv(GL_FRONT, GL_AMBIENT, qm->mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, qm->mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, qm->mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, qm->mat_shininess);

	for (int j = 0; j < meshSize; j++)
	{
		for (int k = 0; k < meshSize; k++)
		{
			glBegin(GL_QUADS);

			glNormal3f(qm->quads[currentQuad].vertices[0]->normal.x,
				qm->quads[currentQuad].vertices[0]->normal.y,
				qm->quads[currentQuad].vertices[0]->normal.z);
			glVertex3f(qm->quads[currentQuad].vertices[0]->position.x,
				qm->quads[currentQuad].vertices[0]->position.y,
				qm->quads[currentQuad].vertices[0]->position.z);

			glNormal3f(qm->quads[currentQuad].vertices[1]->normal.x,
				qm->quads[currentQuad].vertices[1]->normal.y,
				qm->quads[currentQuad].vertices[1]->normal.z);
			glVertex3f(qm->quads[currentQuad].vertices[1]->position.x,
				qm->quads[currentQuad].vertices[1]->position.y,
				qm->quads[currentQuad].vertices[1]->position.z);

			glNormal3f(qm->quads[currentQuad].vertices[2]->normal.x,
				qm->quads[currentQuad].vertices[2]->normal.y,
				qm->quads[currentQuad].vertices[2]->normal.z);
			glVertex3f(qm->quads[currentQuad].vertices[2]->position.x,
				qm->quads[currentQuad].vertices[2]->position.y,
				qm->quads[currentQuad].vertices[2]->position.z);

			glNormal3f(qm->quads[currentQuad].vertices[3]->normal.x,
				qm->quads[currentQuad].vertices[3]->normal.y,
				qm->quads[currentQuad].vertices[3]->normal.z);
			glVertex3f(qm->quads[currentQuad].vertices[3]->position.x,
				qm->quads[currentQuad].vertices[3]->position.y,
				qm->quads[currentQuad].vertices[3]->position.z);

			glEnd();
			currentQuad++;
		}
	}
}

// Deallocate dynamic arrays.
void FreeMemoryQM(QuadMesh* qm)
{
	if (qm->vertices != NULL)
		free(qm->vertices);
	qm->vertices = NULL;
	qm->numVertices = 0;

	if (qm->quads != NULL)
		free(qm->quads);
	qm->quads = NULL;
	qm->numQuads = 0;
}

// Use cross-products to compute the normal vector at each vertex
void ComputeNormalsQM(QuadMesh* qm)
{
	int currentQuad = 0;

	for (int j = 0; j < qm->maxMeshSize; j++)
	{
		for (int k = 0; k < qm->maxMeshSize; k++)
		{
			Vector3D n0, n1, n2, n3;
			Vector3D e0, e1, e2, e3;

			for (int i = 0; i < 4; i++)
			{
				LoadZero(&qm->quads[currentQuad].vertices[i]->normal);
			}

			Subtract(&qm->quads[currentQuad].vertices[1]->position, &qm->quads[currentQuad].vertices[0]->position, &e0);
			Subtract(&qm->quads[currentQuad].vertices[2]->position, &qm->quads[currentQuad].vertices[1]->position, &e1);
			Subtract(&qm->quads[currentQuad].vertices[3]->position, &qm->quads[currentQuad].vertices[2]->position, &e2);
			Subtract(&qm->quads[currentQuad].vertices[0]->position, &qm->quads[currentQuad].vertices[3]->position, &e3);
			Normalize(&e0);
			Normalize(&e1);
			Normalize(&e2);
			Normalize(&e3);

			Vector3D w;    // Working vector;

			Negate(&e3, &w);
			CrossProduct(&e0, &w, &n0);
			Normalize(&n0);
			Add(&qm->quads[currentQuad].vertices[0]->normal, &n0, &qm->quads[currentQuad].vertices[0]->normal);

			Negate(&e0, &w);
			CrossProduct(&e1, &w, &n1);
			Normalize(&n1);
			Add(&qm->quads[currentQuad].vertices[1]->normal, &n1, &qm->quads[currentQuad].vertices[1]->normal);

			Negate(&e1, &w);
			CrossProduct(&e2, &w, &n2);
			Normalize(&n2);
			Add(&qm->quads[currentQuad].vertices[2]->normal, &n2, &qm->quads[currentQuad].vertices[2]->normal);

			Negate(&e2, &w);
			CrossProduct(&e3, &w, &n3);
			Normalize(&n3);
			Add(&qm->quads[currentQuad].vertices[3]->normal, &n3, &qm->quads[currentQuad].vertices[3]->normal);

			for (int i = 0; i < 4; i++)
			{
				Normalize(&qm->quads[currentQuad].vertices[i]->normal);
			}

			currentQuad++;
		}
	}
}

// methods that renders the metaballs created in the quadmesh
void computeMetaballsIntoQuadMesh(struct Metaball* list, int numMetaballs, QuadMesh* qm) {

	for (int i = 0; i < qm->maxMeshSize + 1; i++) {
		for (int j = 0; j < qm->maxMeshSize + 1; j++) {

			int verticeIndex = i * (qm->maxMeshSize + 1) + j;
			Vector3D pos = qm->vertices[verticeIndex].position;
			qm->vertices[verticeIndex].position.y = 0; //resets y position to 0

			for (int k = 0; k <= numMetaballs; k++) {

				// add noise to the blobs to look more realistic
				float r = addRandomNoise();
				float distance = powf(list[k].position.x - pos.x, 2) +
					powf(list[k].position.z - pos.z, 2);

				// f(X,Z) = SUM_k ( b_x * e^(-a_k*r_k^2))
				float b_x = list[k].height * r;
				float a_k = -list[k].width / r;

				qm->vertices[verticeIndex].position.y += (b_x * (expf(a_k * distance)));
			}
		}
	}
	ComputeNormalsQM(qm);
}

float addRandomNoise() {
	// increasing will reduce noise
	// decreasing will increase noise
	int NOISE_DENOMINATOR = 15;
	return 1 + (float)(rand()) / (float)(RAND_MAX) / NOISE_DENOMINATOR;
}

Metaballs initializeMetaballs() {
	Metaballs staticMetaballs;

	
	staticMetaballs.list[0].position.x = -6.10036373;
	staticMetaballs.list[0].position.y = -0.0298475642;
	staticMetaballs.list[0].position.z = -4.35881853;
	staticMetaballs.list[0].width = 0.0999999642;
	staticMetaballs.list[0].height = 0.700000107;

	staticMetaballs.list[1].position.x = -5.73481846;
	staticMetaballs.list[1].position.y = 0.598784626;
	staticMetaballs.list[1].position.z = -3.12270236;
	staticMetaballs.list[1].width = 0.550000012;
	staticMetaballs.list[1].height = 0.700000107;

	staticMetaballs.list[2].position.x = -4.82809687;
	staticMetaballs.list[2].position.y = 0.162341326;
	staticMetaballs.list[2].position.z = -7.86406469;
	staticMetaballs.list[2].width = 0.550000012;
	staticMetaballs.list[2].height = 0.800000131;

	staticMetaballs.list[3].position.x = -3.05935907;
	staticMetaballs.list[3].position.y = 0.579775393;
	staticMetaballs.list[3].position.z = -4.06926680;
	staticMetaballs.list[3].width = 0.550000012;
	staticMetaballs.list[3].height = 0.700000107;

	staticMetaballs.list[4].position.x = 1.00884151;
	staticMetaballs.list[4].position.y = 0.624996722;
	staticMetaballs.list[4].position.z = -4.36985540;
	staticMetaballs.list[4].width = 0.550000012;
	staticMetaballs.list[4].height = 0.800000131;

	staticMetaballs.list[5].position.x = -2.58651757;
	staticMetaballs.list[5].position.y = -2.19709992;
	staticMetaballs.list[5].position.z = -16.8543148;
	staticMetaballs.list[5].width = 0.550000012;
	staticMetaballs.list[5].height = 0.800000131;

	staticMetaballs.list[6].position.x = -1.96070910;
	staticMetaballs.list[6].position.y = 0.0918585211;
	staticMetaballs.list[6].position.z = -6.27612925;
	staticMetaballs.list[6].width = 0.299999952;
	staticMetaballs.list[6].height = 2.10000014;

	staticMetaballs.list[7].position.x = 1.25525820;
	staticMetaballs.list[7].position.y = -0.0248816386;
	staticMetaballs.list[7].position.z = -0.228124738;
	staticMetaballs.list[7].width = 0.0999999642;
	staticMetaballs.list[7].height = -1.00000012;

	staticMetaballs.list[8].position.x = 6.47222662;
	staticMetaballs.list[8].position.y = -0.0351287164;
	staticMetaballs.list[8].position.z = -5.50770903;
	staticMetaballs.list[8].width = 0.299999952;
	staticMetaballs.list[8].height = 1.70000017;

	staticMetaballs.list[9].position.x = 6.72157097;
	staticMetaballs.list[9].position.y = -0.0782028064;
	staticMetaballs.list[9].position.z = -0.870674133;
	staticMetaballs.list[9].width = 0.299999952;
	staticMetaballs.list[9].height = 1.50000012;

	staticMetaballs.list[10].position.x = 6.07216978;
	staticMetaballs.list[10].position.y = -0.0336616710;
	staticMetaballs.list[10].position.z = 4.34948826;
	staticMetaballs.list[10].width = 0.299999952;
	staticMetaballs.list[10].height = 0.399999917;

	staticMetaballs.list[11].position.x = 6.48080206;
	staticMetaballs.list[11].position.y = 0.174273953;
	staticMetaballs.list[11].position.z = 2.72683382;
	staticMetaballs.list[11].width = 0.299999952;
	staticMetaballs.list[11].height = 0.599999905;

	staticMetaballs.list[12].position.x = -1.53580260;
	staticMetaballs.list[12].position.y = -0.00855677575;
	staticMetaballs.list[12].position.z = 1.32467699;
	staticMetaballs.list[12].width = 0.299999952;
	staticMetaballs.list[12].height = 0.599999905;

	staticMetaballs.list[13].position.x = -2.35522842;
	staticMetaballs.list[13].position.y = 0.231828406;
	staticMetaballs.list[13].position.z = 2.21690130;
	staticMetaballs.list[13].width = 0.299999952;
	staticMetaballs.list[13].height = 0.599999905;

	staticMetaballs.list[14].position.x = -5.55137205;
	staticMetaballs.list[14].position.y = 0.0300020706;
	staticMetaballs.list[14].position.z = 0.993400216;
	staticMetaballs.list[14].width = 0.299999952;
	staticMetaballs.list[14].height = 1.60000014;

	staticMetaballs.list[15].position.x = -4.80224276;
	staticMetaballs.list[15].position.y = 0.0312795155;
	staticMetaballs.list[15].position.z = 4.59819365;
	staticMetaballs.list[15].width = 0.299999952;
	staticMetaballs.list[15].height = 1.60000014;

	staticMetaballs.list[16].position.x = 2.76241374;
	staticMetaballs.list[16].position.y = -0.0128028858;
	staticMetaballs.list[16].position.z = -6.90601778;
	staticMetaballs.list[16].width = 0.249999955;
	staticMetaballs.list[16].height = 2.49999976;

	staticMetaballs.currentIndex = 16;

	return staticMetaballs;
}