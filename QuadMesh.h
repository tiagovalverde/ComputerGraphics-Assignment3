#include <stdbool.h>
#include "Vector3D.h"

// Data structure for a vertex
typedef struct MeshVertex
{
	Vector3D position;
	Vector3D normal;
} MeshVertex;

// Data structure for a quad (4-sided polygon)
typedef struct MeshQuad
{
	// pointers to vertices of each quad in the array of vertices
	MeshVertex* vertices[4];
} MeshQuad;

typedef struct
{
	int maxMeshSize;
	float meshDim;

	int numVertices;
	MeshVertex* vertices;    // Dynamic array of all vertices

	int numQuads;
	MeshQuad* quads;         // Dynamic array of all quads

	int numFacesDrawn;

	GLfloat mat_ambient[4];
	GLfloat mat_specular[4];
	GLfloat mat_diffuse[4];
	GLfloat mat_shininess[1];
} QuadMesh;

// represents one metaball
struct Metaball {
	Vector3D position;
	float width, height;
} Metaball;

// represents list of metaballs
typedef struct  {
	int currentIndex;
	struct Metaball list[20];
} Metaballs;

QuadMesh NewQuadMesh(int maxMeshSize);
void SetMaterialQM(QuadMesh* qm, Vector3D ambient, Vector3D diffuse, Vector3D specular, double shininess);
bool CreateMemoryQM(QuadMesh* qm);
bool InitMeshQM(QuadMesh* qm, int meshSize, Vector3D origin, double meshLength, double meshWidth, Vector3D dir1, Vector3D dir2);
void DrawMeshQM(QuadMesh* qm, int meshSize);
void FreeMemoryQM(QuadMesh* qm);
void ComputeNormalsQM(QuadMesh* qm);

void computeMetaballsIntoQuadMesh(struct Metaball* metaballsList, int metaballsCount, QuadMesh* qm);
float addRandomNoise();
Metaballs initializeMetaballs();