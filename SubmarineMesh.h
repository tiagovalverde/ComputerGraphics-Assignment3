#include <gl/glut.h>
#include <stdbool.h>

// Material properties
static GLfloat submarine_mat_ambient[] = { 0.4F, 0.2F, 0.0F, 1.0F };
static GLfloat submarine_mat_specular[] = { 0.1F, 0.1F, 0.0F, 1.0F };
static GLfloat submarine_mat_diffuse[] = { 0.9F, 0.5F, 0.0F, 1.0F };
static GLfloat submarine_mat_shininess[] = { 0.0F };

typedef struct SubmarineMesh
{
	GLfloat mat_ambient[4];
	GLfloat mat_specular[4];
	GLfloat mat_diffuse[4];
	GLfloat mat_shininess[1];
	// 
} SubmarineMesh;

SubmarineMesh newSubmarine();
void drawSubmarine(
	float x_pos, float y_pos,
	float z_pos, float y_rot,
	float propeller_x_rot
);

struct SubmarineProps {
	float x_translate_pos;
	float y_translate_pos;
	float z_translate_pos;
	// starting rotation angle
	float y_axis_rotation;
	// propeller
	float propeller_x_axis_rotation;// state of the propllers angle
	int propeller_timer; // defines how fast the propeller should translate
	bool isEngineOn; // engine state
} SubmarineProps;

void drawTorpedo(float x_pos, float y_pos, float z_pos, float radius, float x_pos_moved);
//submarinePlayerProps