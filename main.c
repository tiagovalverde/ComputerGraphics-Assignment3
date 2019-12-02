/*******************************************************************
		   Multi-Part Model Construction and Manipulation
********************************************************************/
#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gl/glut.h>
#include "Vector3D.h"
#include "QuadMesh.h"
#include "CubeMesh.h"
#include "SubmarineMesh.h"
#include <math.h>
#include <stdlib.h>

const int meshSize = 64;    // Default Mesh Size
const int vWidth = 650;     // Viewport width in pixels
const int vHeight = 500;    // Viewport height in pixels
static float PI = 3.14159265358979323846;

static int currentButton;
static unsigned char currentKey;

// Lighting/shading and material properties for submarine - upcoming lecture - just copy for now
// Light properties
static GLfloat light_position0[] = { -6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_position1[] = { 6.0F, 12.0F, 0.0F, 1.0F };
static GLfloat light_diffuse[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_specular[] = { 1.0, 1.0, 1.0, 1.0 };
static GLfloat light_ambient[] = { 0.2F, 0.2F, 0.2F, 1.0F };

// A quad mesh representing the ground
static QuadMesh groundMesh;
static Metaballs metaballs;
// A mesh representing the static object on the ground
static CubeMesh cubeMesh;
static CubeMesh cubeMesh2;
static CubeMesh cubeMesh3;
static CubeMeshProps cubeMeshProps;
static CubeMeshProps cubeMeshProps2;
static CubeMeshProps cubeMeshProps3;
/**
* A mesh representing the submarine
* Holds the instance of the sumbarine instance
*/

static SubmarineMesh submarineNPC;
struct SubmarineProps submarineNPCProps;

static SubmarineMesh submarineNPC2;
struct SubmarineProps submarineNPCProps2;

static SubmarineMesh submarinePlayer;
struct SubmarineProps submarinePlayerProps;

// Camera state
bool periscopeView = false; 
 
typedef struct ViewState {
	bool inPeriscopeView;
	float posEyeX, posEyeY, posEyeZ;
	float posToLookX, posToLookY, posToLookZ;
	float zoom;
} ViewState;
struct ViewState viewState;



// Structure defining a bounding box, currently unused
typedef struct BoundingBox {
    Vector3D min;
    Vector3D max;
} BoundingBox;

typedef struct Torpedo {
	float speed; 
	float radius;
	float t_x_pos, t_y_pos, t_z_pos;
	float y_rotation;
	bool fired;
} Torpedo;
struct Torpedo torpedo;
int counter = 0;
int npcs_destroyed = 0;

BoundingBox torpedoBB;
BoundingBox NPC_BB;
BoundingBox NPC2_BB;

// Prototypes for functions in this module
void initOpenGL(int w, int h);
void display(void);
void reshape(int w, int h);

void timerPropeller(int); //start engine
void mouse(int button, int state, int x, int y);
void mouseMotionHandler(int xMouse, int yMouse);
void keyboard(unsigned char key, int x, int y);
void functionKeys(int key, int x, int y);
Vector3D ScreenToWorld(int x, int y);

/*submarine functions that trigger its movement */
void setSubmarineMaterialProperties();
void moveUp();
void moveDown();
void rotateRight();
void rotateLeft();
void startPropeler();
void moveSubmarine(float direction);
void displayHelp();

// utils
double constrainAngle(double x);

// initialize state
void initStateVariables();
void updatePeriscopeView();
float toRadians(float rotation);
void fireTorpedo();
float degToRad();

// NPC
void timerPropellerNPC(int value);
void timerPropellerNPC2(int value);
void NPCMovingRoutine();
void NPCMovingRoutine2();
void outputScore();


int main(int argc, char** argv)
{
	// Initialize GLUT
	glutInit(&argc, argv); 
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);  
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); 
	glutInitWindowSize(vWidth, vHeight);
	glutInitWindowPosition(200, 30);
	glutCreateWindow("Assignment 3");

	// Initialize GL
	initOpenGL(vWidth, vHeight);

	// Register callbacks
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	glutMouseFunc(mouse);
	glutMotionFunc(mouseMotionHandler);
	glutKeyboardFunc(keyboard);

	glutSpecialFunc(functionKeys);

	glutTimerFunc(35, fireTorpedo, 0);

	timerPropellerNPC(1);
	NPCMovingRoutine();

	timerPropellerNPC2(1);
	NPCMovingRoutine2();

	// Start event loop, never returns
	glutMainLoop();

	return 0;
}


// Set up OpenGL. For viewport and projection setup see reshape(). */
void initOpenGL(int w, int h)
{
	// Set up and enable lighting
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient); 
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);  
	glLightfv(GL_LIGHT1, GL_AMBIENT, light_ambient);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, light_diffuse);
	glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular); 

	glLightfv(GL_LIGHT0, GL_POSITION, light_position0);
	glLightfv(GL_LIGHT1, GL_POSITION, light_position1);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	//glEnable(GL_LIGHT1);   // This light is currently off

	// Other OpenGL setup
	glEnable(GL_DEPTH_TEST);   // Remove hidded surfaces
	glShadeModel(GL_SMOOTH);   // Use smooth shading, makes boundaries between polygons harder to see 
	glClearColor(0.6F, 0.6F, 0.6F, 0.0F);  // Color and depth for glClear
	glClearDepth(1.0f);
	glEnable(GL_NORMALIZE);    // Renormalize normal vectors 
	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);   // Nicer perspective

	// Set up ground quad mesh
	Vector3D origin = NewVector3D(-8.0f, 0.0f, 8.0f);
	Vector3D dir1v = NewVector3D(1.0f, 0.0f, 0.0f);
	Vector3D dir2v = NewVector3D(0.0f, 0.0f, -1.0f);
	groundMesh = NewQuadMesh(meshSize);
	InitMeshQM(&groundMesh, meshSize, origin, 16.0, 16.0, dir1v, dir2v);

	Vector3D ambient = NewVector3D(0.0f, 0.05f, 0.0f);
	Vector3D diffuse = NewVector3D(0.4f, 0.8f, 0.4f);
	Vector3D specular = NewVector3D(0.04f, 0.04f, 0.04f);
	SetMaterialQM(&groundMesh, ambient, diffuse, specular, 0.2);

	// set initial properties
	initStateVariables();

	computeMetaballsIntoQuadMesh(
		metaballs.list,
		metaballs.currentIndex,
		&groundMesh);

	// setup static mesh
	// GROUND OBJECTS
	cubeMesh = newCube();
	cubeMesh2 = newCube();
	cubeMesh3 = newCube();

	// setup dynamic mesh
	// SUBMARINES
	
	submarineNPC = newSubmarine();
	submarineNPC2 = newSubmarine();
	submarinePlayer = newSubmarine();
	// Set up the bounding box of the scene
	// Currently unused. You could set up bounding boxes for your objects eventually.
	//Set(&BBox.min, -8.0f, 0.0, -8.0);
	//Set(&BBox.max, 8.0f, 6.0,  8.0);
}

// Helper Functions to set and transform submarine
void setSubmarineMaterialProperties() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, submarinePlayer.mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, submarinePlayer.mat_specular); 
	glMaterialfv(GL_FRONT, GL_DIFFUSE, submarinePlayer.mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, submarinePlayer.mat_shininess);  
}
void setSubmarineMaterialPropertiesNPC() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, submarineNPC.mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, submarineNPC.mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, submarineNPC.mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, submarineNPC.mat_shininess);
}

void setSubmarineMaterialPropertiesNPC2() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, submarineNPC2.mat_ambient);
	glMaterialfv(GL_FRONT, GL_SPECULAR, submarineNPC2.mat_specular);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, submarineNPC2.mat_diffuse);
	glMaterialfv(GL_FRONT, GL_SHININESS, submarineNPC2.mat_shininess);
}


// Callback, called whenever GLUT determines that the window should be redisplayed
// or glutPostRedisplay() has been called.
void display(void)
{
	//glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel(GL_SMOOTH);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();

	if (viewState.inPeriscopeView) {
		gluLookAt(
			viewState.posEyeX, viewState.posEyeY, viewState.posEyeZ,
			viewState.posToLookX, 
			viewState.posToLookY,
			viewState.posToLookZ,
			0.0, 1.0, 0.0);
	}
	else {
		// Set up the camera at position (0, 6, 22) looking at the origin, up along positive y axis
		gluLookAt(0.0, 6.0, 15.0, 0.0, 0.0, 0.0, 0.0, 11.0, 0.0);
	}
	// Draw submarine

	// Set submarine material properties
	
	setSubmarineMaterialPropertiesNPC();
	setSubmarineMaterialPropertiesNPC2();
	setSubmarineMaterialProperties();
	// SubmarineMesh function that redraws the submarine according to the:
	// translate coordinates x,y,z
	// submarine y rotation angle
	// propellers x rotation angle
	drawSubmarine(
		submarinePlayerProps.x_translate_pos, 
		submarinePlayerProps.y_translate_pos, 
		submarinePlayerProps.z_translate_pos, 
		submarinePlayerProps.y_axis_rotation, 
		submarinePlayerProps.propeller_x_axis_rotation);

	drawSubmarine(
		submarineNPCProps.x_translate_pos,
		submarineNPCProps.y_translate_pos,
		submarineNPCProps.z_translate_pos,
		submarineNPCProps.y_axis_rotation,
		submarineNPCProps.propeller_x_axis_rotation);

	drawSubmarine(
		submarineNPCProps2.x_translate_pos,
		submarineNPCProps2.y_translate_pos,
		submarineNPCProps2.z_translate_pos,
		submarineNPCProps2.y_axis_rotation,
		submarineNPCProps2.propeller_x_axis_rotation);

	// TEXT
	outputScore();

	if (counter > 0) {
		drawTorpedo(
			torpedo.t_x_pos, 
			torpedo.t_y_pos, 
			torpedo.t_z_pos, 
			torpedo.radius
		);

	}

	// Draw ground mesh
	DrawMeshQM(&groundMesh, meshSize);
	// Draw static mesh
	drawCube(&cubeMesh, &cubeMeshProps);
	drawCube(&cubeMesh2, &cubeMeshProps2);
	drawCube(&cubeMesh3, &cubeMeshProps3);

	glutSwapBuffers();   // Double buffering, swap buffers
}


// Callback, called at initialization and whenever user resizes the window.
void reshape(int w, int h)
{
	// Set up viewport, projection, then change to modelview matrix mode - 
	// display function will then set up camera and do modeling transforms.
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(60.0, (GLdouble)w / h, 0.2, 40.0);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	
}

// Callback, handles input from the keyboard, non-arrow keys
void keyboard(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 's':
		if (!submarinePlayerProps.isEngineOn) startPropeler();
		break;
	case 'f':
		if (submarinePlayerProps.isEngineOn) {
			moveSubmarine(+1);
		}
		break;
	case 'b':
		if (submarinePlayerProps.isEngineOn)moveSubmarine(-1);
		break;
	case 'p':
		viewState.inPeriscopeView = !viewState.inPeriscopeView;
		break;
	case 'o':
		viewState.zoom += 2.0;
		break;
	case 'i':
		viewState.zoom -= 2.0;
		break;
	case 'q':

		torpedo.t_x_pos = submarinePlayerProps.x_translate_pos;
		torpedo.t_y_pos = submarinePlayerProps.y_translate_pos;
		torpedo.t_z_pos = submarinePlayerProps.z_translate_pos;
		torpedo.y_rotation = submarinePlayerProps.y_axis_rotation;

		counter = 50;

		break;
	}
	
	glutPostRedisplay();   // Trigger a window redisplay
}



// Callback, handles input from the keyboard, function and arrow keys
void functionKeys(int key, int x, int y)
{
	// Help key
	if (key == GLUT_KEY_F1)
	{
		displayHelp();
	}
	else if (key == GLUT_KEY_DOWN) {
		if (submarinePlayerProps.isEngineOn) moveDown();
	}
	else if (key == GLUT_KEY_UP) {
		if (submarinePlayerProps.isEngineOn) moveUp();
	}
	else if (key == GLUT_KEY_RIGHT) {
		if (submarinePlayerProps.isEngineOn) rotateRight();
	}
	else if (key == GLUT_KEY_LEFT) {
		if (submarinePlayerProps.isEngineOn) rotateLeft();
	}
	glutPostRedisplay();   // Trigger a window redisplay
}


// Mouse button callback - use only if you want to 
void mouse(int button, int state, int x, int y)
{
	currentButton = button;
	switch (button)
	{
	case GLUT_LEFT_BUTTON:
		if (state == GLUT_DOWN)
		{
			;
		}
		break;
	case GLUT_RIGHT_BUTTON:
		if (state == GLUT_DOWN)
		{
			;
		}
		break;
	default:
		break;
	}

	glutPostRedisplay();   // Trigger a window redisplay
}


// Mouse motion callback - use only if you want to 
void mouseMotionHandler(int xMouse, int yMouse)
{
	if (currentButton == GLUT_LEFT_BUTTON)
	{
		;
	}

	glutPostRedisplay();   // Trigger a window redisplay
}


Vector3D ScreenToWorld(int x, int y)
{
	// you will need to finish this if you use the mouse
	return NewVector3D(0, 0, 0);
}


/********************************************
******** User Input Helper Functions ********
********************************************/

// Simply prints the instructions to the user in the terminal 
void displayHelp() {
	printf("======================================================\n");
	printf("===================== Help ===========================\n");
	printf("======================================================\n");
	printf("'F1':	display help again \n");
	printf("'s':	start the submarine engine \n");
	printf("'f':	move the submarine foward \n");
	printf("'b':	move the submarine backwards \n");
	printf("------------------------------------------------------\n");
	printf("'left'	arrow key:	rotate the submarine left \n");
	printf("'right'	arrow key:	rotate the submarine right \n");
	printf("'up'	arrow key:	move the submarine upwards \n");
	printf("'down'	arrow key:	move the submarine downwards \n");
	printf("------------------------------------------------------\n");
	printf("'p':	toggle between main view and periscope view \n");
	printf("'q':	fire torpedo \n");
	printf("======================================================\n");

}

// moves the submaring up on user input
// checks if submarine reached limits of the world ceiling
void moveUp() {
	if (submarinePlayerProps.y_translate_pos < 10.50) {
		submarinePlayerProps.y_translate_pos += 0.10;
		updatePeriscopeView();
	}
}

// moves the submaring down on user input
// checks if submarine reached limits of the world ground
void moveDown() {
	if (submarinePlayerProps.y_translate_pos > 0.60) {
		submarinePlayerProps.y_translate_pos -= 0.10;
		updatePeriscopeView();	
	}
}

// rotates the submarine right
void rotateRight() {
	submarinePlayerProps.y_axis_rotation -= 5.0;
	updatePeriscopeView();
}

// rotates the submarine left
void rotateLeft() {
	submarinePlayerProps.y_axis_rotation += 5.0;
	updatePeriscopeView();
}

/* triggers the sumarine to move fowards or backwards
*  float direction: +1 to move fowards, -1 to move backwards
*/
void moveSubmarine(float direction) {

	if (direction == 1) submarinePlayerProps.propeller_x_axis_rotation += 40.0;
	if (direction == -1) submarinePlayerProps.propeller_x_axis_rotation -= 40.0;


	float PI = 3.14159265358979323846;
	float CONSTANT_SPEED = 0.1; // submarine constant speed 
	float normalzed_angle = constrainAngle(submarinePlayerProps.y_axis_rotation); // set angle to range btw 0deg and 360deg
	float currentRadian = normalzed_angle * (PI / 180); // transform into angle to radian

	// calculate the amount of movement the submarine should take in the x and z coordinates
	// the direction variable is defining if the submarine moves fowards or backwards
	float x_total_move = (fabs(cos(currentRadian) * CONSTANT_SPEED)) * direction;
	float z_total_move = (fabs(sin(currentRadian) * CONSTANT_SPEED)) * direction;

	// 0 to 90 deg direction
	if (currentRadian > 0 && currentRadian < (PI / 2)) {
		submarinePlayerProps.x_translate_pos += x_total_move;
		submarinePlayerProps.z_translate_pos -= z_total_move;
	}
	if (currentRadian == 0) {
		submarinePlayerProps.x_translate_pos += x_total_move;
	}
	if (currentRadian == (PI / 2)) {
		submarinePlayerProps.z_translate_pos -= z_total_move;
	}
	// 90 to 180 deg direction
	if (currentRadian > (PI / 2) && currentRadian < PI) {
		submarinePlayerProps.x_translate_pos -= x_total_move;
		submarinePlayerProps.z_translate_pos -= z_total_move;
	}
	if (currentRadian == PI) {
		submarinePlayerProps.x_translate_pos += x_total_move;
	}
	// 180 to 270 deg direction
	if (currentRadian > PI&& currentRadian < ((3 * PI) / 2)) {
		submarinePlayerProps.x_translate_pos -= x_total_move;
		submarinePlayerProps.z_translate_pos += z_total_move;
	}
	if (currentRadian == ((3 * PI) / 2)) {
		submarinePlayerProps.z_translate_pos += z_total_move;
	}
	// 270 to 360 deg direction
	if (currentRadian > ((3 * PI) / 2) && currentRadian < (2 * PI)) {
		submarinePlayerProps.x_translate_pos += x_total_move;
		submarinePlayerProps.z_translate_pos += z_total_move;
	}
	if (currentRadian == (2 * PI)) {
		submarinePlayerProps.x_translate_pos += x_total_move;
	}

	// after x,y,z submarine updated, update persicope camera.
	updatePeriscopeView();
}


/* Checks if propller is already rotating.
*  If not, the timer function to rotate the propeller is called
*/
void startPropeler() {
	submarinePlayerProps.isEngineOn = !submarinePlayerProps.isEngineOn;
	glutTimerFunc(0, timerPropeller, 0);
}



/* util function assisting in calculating the movement of the submarine (fowards/backwards)
*  x: current y rotation angle of the submarine
*  return double: correspondent angle in 0 to 350 range
*/
double constrainAngle(double x) {
	x = fmod(x, 360);
	if (x < 0)
		x += 360;
	return x;
}

/*
* timer function that is triggered when the user
* presses the s keyboard to start the submarine propller
*/
void timerPropeller(int value) {
	submarinePlayerProps.propeller_x_axis_rotation += 4.0;
	glutPostRedisplay();
	glutTimerFunc(submarinePlayerProps.propeller_timer / 60, timerPropeller, 0);
}

void timerPropellerNPC(int value) {
	submarineNPCProps.propeller_x_axis_rotation += 4.0;
	glutPostRedisplay();
	glutTimerFunc(submarineNPCProps.propeller_timer / 60, timerPropellerNPC, 0);
}

void timerPropellerNPC2(int value) {
	submarineNPCProps2.propeller_x_axis_rotation += 4.0;
	glutPostRedisplay();
	glutTimerFunc(submarineNPCProps2.propeller_timer / 60, timerPropellerNPC2, 0);
}


/////////////////////////////////////////////////////////
// A3 Functions
/////////////////////////////////////////////////////////
void initStateVariables() {
	
	////////////////////
	// PLAYER SUBMARINE
	///////////////////
	submarinePlayerProps.x_translate_pos = 0.0;
	submarinePlayerProps.y_translate_pos = 2.0;
	submarinePlayerProps.z_translate_pos = 0.0;
	submarinePlayerProps.y_axis_rotation = 0.0;
	submarinePlayerProps.propeller_x_axis_rotation = 0.0;
	submarinePlayerProps.propeller_timer = 1000;
	submarinePlayerProps.isEngineOn = false;

	/////////////////////
	// PERISCOVE VIEW
	////////////////////
	float DegToRad = degToRad();

	viewState.inPeriscopeView = false;

	viewState.posEyeX = submarinePlayerProps.x_translate_pos + 0.5;
	viewState.posEyeY = submarinePlayerProps.y_translate_pos + 0.625;
	viewState.posEyeZ = submarinePlayerProps.z_translate_pos;
	
	viewState.posToLookX = submarinePlayerProps.x_translate_pos + 1.0;
	viewState.posToLookY = (submarinePlayerProps.y_translate_pos + 0.625);
	viewState.posToLookZ = submarinePlayerProps.z_translate_pos;

	viewState.zoom = 0.0;

	////////////////////
	// TORPEDO
	////////////////////
	torpedo.speed = 7;
	torpedo.radius = 0.2;
	torpedo.t_x_pos = 0.0;
	torpedo.t_y_pos = 0.0;
	torpedo.t_z_pos = 0.0;
	torpedo.y_rotation = 0.0;
	torpedo.fired  = false;

	////////////////////
	// NPC SUBMARINES
	///////////////////
	submarineNPCProps.x_translate_pos = -18.0;
	submarineNPCProps.y_translate_pos = 4.0;
	submarineNPCProps.z_translate_pos = -6.0;
	submarineNPCProps.y_axis_rotation = 0.0;
	submarineNPCProps.propeller_x_axis_rotation = 0.0;
	submarineNPCProps.propeller_timer = 1000;
	submarineNPCProps.isEngineOn = true;

	submarineNPCProps2.x_translate_pos = 6.0;
	submarineNPCProps2.y_translate_pos = 2.0;
	submarineNPCProps2.z_translate_pos = -3.0;
	submarineNPCProps2.y_axis_rotation = -90.0;
	submarineNPCProps2.propeller_x_axis_rotation = 0.0;
	submarineNPCProps2.propeller_timer = 1000;
	submarineNPCProps2.isEngineOn = true;
	submarineNPCProps2.NPC_movingStage = 0;
	///////////////////////////
	// METABALS ON GROUNDMESH
	///////////////////////////
	metaballs = initializeMetaballs();
	
	///////////////////////////
	// CubeMesh (Ground Objects
	///////////////////////////
	cubeMeshProps.transX = 4.0;
	cubeMeshProps.transY = 0.5;
	cubeMeshProps.transZ = 6.0;
	cubeMeshProps.rotY = 20.0;
	cubeMeshProps.rotZ = 10.0;
	cubeMeshProps.scaleX = 0.25;
	cubeMeshProps.scaleY = 0.5;
	cubeMeshProps.scaleZ = 0.25;

	cubeMeshProps2.transX = -2.0;
	cubeMeshProps2.transY = 0.3;
	cubeMeshProps2.transZ = 6.0;
	cubeMeshProps2.rotY = 20.0;
	cubeMeshProps2.rotZ = -10.0;
	cubeMeshProps2.scaleX = 0.5;
	cubeMeshProps2.scaleY = 0.25;
	cubeMeshProps2.scaleZ = 0.25;

	cubeMeshProps3.transX = 1.0;
	cubeMeshProps3.transY = -0.5;
	cubeMeshProps3.transZ = -2.0;
	cubeMeshProps3.rotY = 20.0;
	cubeMeshProps3.rotZ = -10.0;
	cubeMeshProps3.scaleX = 1.0;
	cubeMeshProps3.scaleY = 0.50;
	cubeMeshProps3.scaleZ = 0.25;
	
}

void updatePeriscopeView() {
	
	float DegToRad = degToRad();

	viewState.posEyeX = (submarinePlayerProps.x_translate_pos) + (1*(cos(submarinePlayerProps.y_axis_rotation * DegToRad)));
	viewState.posEyeY = submarinePlayerProps.y_translate_pos + 0.625;
	viewState.posEyeZ = (submarinePlayerProps.z_translate_pos) - (1*(sin(submarinePlayerProps.y_axis_rotation * DegToRad)));

	viewState.posToLookX = viewState.posEyeX + ((cos(submarinePlayerProps.y_axis_rotation * DegToRad)) * 5);
	viewState.posToLookY = viewState.posEyeY - 1;
	viewState.posToLookZ = viewState.posEyeZ - ((sin(submarinePlayerProps.y_axis_rotation * DegToRad)) * 5);

}

void fireTorpedo() {
	if(counter > 0) {
		torpedo.radius = 0.2;
		torpedo.t_x_pos +=
			(cosf((submarinePlayerProps.y_axis_rotation) * degToRad())) / (20 / 5) * torpedo.speed;
		torpedo.t_z_pos -=
			(sinf(submarinePlayerProps.y_axis_rotation * degToRad())) / (20 / 5) * torpedo.speed;

		if (
			torpedo.t_x_pos > NPC_BB.min.x &&
			torpedo.t_x_pos < NPC_BB.max.x &&
			torpedo.t_y_pos > NPC_BB.min.y &&
			torpedo.t_y_pos < NPC_BB.max.y &&
			torpedo.t_z_pos > NPC_BB.min.z &&
			torpedo.t_z_pos < NPC_BB.max.z
		) 
		{
			npcs_destroyed++;
			submarineNPCProps.x_translate_pos = -18.0;
			submarineNPCProps.y_translate_pos = 4.0;
			submarineNPCProps.z_translate_pos = -6.0;

			printf("Enemies Destroyed: %d\n", npcs_destroyed);
		}
		else if (
			torpedo.t_x_pos > NPC2_BB.min.x&&
			torpedo.t_x_pos < NPC2_BB.max.x &&
			torpedo.t_y_pos > NPC2_BB.min.y&&
			torpedo.t_y_pos < NPC2_BB.max.y &&
			torpedo.t_z_pos > NPC2_BB.min.z&&
			torpedo.t_z_pos < NPC2_BB.max.z
		) {
			npcs_destroyed++;
			submarineNPCProps2.x_translate_pos = 6.0;
			submarineNPCProps2.y_translate_pos = 2.0;
			submarineNPCProps2.z_translate_pos = -3.0;
			submarineNPCProps2.y_axis_rotation = -90.0;
			submarineNPCProps2.NPC_movingStage = 0;

			printf("Enemies Destroyed: %d\n", npcs_destroyed);
		}
		else {
			counter--;
		}
		
	}
	else {
		torpedo.radius = 0.0;
	}
	glutPostRedisplay();
	glutTimerFunc(35, fireTorpedo, 0);	
}

float toRadians(float rotation) {
	float PI = 3.14159265358979323846;
	return (rotation * PI) / 180;
}

float degToRad() { return (PI / 180); }

void NPCMovingRoutine() {

	if (submarineNPCProps.x_translate_pos > 18.0) {
		submarineNPCProps.x_translate_pos = -18.0;
	}
	else {
		submarineNPCProps.x_translate_pos += 0.05;

	}
	NPC_BB.max.x = submarineNPCProps.x_translate_pos + 2.0;
	NPC_BB.max.y = submarineNPCProps.y_translate_pos + 0.40;
	NPC_BB.max.z = submarineNPCProps.z_translate_pos + 0.40;

	NPC_BB.min.x = submarineNPCProps.x_translate_pos - 2.0;
	NPC_BB.min.y = submarineNPCProps.y_translate_pos - 0.40;
	NPC_BB.min.z = submarineNPCProps.z_translate_pos - 0.40;
	glutPostRedisplay();
	glutTimerFunc(10, NPCMovingRoutine, 0);
}

void NPCMovingRoutine2() {

	if (submarineNPCProps2.NPC_movingStage == 0) {
		if (submarineNPCProps2.z_translate_pos > 8.0) {
			submarineNPCProps2.NPC_movingStage = 1;
		}
		else {
			submarineNPCProps2.z_translate_pos += 0.05;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 1) {
		if (submarineNPCProps2.y_axis_rotation == -180.0) {
			submarineNPCProps2.NPC_movingStage = 2;
		}
		else {
			submarineNPCProps2.y_axis_rotation -= 1.0;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 2) {
		if (submarineNPCProps2.x_translate_pos < -6.0 ) {
			submarineNPCProps2.NPC_movingStage = 3;
		}
		else {
			submarineNPCProps2.x_translate_pos -= 0.05;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 3) {
		if (submarineNPCProps2.y_translate_pos > 4.0) {
			submarineNPCProps2.NPC_movingStage = 4;
		}
		else {
			submarineNPCProps2.y_translate_pos += 0.05;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 4) {
		if (submarineNPCProps2.y_axis_rotation == 0.0) {
			submarineNPCProps2.NPC_movingStage = 5;
		}
		else {
			submarineNPCProps2.y_axis_rotation += 1.0;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 5) {
		if (submarineNPCProps2.x_translate_pos > 6.0) {
			submarineNPCProps2.NPC_movingStage = 6;
		}
		else {
			submarineNPCProps2.x_translate_pos += 0.05;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 6) {
		if (submarineNPCProps2.y_axis_rotation == 90.0) {
			submarineNPCProps2.NPC_movingStage = 7;
		}
		else {
			submarineNPCProps2.y_axis_rotation += 1.0;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 7) {
		if (submarineNPCProps2.z_translate_pos < -3.0) {
			submarineNPCProps2.NPC_movingStage = 8;
		}
		else {
			submarineNPCProps2.z_translate_pos -= 0.05;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 8) {
		if (submarineNPCProps2.y_axis_rotation == -90.0) {
			submarineNPCProps2.NPC_movingStage = 9;
		}
		else {
			submarineNPCProps2.y_axis_rotation -= 1.0;
		}
	}

	if (submarineNPCProps2.NPC_movingStage == 9) {
		if (submarineNPCProps2.y_translate_pos < 2.0) {
			submarineNPCProps2.NPC_movingStage = 0;
		}
		else {
			submarineNPCProps2.y_translate_pos -= 0.05;
		}
	}
	NPC2_BB.max.x = submarineNPCProps2.x_translate_pos + 2.0;
	NPC2_BB.max.y = submarineNPCProps2.y_translate_pos + 0.40;
	NPC2_BB.max.z = submarineNPCProps2.z_translate_pos + 0.40;

	NPC2_BB.min.x = submarineNPCProps2.x_translate_pos - 2.0;
	NPC2_BB.min.y = submarineNPCProps2.y_translate_pos - 0.40;
	NPC2_BB.min.z = submarineNPCProps2.z_translate_pos - 0.40;

	glutPostRedisplay();
	glutTimerFunc(10, NPCMovingRoutine2, 0);
}


//void output(int x, int y, float r, float g, float b, int font, char* string)
void outputScore()
{
	char label[100] = "Enemies destroyed: ";
	//printf("%s\n", label);

	char score[12];
	sprintf(score, "%d", npcs_destroyed);
	//printf("%s\n", score);

	strcat(label, score);

	int font = GLUT_BITMAP_HELVETICA_18;
	int r = 0;
	int g = 0;
	int b = 0;
	int x = -9.0;
	int y = 7.0;

	glColor3f(r, g, b);
	glRasterPos2f(x, y);
	int len, i;
	len = (int)strlen(label);
	for (i = 0; i < len; i++) {
		glutBitmapCharacter(font, label[i]);
	}
}