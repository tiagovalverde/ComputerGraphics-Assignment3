#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gl/glut.h>
#include "SubmarineMesh.h"

SubmarineMesh newSubmarine() {
	SubmarineMesh submarineMesh;
	submarineMesh.mat_ambient[0] = submarine_mat_ambient[0];
	submarineMesh.mat_ambient[1] = submarine_mat_ambient[1];
	submarineMesh.mat_ambient[2] = submarine_mat_ambient[2];
	submarineMesh.mat_ambient[3] = submarine_mat_ambient[3];

	submarineMesh.mat_specular[0] = submarine_mat_specular[0];
	submarineMesh.mat_specular[1] = submarine_mat_specular[1];
	submarineMesh.mat_specular[2] = submarine_mat_specular[2];
	submarineMesh.mat_specular[3] = submarine_mat_specular[3];

	submarineMesh.mat_diffuse[0] = submarine_mat_diffuse[0];
	submarineMesh.mat_diffuse[1] = submarine_mat_diffuse[1];
	submarineMesh.mat_diffuse[2] = submarine_mat_diffuse[2];
	submarineMesh.mat_diffuse[3] = submarine_mat_diffuse[3];

	submarineMesh.mat_shininess[0] = submarine_mat_shininess[0];

	// custom submarine properties




	return submarineMesh;
}

/* drawSubmarine draws all the objects contained in the submarine
*	x_pos: new x coordinates
*	y_pos: new y coordinates
*	z_pos: new z coordinates
*	y_rot: new y rotation
*	propeller_x_rot: new x rotation of the propellers
*/
void drawSubmarine(float x_pos, float y_pos, float z_pos, float y_rot, float propeller_x_rot) {

	// Base of the submarine
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // rotate left or right
	glScalef(2.0, 0.40, 0.40); // define the shape of the sphere to be base
	glutSolidSphere(1, 50, 50);
	glPopMatrix();


	// Top of the submarine block (central room)
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation
	glTranslatef(0.0, 0.40, 0.0); // adjsuts position according to new base position
	glScalef(1.0, 0.20, 0.20); // defines the shape of the block
	glutSolidCube(1.0);
	glPopMatrix();

	// front fins
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation
	glTranslatef(1.0, 0.0, 0.0); // adjsuts position according to new base position
	glScalef(0.25, 0.05, 1.0); // defines the shape of the block
	glutSolidCube(1.0);
	glPopMatrix();

	// fins from top (central room)
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos);   // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation
	glTranslatef(0.20, 0.45, 0.0); // adjsuts position according to new base position
	glScalef(0.25, 0.03, 0.50); // defines the shape of the block
	glutSolidCube(1.0);
	glPopMatrix();


	/************************
	 * Submarine Back Fins **
	************************/
	// fin back top
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos);  // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.7, 0.25, 0.0); // adjust position of fin in correct location
	glRotatef(5.0, 0.0, 0.0, 1.0); // adjust position of fin in correct location
	glScalef(0.30, 0.20, 0.025);  // define shape of the fin
	glutSolidCube(1.0);
	glPopMatrix();

	// fin back bottom
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos);  // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.7, -0.20, 0.0); // adjust position of fin in correct location
	glRotatef(-5.0, 0.0, 0.0, 1.0); // adjust position of fin in correct location
	glScalef(0.30, 0.20, 0.025);  // define shape of the fin
	glutSolidCube(1.0);
	glPopMatrix();

	// fin back right
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.7, 0.0, 0.15); // adjust position of fin in correct location
	glRotatef(-5.0, 0.0, 0.0, 1.0);// adjust position of fin in correct location
	glScalef(0.20, 0.025, 0.30);  // define shape of the fin
	glutSolidCube(1.0);
	glPopMatrix();
	// fin back left
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.7, 0.0, -0.15); // adjust position of fin in correct location
	glRotatef(-5.0, 0.0, 0.0, 1.0); // adjust position of fin in correct location
	glScalef(0.20, 0.025, 0.30);  // define shape of the fin
	glutSolidCube(1.0);
	glPopMatrix();


	/*************************
	* Propeller Blades Objects
	*************************/

	// propeller blade 1
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.95, 0.0, 0.0); // adjust position of blade in correct location
	glRotatef(propeller_x_rot, 1.0, 0.0, 0.0); // adjust propeller new angle
	glScalef(0.02, 0.45, 0.10); // define the shape of the object
	glutSolidCube(1.0);
	glPopMatrix();

	// propeller blade 2
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base coordinates
	glRotatef(y_rot, 0.0, 1.0, 0.0); // define base rotation 
	glTranslatef(-1.95, 0.0, 0.0); // adjust position of blade in correct location
	glRotatef(propeller_x_rot, 1.0, 0.0, 0.0); // adjust propeller new angle
	glRotatef(-90.0, 1.0, 0.0, 0.0);
	glScalef(0.02, 0.45, 0.10); // define the shape of the object
	glutSolidCube(1.0);
	glPopMatrix();

	/*************************
	* Periscope Objects
	*************************/
	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // rotate left or right
	glTranslatef(0.20, 0.40, 0.0);
	glScalef(0.05, 0.50, 0.05); // define the shape of the sphere to be base
	glutSolidCube(1.0);
	glPopMatrix();

	glPushMatrix();
	glTranslatef(x_pos, y_pos, z_pos); // define base position
	glRotatef(y_rot, 0.0, 1.0, 0.0); // rotate left or right
	glTranslatef(0.25, 0.625, 0.0);
	glScalef(0.10, 0.05, 0.05); // define the shape of the sphere to be base
	glutSolidCube(1.0);
	glPopMatrix();
}

void drawTorpedo(float radius) {

	glPushMatrix();
	//glTranslatef(subX, subY, subZ);
	glRotatef(45.0, 0.0, 0.0, 0.0);
	glScalef(1.0, 1.0, 1.0);
	glutSolidSphere(radius, 20, 20);
	glPopMatrix();
}