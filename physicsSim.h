#ifndef PHYSICSSIM_CLASS_H
#define PHYSICSSIM_CLASS_H
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<glad/glad.h>
#include<vector>
#include<iostream>
#include <glm/gtx/string_cast.hpp>
#include <thread>

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<stb/stb_image.h>

// create a point struct for the masses: 
struct particle {
	glm::vec3 pos;
	glm::vec3 vel = { 0,0,0 };
	glm::vec3 accel = { 0,0,0 };
	float mass = 20; // in kg
};

//create a spring struct L (a+bsin(/omega*t +c)) :
struct spring {
	float k;
	float OGlength;
	float restLength;
	particle* particle1;
	particle* particle2;
	float b;
	float c;
};

struct object {
	std::vector<particle> robParticles;
	std::vector<spring> robSprings;
};


class PhysicsSim {
public:
	float frameRate = 0.015; // seconds/frame
	int numPart = 8; // *****************************************************************

	//default constructor
	PhysicsSim(); 
	//this is for making a robot out of an oject class
	PhysicsSim(object*);

	//returns a GLfloat of vertices ready for simulation:
	void newFrame();
	void printPoints();
	void printSprings();
	void Breath(float scale);
	void generateInd();
	object* robot;

	//list for the simulation
	GLfloat* vertices;
	GLuint* indices;
	void physicsLoop(int steps);
	void threddedFVP();
	void upDateVertices();

private:
	//list of particles and springs in the object
	std::vector<spring> constantSprings;
	void forces();

	friend class EvolveSprings;

};

void forcesT(particle* p1, particle* p2, float restLength, float k);
void partT(particle* p1);

#endif
