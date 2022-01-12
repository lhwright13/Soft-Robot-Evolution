//By Lucas Wright.
//Some OpenGL code barrowed from Victor Gordan see
//Program to evolve robots,
/* for part 2 I want to write a program that read an object file and
converts the vectors to spring so that I can run my evolutionary controller
program on any object and make it move.*/

/* 
get the evolving the morphologies to work.

*/





#include<iostream>
#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<stb/stb_image.h>
#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include <algorithm> 
#include<thread>

#include "Tree.h"
#include"Texture.h"
#include"shaderClass.h"
#include"VAO.h"
#include"VBO.h"
#include"EBO.h"
#include"physicsSim.h"
#include"EvolveSprings.h"

//prototypes:
void simulation(PhysicsSim *);
void makeNewRobStruct1(incoding equation, robArrayObject* newStructure);
void sumulateEqution(EvolveSpring* Evolvedsystem);

// global vars:
glm::vec3 g(0, -10, 0); // y is the vert direction
float dt = 0.0008; // this delta time
float bone = 90000; //stiffness of bone
float soft = 50000; //stiffness of soft tissue
float materialThreshhold = 0; //used to determine if material in bone or soft
double sFriction= .8; //static friction coefficent
double kFriction = .5; // kinetic friction coefficent
const int x = 2; // starting pos of robot 
const int y = 2; // starting pos of robot 
const int z = 2; // starting pos of robot 
const unsigned int width = 1800; //GUI dimentions
const unsigned int height = 1800;
float scale = 3.5; //scale the robot
float yShift = 1.5; //shifts the robot up so that its above the floor at the start
double w = 3; // frequency of controller
float testingTime = 9;// seconds of simulation
int numSaved = 2; //num of robots that are not replaced
float drag = .9995; //drag 


// prints the contents of vector
void print(std::vector <float> const& a) {
	cout << "The vector elements are :";

	for (int i = 0; i < a.size(); i++)
		std::cout << a.at(i) << ' ';
}

// Vertices coordinates
GLfloat vertices[] =
{ //     COORDINATES     /        COLORS      /   TexCoord  //
	-0.5f, 0.3f,  0.5f,     0.83f, 0.0f, 0.44f,	    0.0f, 0.0f,
	-0.5f, 0.3f, -0.5f,     1.0f, 0.70f, 0.44f,	    5.0f, 0.0f,
	 0.5f, 0.3f, -0.5f,     0.83f, 0.70f, 0.44f,	0.0f, 0.0f,
	 0.5f, 0.3f,  0.5f,     0.83f, 0.70f, 0.44f,	5.0f, 0.0f,
	 -0.5f, 1.3f,  0.5f,     0.83f, 0.70f, 0.44f,	0.0f, 0.0f,
	-0.5f, 1.3f, -0.5f,     0.83f, 0.70f, 0.44f,	5.0f, 0.0f,
	 0.5f, 1.3f, -0.5f,     0.83f, 0.70f, 0.44f,	0.0f, 0.0f,
	 0.5f, 1.3f,  0.5f,     0.83f, 0.70f, 0.44f,	5.0f, 0.0f,

};

GLfloat floorVertices[] =
{
-6.0f, -0.05f, 6.0f, 0.83f, 0.0f, 0.44f, -2.0f, 2.0f,
-6.0f, -0.05f, -6.0f, 1.0f, 0.70f, 0.44f, -2.0f, -2.0f,
6.0f, -0.05f, -6.0f, 0.83f, 0.70f, 0.44f, 2.0f, -2.0,
6.0f, -0.05f, 6.0f, 0.83f, 0.70f, 0.44f, 2.0f, 2.0
};

GLuint floorindices[] =
{
	0,1,2,
	0,2,3
};
// Indices for springs
GLuint indices[] =
{
	0, 1,
	0, 2,
	0, 3,
	0, 4,
	0, 5,
	0, 6,
	0, 7,
	1, 2,
	1, 3,
	1, 4,
	1, 5,
	1, 6,
	1, 7,
	2, 3,
	2, 4,
	2, 5,
	2, 6,
	2, 7,
	3, 4,
	3, 5,
	3, 6,
	3, 7,
	4, 5,
	4, 6,
	4, 7,
	5, 6,
	5, 7,
	6, 7,

};

int main()
{

	vector<float> bestOfBestList;
	robArrayObject array;
	//this is where the primitive robot object is placed
	array.a = { {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} } ,{{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} },
		{{1,1,1,1},{1,1,1,1},{1,1,1,1} }, {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} } }; // cube
	EvolveSpring evolvedPop1(array, 3); // this will create the equations and test the robot 

	evolvedPop1.evolveMorph(1);
	
	sumulateEqution(&evolvedPop1);
}

void sumulateEqution(EvolveSpring* Evolvedsystem) {
	robArrayObject array;
	array.a = { {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} } ,{{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} },
		{{1,1,1,1},{1,1,1,1},{1,1,1,1} }, {{1,1,1,1},{1,1,1,1},{1,1,1,1},{1,1,1,1} } };
	//Now simulate the equations we evolved create the new structure array
	robArrayObject newStructure;
	newStructure.a = array.a; // create a copy to alter
	makeNewRobStruct1(Evolvedsystem->fastestEQ, &newStructure); // this will use the eq to change some 1s to 0s

	Evolvedsystem->equations[0].treeb.print();
	cout << endl;
	Evolvedsystem->equations[0].treeC.print();
	cout << endl;
	Evolvedsystem->equations[0].treek.print();
	cout << endl;
	Evolvedsystem->equations[0].TreeM.print();

	cout << endl << Evolvedsystem->equations[0].speed << endl;
	cout << "the speed list: " << endl;
	print(Evolvedsystem->speedList);

	object blankBot;
	arrayToRobot(newStructure, &blankBot); //adding the spring and masses using the new form
	loadcontroller(&blankBot, &(Evolvedsystem->fastestEQ.treek), &(Evolvedsystem->fastestEQ.treeb), &(Evolvedsystem->fastestEQ.treeC));
	PhysicsSim newSystem(&blankBot); // creating the new system

	int x;
	cout << "press go";
	cin >> x;
	simulation(&newSystem);
}

void simulation(PhysicsSim *system1) {
	// FOR THE PLOTTING: 
		// Initialize GLFW
	glfwInit();

	// Tell GLFW what version of OpenGL we are using 
	// In this case we are using OpenGL 3.3
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	// Tell GLFW we are using the CORE profile
	// So that means we only have the modern functions
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Create a GLFWwindow object of 800 by 800 pixels
	GLFWwindow* window = glfwCreateWindow(width, height, "Robot Evolution", NULL, NULL);
	// Error check if the window fails to create
	if (window == NULL)
	{
		std::cout << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}
	// Introduce the window into the current context
	glfwMakeContextCurrent(window);

	//Load GLAD so it configures OpenGL
	gladLoadGL();
	// Specify the viewport of OpenGL in the Window
	// In this case the viewport goes from x = 0, y = 0, to...
	glViewport(0, 0, width, height);

	//sets the size of the points.
	glPointSize(20.0f);
	glLineWidth(5.0f);

	// Generates Shader object using shaders default.vert and default.frag
	Shader shaderProgram("default.vert", "default.frag");


	// Gets ID of uniform called "scale"
	GLuint uniID = glGetUniformLocation(shaderProgram.ID, "scale");

	Texture brickTex("brick.png", GL_TEXTURE_2D, GL_TEXTURE0, GL_RGBA, GL_UNSIGNED_BYTE);
	brickTex.texUnit(shaderProgram, "tex0", 0);

	Texture orangeTex("orange.png", GL_TEXTURE_2D, GL_TEXTURE0, GL_RGBA, GL_UNSIGNED_BYTE);
	orangeTex.texUnit(shaderProgram, "tex0", 0);

	//Variables that help the rotation of the pyramid
	float rotation = 0.0f;
	double prevTime = glfwGetTime();

	// Enables the Depth Buffer
	glEnable(GL_DEPTH_TEST);

	// Main while loop
	VAO VAO1;
	VAO VAO2;
	VAO VAO3;

	int VBOSize = sizeof(GLuint) * system1->robot->robParticles.size() * 8;
	int EBOSize = sizeof(GLuint) * system1->robot->robSprings.size() * 2;

	while (!glfwWindowShouldClose(window))
	{
		// this is setting up the junk for the buffer
		// Generates Vertex Array Object and binds it
		//VAO VAO1;
		VAO1.Bind();
		// Generates Vertex Buffer Object and links it to vertices
		//VBO VBO1(system1->vertices, sizeof(system1->vertices));
		VBO VBO1((system1->vertices), VBOSize);

		// Generates Element Buffer Object and links it to indices
		EBO EBO1(system1->indices, EBOSize);

		// Links VBO attributes such as coordinates and colors to VAO corrisponds to the vitices list
		VAO1.LinkAttrib(VBO1, 0, 3, GL_FLOAT, 8 * sizeof(float), (void*)0);
		VAO1.LinkAttrib(VBO1, 1, 3, GL_FLOAT, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		VAO1.LinkAttrib(VBO1, 2, 2, GL_FLOAT, 8 * sizeof(float), (void*)(6 * sizeof(float)));

		// Unbind all to prevent accidentally modifying them
		VAO1.Unbind();
		VBO1.Unbind();
		EBO1.Unbind();
		//for the floor
		VAO2.Bind();

		// Generates Vertex Buffer Object and links it to vertices
		VBO VBO2(floorVertices, sizeof(floorVertices));
		// Generates Element Buffer Object and links it to indices
		EBO EBO2(floorindices, sizeof(floorindices));

		VAO2.LinkAttrib(VBO2, 0, 3, GL_FLOAT, 8 * sizeof(float), (void*)0);
		VAO2.LinkAttrib(VBO2, 1, 3, GL_FLOAT, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		VAO2.LinkAttrib(VBO2, 2, 2, GL_FLOAT, 8 * sizeof(float), (void*)(6 * sizeof(float)));

		VAO2.Unbind();
		VBO2.Unbind();
		EBO2.Unbind();

		VAO3.Bind();

		// Generates Vertex Buffer Object and links it to vertices
		VBO VBO3((system1->vertices), VBOSize);
		// Generates Element Buffer Object and links it to indices
		EBO EBO3(system1->indices, EBOSize);
		VAO3.LinkAttrib(VBO3, 0, 3, GL_FLOAT, 8 * sizeof(float), (void*)0);
		VAO3.LinkAttrib(VBO3, 1, 3, GL_FLOAT, 8 * sizeof(float), (void*)(3 * sizeof(float)));
		VAO3.LinkAttrib(VBO3, 2, 2, GL_FLOAT, 8 * sizeof(float), (void*)(6 * sizeof(float)));

		VAO3.Unbind();
		VBO3.Unbind();
		EBO3.Unbind();
		// Specify the color of the background
		glClearColor(0.07f, 0.13f, 0.17f, 1.0f);
		// Clean the back buffer and depth buffer
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		// Tell OpenGL which Shader Program we want to use
		shaderProgram.Activate();

		// Simple timer
		double crntTime = glfwGetTime();
		if (crntTime - prevTime >= 0.015)
		{
			rotation += 0.0f;
			prevTime = crntTime;
		}

		//get a new frame!!!!!!!!
		system1->newFrame();

		//This is for if we want it to breath:
		system1->Breath(crntTime);

		// Initializes matrices so they are not the null matrix
		glm::mat4 model = glm::mat4(1.0f);
		glm::mat4 view = glm::mat4(1.0f);
		glm::mat4 proj = glm::mat4(1.0f);
		// Assigns different transformations to each matrix
		model = glm::rotate(model, glm::radians(rotation), glm::vec3(0.0f, 1.0f, 0.0f));
		view = glm::translate(view, glm::vec3(0.0f, -0.9f, -10.f));
		proj = glm::perspective(glm::radians(45.0f), (float)width / height, 0.3f, 150.0f);

		// Outputs the matrices into the Vertex Shader
		int modelLoc = glGetUniformLocation(shaderProgram.ID, "model");
		glUniformMatrix4fv(modelLoc, 1, GL_FALSE, glm::value_ptr(model));
		int viewLoc = glGetUniformLocation(shaderProgram.ID, "view");
		glUniformMatrix4fv(viewLoc, 1, GL_FALSE, glm::value_ptr(view));
		int projLoc = glGetUniformLocation(shaderProgram.ID, "proj");
		glUniformMatrix4fv(projLoc, 1, GL_FALSE, glm::value_ptr(proj));

		orangeTex.Bind();
		// Assigns a value to the uniform; NOTE: Must always be done after activating the Shader Program
		glUniform1f(uniID, 0.5f);
		// Binds texture so that is appears in rendering
		// Bind the VAO so OpenGL knows to use it
		VAO1.Bind();


		glDrawElements(GL_POINTS, system1->robot->robSprings.size() * 2, GL_UNSIGNED_INT, 0);

		orangeTex.Unbind();

		brickTex.Bind();
		VAO1.Unbind();
		//ground stuff
		VAO2.Bind();
		// Draw primitives, number of indices, datatype of indices, index of indices
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		VAO2.Unbind();
		brickTex.Unbind();

		VAO3.Bind();
		// Draw primitives, number of indices, datatype of indices, index of indices
		glDrawElements(GL_LINES, system1->robot->robSprings.size() * 2, GL_UNSIGNED_INT, 0);

		VAO3.Unbind();
		// Swap the back buffer with the front buffer
		glfwSwapBuffers(window);
		// Take care of all GLFW events
		glfwPollEvents();
		VBO1.Delete();
		EBO1.Delete();
		VBO2.Delete();
		EBO2.Delete();
		VBO3.Delete();
		EBO3.Delete();
	}
	// Delete all the objects we've created
	VAO1.Delete();
	VAO2.Delete();
	VAO2.Delete();
	//brickTex.Delete();
	shaderProgram.Delete();
	// Delete window before ending the program
	glfwDestroyWindow(window);
	// Terminate GLFW before ending the program
	glfwTerminate();
	return;
}

void makeNewRobStruct1(incoding equation, robArrayObject* newStructure) {
	float morfValue; //found using the equations
	// here we change some of the 1s to zeros... changing the 3d representation file
	for (int i = 0; i < newStructure->a.size(); i++) {
		for (int j = 0; j < newStructure->a[i].size(); j++) {
			for (int k = 0; k < newStructure->a[i][j].size(); k++) {
				//here we plug in the equation of morphology and if greater than some value make zero
				morfValue = evaluateEquation(&equation.treek, 1, i, j, k);
				if ((morfValue < materialThreshhold) && !((i == 0) && (j == 0) && (k == 0))) {
					newStructure->a[i][j][k] = 0;
				}
			}
		}
	}
}
