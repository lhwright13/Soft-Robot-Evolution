#ifndef EVOLVESPRING_CLASS_H
#define EVOLVESPRING_CLASS_H

#include<glm/glm.hpp>
#include<glm/gtc/matrix_transform.hpp>
#include<glm/gtc/type_ptr.hpp>
#include<glad/glad.h>
#include<vector>
#include<iostream>
#include <glm/gtx/string_cast.hpp>
#include<thread>

#include<glad/glad.h>
#include<GLFW/glfw3.h>
#include<stb/stb_image.h>
#include"physicsSim.h"
#include "Tree.h"

#include <string>
#include <math.h> 
#include <random>

struct incoding {
    Tree treek;
    Tree treeb;
    Tree treeC;
    Tree TreeM;
    float speed;
};

typedef vector<int> v1d;
typedef vector<v1d> v2d;
typedef vector<v2d> v3d;

struct robArrayObject {
    v3d a;
};

// instences of this class will be a robot.
class EvolveSpring {
public: 
    //vars: 
    std::vector<incoding> equations;
    int popSize;

    incoding fastestEQ;
    vector<float> speedList;
    float xyzScale = 1;
    robArrayObject blankRobotSchem;

    //other constructor for a rob with non fixed morphology
    EvolveSpring(robArrayObject blankRob, int populationSize);
	
    double testRobot(PhysicsSim* system);
    void breedPop(int numSaved);
    void sex(int parent1, int parent2, int child);
    void evolveMorph(int iterations); //this will evolve both the morph and control
    float formMorphologyAndTest(int equationIndex);

private:
    void mutatePop(int numSaved);
    
    void makeNewRobStruct(int index, robArrayObject* newStructure);

};
void tradeTree(Tree* tree, Tree* tree2, Tree* childTree);
void mutateForm(incoding* equForMut, int strangth);
void loadcontroller(object* robot, Tree* kTree, Tree* bTree, Tree* cTree);
void arrayToRobot(robArrayObject, object* newRobot);
void addSprings(object* newRobot);
void addToList(object* o, float i, float  j, float k);
float RangedRand(int range_min, int range_max);
std::string returnSymbolicOP(node curNode);
double evaluateEquation(Tree* testTree, int index, float x, float y, float z);
float doOperation(int code, float left, float right);
bool compareEqs(incoding t1, incoding t2);
void mutateIncoding(incoding* equForMut, int strangth);
float normalize(float oldVal, float min, float max);

//for the mophology:
void simulation(PhysicsSim*);
bool IsSpringIn(object* newRobot, int i, int  j);


#endif