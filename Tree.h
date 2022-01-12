#ifndef TREE_CLASS_H
#define TREE_CLASS_H
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

#include <string>
#include <math.h> 
#include <random>

using namespace std;

struct node { // struct for the node
    float constant;
    int code; // this defines the type node
    bool isOp = false; // if an oporator true
    bool isUnitary = false; // if unitary then true
    bool isFilled = false;
};

class Tree // class to hold a single memeber of a population
{
private:

    void PicNodeEntry(int index); //returns the node that will get set by the fill tree func 

public:
    void setNode(int index, int code);
    float error = 0; // this is the error when compared with the data
    int maxLevel = 8; // by default it will equal 8
    std::vector<node> nodeList; // this is the array that will hold the tree.
    Tree(int maxDepth);
    Tree();
    ~Tree();
    vector<int> GetIndexes();
    void fillTree(int index = 1); // used to init the tree
    static int getLevel(int index);
    int getdepth(int index);
    int getCompexity(int index);

    void print(int index = 1);
    void mutateTree(int mutationStrength, int index = 1);
    void CrossOver(int p1nodeSwitch, int p2nodeSwitch, Tree* p1, Tree* p2, int curParentIndex =1, int childIndex =1);
};

float RangedRand(int range_min, int range_max);
std::string returnSymbolicOP(node curNode);
double evaluateEquation(Tree* testTree, int index, float x, float y, float z);
float doOperation(int code, float left, float right);
#endif