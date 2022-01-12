#include "Tree.h"
extern float bone;
extern float soft;
extern float testingTime;
extern float dt;
extern int numSaved;

/* this file will take a robot structure and evolve the parameters for it
this will be done using Compositional Pattern Producing Networks (CPPNs) */
Tree::Tree() {
	int maxLevel = 8;
	nodeList.resize(1000);
	//fillTree();
}

//constructor:
Tree::Tree(int maxDepth) {
	int maxLevel = maxDepth;
	nodeList.resize(1500);
	fillTree();
}

Tree::~Tree() {}

//called by constructor to fill tree.
void Tree::fillTree(int index) { //varible argument
	if (nodeList.capacity() < (index * 2 + 2)) { // plus 2 becasue index starts at 1
		nodeList.resize(index * 2 + 2);
	}

	PicNodeEntry(index); // add the new node, the node type depends on level
	if (!nodeList[index].isOp)
		return;
	//fill the children of the current node, if not unitary fill with 2 kids.
	fillTree((index * 2) + 1); // put child of unitary on the right sin(x) not xsin
	if (!(nodeList[index].isUnitary)) { //if not unitary add the other child too
		fillTree(index * 2);
	}
	return;
}

//pics the node probabalistically based on tree depth 
void Tree::PicNodeEntry(int index) {

	int pick = round(RangedRand(0, pow(maxLevel, 2))); //(0-mxlev^2)

	if (pick > pow(getLevel(index), 2)) { //pick  op
		int which_op = round(RangedRand(4, 11)); //returns 4-11
		setNode(index, which_op); // 
	}
	else {
		int x_or_const = round(RangedRand(0, 3)); 
		setNode(index, x_or_const); //  x,y,z or const
	}
}

//sets the node given an index and code
void Tree::setNode(int index, int code) {
	if (nodeList.capacity() < (index * 2 + 2)) { // plus 2 becasue index starts at 1
		nodeList.resize(index * 2 + 2);
	}

	nodeList[index].code = code; // set the code property in the struct
	nodeList[index].isFilled = true;
	nodeList[index].constant = 0; //clear it first
	switch (code) {
	case 0: // 0 code == x variable
		nodeList[index].isOp = false;
		nodeList[index * 2].isFilled = false;
		nodeList[index * 2 + 1].isFilled = false;
		break;
	case 1: // 0 code == y variable
		nodeList[index].isOp = false;
		nodeList[index * 2].isFilled = false;
		nodeList[index * 2 + 1].isFilled = false;
		break;
	case 2: // 0 code == z variable
		nodeList[index].isOp = false;
		nodeList[index * 2].isFilled = false;
		nodeList[index * 2 + 1].isFilled = false;
		break;
	case 3: // constant
		nodeList[index * 2].isFilled = false;
		nodeList[index * 2 + 1].isFilled = false;
		nodeList[index].isOp = false;
		nodeList[index].constant = (RangedRand(-2, 2));
		break;
	case 4: // /
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = false;
		break;
	case 5: // *
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = false;
		break;
	case 6: // sin
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;
		break;
	case 7: // cos
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;
		break;
	case 8: // tanh
		//nodeList[index * 2].isFilled = false; //setting left child false if not
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;
		break;
	case 9: // reluAct
		//nodeList[index * 2].isFilled = false; //setting left child false if not
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;
		break;
	case 10: //log
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;
		break;
	case 11: //gaussian
		nodeList[index].isOp = true;
		nodeList[index].isUnitary = true;


	}

}

//retunrs the level given an index
int Tree::getLevel(int index) {
	int count = 1;
	while (!(1 == index)) {
		count++;
		index = floor(index / 2);
	}
	return count;
}

//prints the formula from the tree as a string
void Tree::print(int index) {
	if (!nodeList[index].isFilled) {
		return;
	}

	/* first recur on left child */
	cout << "(";
	print(index * 2);

	/* then print the data of node */
	std::cout << returnSymbolicOP(nodeList[index]) << " ";

	/* now recur on right child */
	print(index * 2 + 1);
	cout << ")";
}
vector<int> Tree::GetIndexes() {
	vector<int> indexList;
	for (int i = 0; i < 255; i++) {
		if (nodeList[i].isFilled)
			indexList.push_back(i);
	}
	return indexList;
}

//mutates the tree, strength is from 0 to 100.
//stregth is the prob of each node being mutated
void Tree::mutateTree(int mutationStrength, int index) {
	if (false == nodeList[index].isFilled) {
		return;
	}
	/* first recur on left child */
	mutateTree(mutationStrength, index * 2);

	//point mutations:
	if (mutationStrength > round(RangedRand(0, 50))) { // pobabalistic mutation
		nodeList[index];
		if (nodeList[index].code < 4) { // if x or constant
			setNode(index, round(RangedRand(0, 3)));
		}
		else if (nodeList[index].code < 6) {
			setNode(index, round(RangedRand(4, 5))); // change binary op
		}
		else {
			setNode(index, round(RangedRand(6, 11))); // change unary
		}
	}
	//heavier mutation:
	if (mutationStrength > RangedRand(0, 300)) {
		if (nodeList.capacity() < (index * 2 + 2)) { // plus 2 becasue index starts at 1
			nodeList.resize(index * 2 + 2);
		}
		fillTree(index); // rewrite a branch
	}

	/* now recur on right child */
	mutateTree(mutationStrength, index * 2 + 1);
}

int Tree::getdepth(int index) { //return the depth of the longest branch
	if (!nodeList[index].isFilled)
		return 0;
	else {
		int left = getdepth(index * 2);
		int right = getdepth(index * 2 + 1);
		if (left > right)
			return left + 1;
		else return right + 1;
	}
}

int Tree::getCompexity(int index) {
	if (!nodeList[index].isFilled)
		return 0;
	else {
		int left = getCompexity(index * 2);
		int right = getCompexity(index * 2 + 1);

		return left + 1 + right + 1;
	}
}

float RangedRand(int min, int max)
{
	random_device rd;
	mt19937 gen(rd());
	uniform_int_distribution<> dist(min * 100, max * 100);
	float randVal = static_cast<float>(dist(gen)) / 100;
	return randVal;
}

std::string returnSymbolicOP(node curNode) {
	int code = curNode.code;
	switch (code) {
	case 0: // 0 code == x variable
		return "x";
		break;
	case 1: // 0 code == x variable
		return "y";
		break;
	case 2: // 0 code == z variable
		return "z";
		break;
	case 3: // constant
		return std::to_string(curNode.constant);
		break;
	case 4: // +
		return "/";
		break;
	case 5: // - 
		return "*";
		break;
	case 6: // /
		return "sin";
		break;
	case 7: // *
		return "cos";
		break;
	case 8: // sin
		return "tanh";
		break;
	case 9: // cos
		return"relu";
		break;
	case 10: //log
		return"log";
		break;
	case 11: //gaussian
		return"1*";
		break;
	}
	cout << code << "not found \n";
	return "error";
}

double evaluateEquation(Tree* testTree, int index, float x, float y, float z) {
	if (false == testTree->nodeList[index].isFilled) { // if it is an empty node
		//this is really only for the uniary oporator and doesn't do anything
		return 0;
	}
	if (!testTree->nodeList[index].isOp) { // if it is a constant then return it.
		if (testTree->nodeList[index].code == 0) { // is the var x
			return x;
		}
		else if (testTree->nodeList[index].code == 1) {
			return y;
		}
		else if (testTree->nodeList[index].code == 2) {
			return z;
		}
		else {
			return testTree->nodeList[index].constant; // is a constant
		}
	}

	float leftval = evaluateEquation(testTree, index * 2, x, y, z);
	float rightval = evaluateEquation(testTree, index * 2 + 1, x, y, z);
	/*cout << " here is the op\n" << leftval << returnSymbolicOP(equation.nodeList[index]) << rightval << '\n';*/

	return doOperation(testTree->nodeList[index].code, leftval, rightval);
}

float doOperation(int code, float left, float right) {
	switch (code) {
	case 0: // 0 code == x variable
		cout << "error send a variable instead of a op";
		return 0;
	case 1: // constant
		cout << "error send a constant instead of a op";
		return 0;
	case 2: // constant
		cout << "error send a constant instead of a op";
		return 0;
	case 3: // constant
		cout << "error send a constant instead of a op";
		return 0;
	case 4: // /
		return left / right;
	case 5: // *
		return left * right;
	case 6: // sin
		return sin(right);
	case 7: // cos
		return cos(right);
	case 8: // thanh
		return tanh(right);
	case 9: // relu activation function
		if (right < 0)
			return 0;
		else 
			return right;
	case 10: // log
		if (right > 0)
			return log(right);
		else
			return tanh(right);
	case 11: // gaussian
		float variance = 1.0;
		random_device rd;
		mt19937 gen(rd());
		std::normal_distribution<float> d(right, variance);
		return d(gen);
	}
	cout << code << "error in the evaluation, code not found \n";
	return 0;
}

void Tree::CrossOver(int p1nodeSwitch, int p2nodeSwitch, Tree* p1, Tree* p2, int curParentIndex, int childIndex){
	if (!p1->nodeList[curParentIndex].isFilled) {
		return;
	}
	if (p1nodeSwitch == curParentIndex) {
		curParentIndex = p2nodeSwitch;
		//add the node from the other parent: 
		setNode(childIndex, p2->nodeList[curParentIndex].code); // clean up flags
		nodeList[childIndex] = p2->nodeList[curParentIndex];
		//recurce, with the parents reversed
		CrossOver(p1nodeSwitch, p2nodeSwitch, p2, p1, curParentIndex*2, childIndex * 2);
		CrossOver(p1nodeSwitch, p2nodeSwitch, p2, p1, curParentIndex*2+1, childIndex * 2+1);
	}
	else if (!(p1->nodeList[curParentIndex].isOp)) {
		setNode(childIndex, p2->nodeList[curParentIndex].code); // clean up flags
		nodeList[childIndex] = p1->nodeList[curParentIndex];
		return;
	}
	else {
		setNode(childIndex, p2->nodeList[curParentIndex].code); // clean up flags
		nodeList[childIndex] = p1->nodeList[curParentIndex];

		CrossOver(p1nodeSwitch, p2nodeSwitch, p1, p2, curParentIndex*2, childIndex * 2);
		CrossOver(p1nodeSwitch, p2nodeSwitch, p1, p2, curParentIndex*2+1, childIndex * 2 + 1);
	}
}