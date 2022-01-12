#include "EvolveSprings.h"
extern float bone;
extern float soft;
extern float testingTime;
extern float dt;
extern int numSaved;
extern float scale;
extern float yShift;
extern float materialThreshhold;

// ******************************************************************************************
//functions for the evolve spring class

//constructor

EvolveSpring::EvolveSpring(robArrayObject blankRob, int populationSize) {
	popSize = populationSize;
	blankRobotSchem = blankRob;

	//building the population of equations:
	for (int i = 0; i < popSize; i++) {
		//populate:
		Tree kTree(8);
		Tree bTree(8);
		Tree CTree(8);
		Tree MTree(4);
		incoding newIncoding;
		newIncoding.treek = kTree;
		newIncoding.treeb = bTree;
		newIncoding.treeC = CTree;
		newIncoding.TreeM = MTree;
		cout << endl;
		cout << endl;
		kTree.print();
		cout << endl;
		bTree.print();
		cout << endl;
		equations.push_back(newIncoding);
		
	}
}

void  EvolveSpring::evolveMorph(int iterations) {

	for (int i = 0; i < iterations; i++) {
		for (int j = 0; j < popSize; j++) { // here we look at each indeividual incoding
			//now we test each member of the population:
			double distance = formMorphologyAndTest(j); //this should build the robot, test and return speed
			if (distance > 150) // if somthing weird went down...
				equations[j].speed = 0;
			else
				equations[j].speed = distance;
			cout << distance << endl;
		}
		std::sort(equations.begin(), equations.end(), compareEqs); // does reordering, fastest first
		if (equations[0].speed > fastestEQ.speed) {
			fastestEQ = equations[0];
			speedList.push_back(equations[0].speed);
		}
		//replace the falures with mutated parents.
		breedPop(numSaved);
		mutatePop(numSaved);
	}
}

void EvolveSpring::makeNewRobStruct(int index, robArrayObject* newStructure) {
	float morfValue; //found using the equations
	// here we change some of the 1s to zeros... changing the 3d representation file
	for (int i = 0; i < newStructure->a.size(); i++) {
		for (int j = 0; j < newStructure->a[i].size(); j++) {
			for (int k = 0; k < newStructure->a[i][j].size(); k++) {
				//here we plug in the equation of morphology and if greater than some value make zero
				morfValue = evaluateEquation(&equations[index].TreeM, 1, i, j, k);
				//if not the first value and less than 0 kill it.
				if ((morfValue < materialThreshhold) && !((i == 0) && (j == 0) && (k == 0))) {
					newStructure->a[i][j][k] = 0;
				}
				else {
				}
			}
		}
	}
}

float EvolveSpring::formMorphologyAndTest(int equationIndex) {
	//create the new structure array
	robArrayObject newStructure;
	newStructure.a = blankRobotSchem.a; // create a copy to alter
	makeNewRobStruct(equationIndex, &newStructure); // this will use the eq to change some 1s to 0s

	//build and set the bestBot.
	object blankBot; // this adress will be passed around;
	arrayToRobot(newStructure, &blankBot); //adding the spring and masses using the new form
	loadcontroller(&blankBot, &(equations[0].treek), &(equations[0].treeb), &(equations[0].treeC));
	PhysicsSim newSystem(&blankBot); // creating the new system

	float distance;
	//at this point we have a robot ready to test called newSystem
	distance = testRobot(&newSystem);
	return distance;
}


void EvolveSpring::breedPop(int numSaved) {
	for (int child = (numSaved); child < (popSize); child++) { // run through the non saved part of the population:

		int parent1 = round(RangedRand(0, numSaved));
		int parent2 = round(RangedRand(0, numSaved));
		mutateIncoding(&equations[parent1], 10); //very mild mutation
		mutateIncoding(&equations[parent1], 5); //very mild mutation
		// create a vector of all the indexes with information
		sex(parent1, parent2, child);

		mutateIncoding(&equations[parent1], 20); //very mild mutation
		mutateIncoding(&equations[parent1], 10); //very mild mutation
		
	}
}

void EvolveSpring::sex(int parent1, int parent2, int child) {
	//here we have cross over pick 
	tradeTree(&equations[parent1].treeb, &equations[parent2].treeb, &equations[child].treeb);
	tradeTree(&equations[parent1].treeC, &equations[parent2].treeC, &equations[child].treeC);
	tradeTree(&equations[parent1].treek, &equations[parent2].treek, &equations[child].treek);
	tradeTree(&equations[parent1].TreeM, &equations[parent2].TreeM, &equations[child].TreeM);
}

void tradeTree(Tree* tree1, Tree* tree2, Tree* childTree) {
	//first get their indexes: 
	vector <int> p1Ind = tree1->GetIndexes();
	vector <int> p2Ind = tree2->GetIndexes();

	//use a gaussian distubution to try and select noted in the middle of the tree.
	default_random_engine generator;
	float variance1 = p1Ind.size() / 6.0; //99.99966% out of bounds... lets hope not
	float variance2 = p2Ind.size() / 6.0;

	normal_distribution<float> distribution1(p1Ind.size() / 2.0 + 1, variance1);
	normal_distribution<float> distribution2(p2Ind.size() / 2.0 + 1, variance2);

	int pick1 = round(distribution1(generator));
	int pick2 = round(distribution1(generator));

	childTree->CrossOver(pick1, pick2, tree1, tree2);
	
}

void loadcontroller(object* robot, Tree* kTree, Tree* bTree, Tree* cTree) {
	// this needs some nomalization...
	float xmin = robot->robParticles[0].pos.x;
	float xmax = robot->robParticles[0].pos.x;
	float ymin = robot->robParticles[0].pos.y;
	float ymax = robot->robParticles[0].pos.y;
	float zmin = robot->robParticles[0].pos.z;
	float zmax = robot->robParticles[0].pos.z;

	//find the max and mins
	for (int i = 0; i < robot->robParticles.size(); i++) {
		if (robot->robParticles[i].pos.x < xmin)
			xmin = robot->robParticles[i].pos.x;
		else if (robot->robParticles[i].pos.x > xmax)
			xmax = robot->robParticles[i].pos.x;

		if (robot->robParticles[i].pos.y < ymin)
			ymin = robot->robParticles[i].pos.y;
		else if (robot->robParticles[i].pos.y > ymax)
			ymax = robot->robParticles[i].pos.y;

		if (robot->robParticles[i].pos.z < zmin)
			zmin = robot->robParticles[i].pos.z;
		else if (robot->robParticles[i].pos.z > zmax)
			zmax = robot->robParticles[i].pos.z;
	}

	//go through all the springs
	for (int i = 0; i < robot->robSprings.size(); i++) {
		float x1 = robot->robSprings[i].particle1->pos.x;
		float y1 = robot->robSprings[i].particle1->pos.y;
		float z1 = robot->robSprings[i].particle1->pos.z;
		float x2 = robot->robSprings[i].particle2->pos.x;
		float y2 = robot->robSprings[i].particle2->pos.y;
		float z2 = robot->robSprings[i].particle2->pos.z;

		float newK1;
		float newK2;
		float newb1;
		float newb2;
		float newc1;
		float newc2;

		//find the values for at both points and avergae them for K
		newK1 = evaluateEquation(kTree, 1, normalize(x1, xmin, xmax), \
			normalize(y1, ymin, ymax), normalize(z1, zmin, zmax));
		newK2 = evaluateEquation(kTree, 1, normalize(x2, xmin, xmax), \
			normalize(y2, ymin, ymax), normalize(z2, zmin, zmax));

		if (0 < (newK1 + newK2) / 2) {  // setting it as bone or soft
			robot->robSprings[i].k = bone;
		}
		else
			robot->robSprings[i].k = soft;

		//find the values for at both points and avergae them for K
		newb1 = evaluateEquation(bTree, 1, normalize(x1, xmin, xmax), \
			normalize(y1, ymin, ymax), normalize(z1, zmin, zmax)) * .8;
		newb2 = evaluateEquation(bTree, 1, normalize(x2, xmin, xmax), \
			normalize(y2, ymin, ymax), normalize(z2, zmin, zmax)) * .8;
		if ((robot->robSprings[i].OGlength / 14 > ((newb1 + newb2) / 4)) &&  // must be between .5 and  -.5
			(-robot->robSprings[i].OGlength / 14 < ((newb1 + newb2) / 4))) {  // the b value is dangouous
			robot->robSprings[i].b = (newb1 + newb2) / 4; //trim if returns a value to large
		}
		else if ((newb1 + newb2) > 0)
			robot->robSprings[i].b = robot->robSprings[i].OGlength / 7;
		else
			robot->robSprings[i].b = robot->robSprings[i].OGlength / -7;

		//find the values for at both points and avergae them for K
		newc1 = evaluateEquation(cTree, 1, normalize(x1, xmin, xmax), \
			normalize(y1, ymin, ymax), normalize(z1, zmin, zmax));
		newc2 = evaluateEquation(cTree, 1, normalize(x2, xmin, xmax), \
			normalize(y2, ymin, ymax), normalize(z2, zmin, zmax));

		robot->robSprings[i].c = (newb1 + newb2 / 2);

	}
}

double EvolveSpring::testRobot(PhysicsSim* system) {
	glm::vec3 OGpos = system->robot->robParticles[0].pos;
	int frames = testingTime * 60;
	for (int i = 0; i < frames; i++) {
		system->newFrame();
		system->Breath(i * (1 / 60));
	}
	//return the distance that it traveled after t time
	return glm::distance(OGpos, system->robot->robParticles[0].pos);
}

void EvolveSpring::mutatePop(int numSaved) {
	int whichParent = 0;
	for (int i = (numSaved); i < (popSize); i++) { // run through the non saved part of the population:
		equations[i] = equations[whichParent];//copying the parent into the falure's spot
		mutateIncoding(&equations[i], (50));
		mutateForm(&equations[i], (10));
		whichParent++;
		if (whichParent == numSaved)
			whichParent = 0;
	}

	for (int i = 0; i < numSaved; i++) { //lastly small mutation for parents
		mutateIncoding(&equations[i], (10));
		mutateForm(&equations[i], (5));
	}
}

// THIS IS THE STUFF FOR THE MOPHOLOGY: &&&&&&&&&&&&&&&&&&&&&&&&*

//take an 3-d array and returns an robot object
void arrayToRobot(robArrayObject array, object* newRobot) {
	//loop through the 3d array and replace each point with a cube
	for (int i = 0; i < array.a.size(); i++) {
		for (int j = 0; j < array.a[i].size(); j++) {
			for (int k = 0; k < array.a[i][j].size(); k++) {
				//we need a list of all the springs and masses.
				//make 4 points
				if (array.a[i][j][k] == 1)
					addToList(newRobot, i, j, k);
			}
		}
	}
	// now we add the spring by iterating through the masses:
	addSprings(newRobot);
}
//this is working I think
void addSprings(object* newRobot) {
	for (int i = 0; i < (newRobot)->robParticles.size(); i++) {
		for (int j = i + 1; j < (newRobot)->robParticles.size(); j++) {
			//if its not already in the list add it
			float distance = glm::distance((newRobot)->robParticles[i].pos, (newRobot)->robParticles[j].pos);
			if (!IsSpringIn(newRobot, i, j) && (distance <= sqrt(3 * pow(scale, 2)))) {
				spring s;
				s.particle1 = (&((newRobot)->robParticles[i]));
				s.particle2 = &((newRobot)->robParticles[j]);
				s.restLength = distance;
				s.OGlength = distance;
				(newRobot)->robSprings.push_back(s);
			}
		}
	}
}

//this still needs to be fixed, not hitting the duplicates
bool IsSpringIn(object* newRobot, int i, int  j) {
	for (int k = 0; k < (newRobot)->robSprings.size(); k++) {
		if (((newRobot)->robSprings[k].particle1->pos == (newRobot)->robParticles[i].pos)) {
			if ((newRobot)->robSprings[k].particle2->pos == (newRobot)->robParticles[j].pos) {
				return true;
			}
		}
		if (((newRobot)->robSprings[k].particle1->pos == (newRobot)->robParticles[j].pos)) {
			if ((newRobot)->robSprings[k].particle2->pos == (newRobot)->robParticles[i].pos) {
				return true;
			}
		}
	}
	return false;
}
//add a block to the robot
void addToList(object* newRobot, float i, float  j, float k) {
	//these are temps creats boxes of 2 * scale
	particle p1;
	p1.pos = glm::vec3(i * scale + scale / 2, j * scale + scale / 2 + yShift, k * scale + scale / 2);
	newRobot->robParticles.push_back(p1);
	particle p2;
	p2.pos = glm::vec3(i * scale + scale / 2, j * scale + scale / 2 + yShift, k * scale - scale / 2);
	newRobot->robParticles.push_back(p2);
	particle p3;
	p3.pos = glm::vec3(i * scale + scale / 2, j * scale - scale / 2 + yShift, k * scale + scale / 2);
	newRobot->robParticles.push_back(p3);
	particle p4;
	p4.pos = glm::vec3(i * scale + scale / 2, j * scale - scale / 2 + yShift, k * scale - scale / 2);
	newRobot->robParticles.push_back(p4);
	particle p5;
	p5.pos = glm::vec3(i * scale - scale / 2, j * scale + scale / 2 + yShift, k * scale + scale / 2);
	newRobot->robParticles.push_back(p5);
	particle p6;
	p6.pos = glm::vec3(i * scale - scale / 2, j * scale + scale / 2 + yShift, k * scale - scale / 2);
	newRobot->robParticles.push_back(p6);
	particle p7;
	p7.pos = glm::vec3(i * scale - scale / 2, j * scale - scale / 2 + yShift, k * scale + scale / 2);
	newRobot->robParticles.push_back(p7);
	particle p8;
	p8.pos = glm::vec3(i * scale - scale / 2, j * scale - scale / 2 + yShift, k * scale - scale / 2);
	newRobot->robParticles.push_back(p8);

	//array of the addresses of all the new points
	particle* newPoints[8] = { &p1,&p2,&p3,&p4,&p5,&p6,&p7,&p8 };

	std::vector<particle> allPoints = newRobot->robParticles;
	int NumOfPoints = size(newRobot->robParticles);
	std::vector <int> toDelete;

	//now we check for duplicates and combine them:
	for (int i = 0; i < NumOfPoints - 8; i++) {
		for (int j = NumOfPoints - 1; j >= NumOfPoints - 8; j--) {
			if (allPoints[i].pos == allPoints[j].pos) {
				//reset the point 
				newPoints[j - (NumOfPoints - 8)] = &allPoints[i];
				//save to delete later
				toDelete.push_back(j);
			}

		}
	}
	sort(toDelete.begin(), toDelete.end());
	//delete the elements that need to be delte, this will also resize 
	for (int i = toDelete.size() - 1; i >= 0; i--) {
		allPoints.erase(allPoints.begin() + toDelete[i]);
	}

	newRobot->robParticles = allPoints;

	//now we should check for duplicates the same way:
	//it works at this point 
}


void mutateIncoding(incoding* equForMut, int strangth) {
	equForMut->treeb.mutateTree(strangth);
	equForMut->treeC.mutateTree(strangth);
	equForMut->treek.mutateTree(strangth);
}

void mutateForm(incoding* equForMut, int strangth) {
	equForMut->TreeM.mutateTree(strangth);
}


bool compareEqs(incoding t1, incoding t2)
{
	return (t1.speed > t2.speed);
}

//fit it between -3 aned 3, where the action is!!! 
float normalize(float oldVal, float min, float max) {
	return (oldVal / (max - min) * 14 - 5);
}