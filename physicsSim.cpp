#include"physicsSim.h"


extern glm::vec3 g; // y is the vert direction
extern float dt;
extern double sFriction;
extern double kFriction;
extern double w; //frequency
extern float drag;

//PhysicsSim::PhysicsSim() {
//	//dont really need this
//}

	//constructor takes the list of virtices (the masses are incoded in the spring structure.
PhysicsSim::PhysicsSim() { // for box
}


PhysicsSim::PhysicsSim(object* newRobot) { // for new robot
	robot = newRobot; // deref pointer
	
	constantSprings = robot->robSprings; //saved for movement purposes
	GLfloat* v; //dynamically allocated thus we must pass the size around
	int n = robot->robParticles.size() * 8;
	v = new GLfloat[n];
	vertices = v;
	upDateVertices();

	GLuint* I; //dynamically allocated thus we must pass the size around
	n = robot->robSprings.size()*2; // times two for both index
	I = new GLuint[n];
	indices = I;
	generateInd();
}

void PhysicsSim::generateInd() {// ***********************************************************
	//here we are going to iterate through the springs
	//and create the mapping for the graphic. 
	//we can do this by subtracting the start of the array and its address

	for (int i = 0; i < robot->robSprings.size(); i++) {
		for (int j = 0; j < robot->robParticles.size(); j++) {
			if (&(robot->robParticles[j]) == robot->robSprings[i].particle1){
				indices[i * 2] = j;
			}
			if (&(robot->robParticles[j]) == robot->robSprings[i].particle2) {
				
				indices[i * 2 + 1] = j;
			}
		}
	}
}

//returns a GLfloat of vertices ready for simulation:
void PhysicsSim::newFrame() {

	//call the physics loop for one frame
	physicsLoop(frameRate/dt);

	//take the new particle list and make virt list and return it.
	upDateVertices();

	// the following is used for calculating energy of system:
	/*float Kenergy = 0;
	float Penergy = 0;
	float Genergy = 0;
	float total = 0;*/

	/*for (int part = 0; part < size(particles); part++) {
		Genergy += particles[part].mass * 9.8 * particles[part].pos.y;
		Kenergy += 0.5 * particles[part].mass * pow(length(particles[part].vel), 2);
	}
	for (int spring = 0; spring < size(springs); spring++) {
		float Lcurrent = glm::distance(springs[spring].particle1->pos, springs[spring].particle2->pos);
		float deltaL = Lcurrent - springs[spring].restLength;
		Penergy += 0.5 * K * pow(deltaL, 2);
	}
	total = Penergy + Kenergy + Genergy;
	cout << Penergy << "	,	"<< Kenergy << "	,	" << Genergy << "	," << total << endl;*/
	return;
}

//takes the GLfloat list and converts it to a more useful object.
//void PhysicsSim::convertGLtoObject(GLfloat* vertices) {
//	//connect the particle and springs lists to robot
//	//setting the particles in the array:
//	for (int i = 0; i < numPart; i++) {
//		particle temp; //temp is a particle struct type
//		// 8 becasue of the other info incoded in vertices: color and texture
//		glm::vec3 v(vertices[i * 8] * 10, vertices[i * 8 + 1] * 10, vertices[i * 8 + 2] * 10);
//		temp.pos = v;
//		robot->robParticles.push_back(temp);
//	}
//
//	//setting the springs up in their array: //this will need to change for alternate shapes. **************************
//	for (int i = 0; i < numPart-1; i++) {
//		for (int j = 1 + i; j < numPart; j++) { // loop connects all points together, NO DOUBLE LINKAGE
//			spring s;
//			s.particle1 = &robot->robParticles[i];
//			s.particle2 = &robot->robParticles[j];
//			float temp = glm::distance(s.particle1->pos, s.particle2->pos);
//			s.restLength = temp;
//			s.k = K; 
//			robot->robSprings.push_back(s);
//		}
//	}
//}

void PhysicsSim::printPoints() {

	//prints the location of all particles
	for (int i = 0; i < robot->robParticles.size(); i++)
		std::cout << glm::to_string(robot->robParticles[i].pos) << std::endl << ' ';
}
void PhysicsSim::printSprings() {
	//prints the location of all particles
	for (int i = 0; i < robot->robSprings.size(); i++)
		std::cout << glm::to_string(robot->robSprings[i].particle1->pos) <<"	,	"<<  \
		glm::to_string(robot->robSprings[i].particle2->pos) << std::endl;
}

void PhysicsSim::Breath(float t) { //this is now just move
	for (int i = 0; i < size(robot->robSprings); i++) {
		float b = robot->robSprings[i].b;
		float c = robot->robSprings[i].c;
		robot->robSprings[i].restLength = robot->robSprings[i].OGlength + \
			b *1.4* sin(w * t + c);
	}
}

void PhysicsSim::upDateVertices() {
	for (int i = 0; i < robot->robParticles.size(); i++) {
		// 8 becasue of the other info incoded in vertices: color and texture
		vertices[i * 8] = robot->robParticles[i].pos.x * 0.1;
		vertices[i * 8 + 1] = robot->robParticles[i].pos.y * 0.1;
		vertices[i * 8 + 2] = robot->robParticles[i].pos.z * 0.1;

		//we can also put color code stuff in here if we want
	}
}
//update the rest lengths stuff and we should be right as rain.
void PhysicsSim::physicsLoop(int steps) { //around 150 steps per frame ***********************gravity is working, the springs are not
	
	//number of time steps we are going to take:
	for (int i = 0; i < steps; i++){
		forces();
		//threddedFVP();
		//velocityAndPosition();
	}
}

void PhysicsSim::forces() {
	//we need to reset the accelaration
	for (int part = 0; part < size(robot->robParticles); part++)
		robot->robParticles[part].accel = { 0,0,0 };

	//for each spring calculate its forces
	for (int spring = 0; spring < size(robot->robSprings); spring++) {
		//calculate the force, on both particles:

		float Lcurrent = glm::distance(robot->robSprings[spring].particle1->pos, robot->robSprings[spring].particle2->pos);
		float deltaL = Lcurrent - robot->robSprings[spring].restLength;
		float force = robot->robSprings[spring].k * deltaL; // this is a magnatude

		//add the accelaration to the particles a = F/M * normalized vector for direction:
		glm::vec3 direction = normalize(robot->robSprings[spring].particle2->pos - robot->robSprings[spring].particle1->pos);
		robot->robSprings[spring].particle1->accel += force * direction / robot->robSprings[spring].particle1->mass;
		robot->robSprings[spring].particle2->accel += force * -direction / robot->robSprings[spring].particle2->mass; // equal and opposite
		
	}


	//for gravity and friction:
	for (int part = 0; part < size(robot->robParticles); part++) {
		//add gravity to the accelaration basically 9.8m/s/s in the -y direction
		robot->robParticles[part].accel += g;
		//this is for the floor, puts a force on the point relative to how far below the floor it is:
		if (robot->robParticles[part].pos.y < 0) {
			float normalForce = -(robot->robParticles[part].pos.y) * 1000000;
			robot->robParticles[part].accel.y += normalForce / robot->robParticles[part].mass;
			// fric calculation:
			double hforce = sqrt(pow(robot->robParticles[part].accel.x * robot->robParticles[part].mass, 2) + \
				pow(robot->robParticles[part].accel.z * robot->robParticles[part].mass, 2));
			//static -> no movement
			if (hforce < normalForce * sFriction) {
				robot->robParticles[part].accel.x = 0;
				robot->robParticles[part].accel.z = 0;
			}
			else { // kinetic REDUCE By ratio of hforce-fric to hforce
				robot->robParticles[part].accel.x *= ((hforce - (normalForce * sFriction)) / hforce);
				robot->robParticles[part].accel.z *= ((hforce - (normalForce * sFriction)) / hforce);
			}

		}

		//calculate the new velocity after some dt v = da/dt ie a*dt + v_0 * drag
		robot->robParticles[part].vel = (robot->robParticles[part].vel * drag + robot->robParticles[part].accel * dt);


		//calculate the new position after some dt p = dv/dt ie v*dt + p_0
		robot->robParticles[part].pos += robot->robParticles[part].vel * dt;
	}

}