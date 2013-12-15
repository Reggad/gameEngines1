#include "Assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Content.h"
#include "Box.h"
#include "Cylinder.h"
#include "ParticleEffect.h"
#include "FountainEffect.h"
#include "SnowEffect.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;


Assignment::Assignment(void)
{
	physicsFactory = NULL;
	dynamicsWorld = NULL;
	broadphase = NULL;
	dispatcher = NULL;
	solver = NULL;
	fullscreen = false;
	elapsed = 10000;
}

	string tag;	

	btHingeConstraint * plclasp;
	btHingeConstraint * hinge1;
	btHingeConstraint * hinge2;
	btHingeConstraint * hinge3;
	btHingeConstraint * hinge4;
	btHingeConstraint * hinge5;

	shared_ptr<PhysicsController> weight;
	shared_ptr<PhysicsController> support;
	shared_ptr<PhysicsController> arm;
	shared_ptr<PhysicsController> sling;
	shared_ptr<PhysicsController> payload;

	glm::vec3 start_support;
	glm::vec3 start_arm;
	glm::vec3 start_weight;
	glm::vec3 start_sling;
	glm::vec3 start_payload;



Assignment::~Assignment(void)
{
}
	bool pressed = false;

//shared_ptr<PhysicsController> cyl;
//std::shared_ptr<GameComponent> station;

bool Assignment::Initialise() 
{
	riftEnabled = false;
	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
	
	tag = "tBox";

    // The world.
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,0,0));


	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	//shared_ptr<PhysicsController> origin = physicsFactory->CreateBox(1,1,1,0, glm::vec3(0, 0, 0), glm::quat()); 

	//physicsFactory->CreateWall(glm::vec3(50,0,20), 10, 10);

	start_support = glm::vec3(5, 10, 0);
	start_arm = glm::vec3(1, 13, 2.5);
	start_weight = glm::vec3(12, 13, 2.5);
	start_sling = glm::vec3(-7, 6.5, 2.5);
	start_payload = glm::vec3(-5, 1, 2.5);

	support = physicsFactory->CreateTBox(1,10,1,0, start_support, glm::quat()); 
	arm = physicsFactory->CreateTBox(15,1,1,1, start_arm, glm::quat()); 
	weight = physicsFactory->CreateTBox(5,3,1.5,1, start_weight, glm::quat()); 
	sling = physicsFactory->CreateTBox(1,10,1,1, start_sling, glm::quat()); 
	payload = physicsFactory->CreateTBox(1,1,1,1, start_payload, glm::quat()); 
/*
	shared_ptr<PhysicsController> seg1 = physicsFactory->CreateBox(1,1,1, glm::vec3(-6.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg2 = physicsFactory->CreateBox(1,1,1, glm::vec3(-4.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg3 = physicsFactory->CreateBox(1,1,1, glm::vec3(-2.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg4 = physicsFactory->CreateBox(1,1,1, glm::vec3(-0.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg5 = physicsFactory->CreateBox(1,1,1, glm::vec3(1.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg6 = physicsFactory->CreateBox(1,1,1, glm::vec3(3.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg7 = physicsFactory->CreateBox(1,1,1, glm::vec3(5.5, 2, 2.5), glm::quat()); 
	shared_ptr<PhysicsController> seg8 = physicsFactory->CreateBox(1,1,1, glm::vec3(7.5, 2, 2.5), glm::quat()); 
*/

	hinge1 = new btHingeConstraint(*arm->rigidBody, *weight->rigidBody, btVector3(9,0,0.0f),btVector3(-3,0,0.0f), btVector3(0,1,0), btVector3(0,0,1), true);
	dynamicsWorld->addConstraint(hinge1);
	hinge2 = new btHingeConstraint(*arm->rigidBody, *sling->rigidBody, btVector3(-8,0,0),btVector3(0,6.5,0), btVector3(0,1,0), btVector3(0,0,1), true);
	dynamicsWorld->addConstraint(hinge2);
	hinge3 = new btHingeConstraint(*support->rigidBody, *arm->rigidBody, btVector3(0,3.5,1.5f),btVector3(4.5,0,-1.0f), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(hinge3);
	plclasp = new btHingeConstraint(*sling->rigidBody, *payload->rigidBody, btVector3(0,-5.5,0),btVector3(-2,0,0), btVector3(0,0,1), btVector3(0,1,0), true);
	dynamicsWorld->addConstraint(plclasp);

/*
	hinge = new btHingeConstraint(*box3->rigidBody, *seg1->rigidBody, btVector3(-7.5,-2,0.0f),btVector3(0,8,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg1->rigidBody, *seg2->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg2->rigidBody, *seg3->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg3->rigidBody, *seg4->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg4->rigidBody, *seg5->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg5->rigidBody, *seg6->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg6->rigidBody, *seg7->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
	hinge = new btHingeConstraint(*seg7->rigidBody, *seg8->rigidBody, btVector3(1,0,0.0f),btVector3(-1,0,0.0f), btVector3(1,0,0), btVector3(1,0,0), true);
	dynamicsWorld->addConstraint(hinge);
*/
	//physicsFactory->CreateRagDoll(glm::vec3(10,10,0));
	//physicsFactory->CreateRagDoll(glm::vec3(20,10,0));
	physicsFactory->CreateRagDoll(glm::vec3(30,10,20));
	
	
	shared_ptr<SnowEffect> snow = make_shared<SnowEffect>();
	Attach(snow);



	ship1 = make_shared<GameComponent>();
	ship1->Attach(Content::LoadModel("cobramk3", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	ship1->position = glm::vec3(-10, 2, -10);
	ship1->Attach(make_shared<VectorDrawer>());
	Attach(ship1);

	ship2 = make_shared<GameComponent>();
	ship2->Attach(Content::LoadModel("ferdelance", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	ship2->Attach(make_shared<VectorDrawer>());
	ship2->diffuse= glm::vec3(1.0f,0.0f,0.0f);
	ship2->specular = glm::vec3(1.2f, 1.2f, 1.2f);
	ship2->position = glm::vec3(10, 2, -10);
	Attach(ship2);

	slerping = false;
	t = 0.0f;

	trail = make_shared<FountainEffect>(500);
	trail->position = ship1->position;
	Attach(trail);

	mass = 1.0f;
	ship1->velocity = glm::vec3(0,0,0);

	if (!Game::Initialise()) {
		return false;
	}

	camera->GetController()->position = glm::vec3(0,20, 70);
	
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
		stringstream ss;	

		ss << "Controls";
        PrintText(ss.str());
        ss.str("");
		ss << "Hold L to apply force to the counter weight";
        PrintText(ss.str());
		ss.str("");
		ss << "Press K to detach the payload";
        PrintText(ss.str());
		ss.str("");


	float newtons = 10.0f;
	float epsilon = glm::epsilon<float>();
	if (keyState[SDL_SCANCODE_UP])
	{
		force += ship1->look * newtons;
	}
	if (keyState[SDL_SCANCODE_DOWN])
	{
		force -= ship1->look * newtons;
	}
	if (keyState[SDL_SCANCODE_LEFT])
	{
		force -= ship1->right * newtons;
	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		force += ship1->right * newtons;
	}
	if (keyState[SDL_SCANCODE_Q] && ! slerping)
	{
		slerping = true;
		fromQuaternion = ship2->orientation;

		glm::vec3 toShip1 = ship1->position - ship2->position;
		toShip1 = glm::normalize(toShip1);
		glm::vec3 axis = glm::cross(GameComponent::basisLook, toShip1);
		axis = glm::normalize(axis);
		float theta = glm::acos(glm::dot(toShip1, GameComponent::basisLook));
		toQuaternion = glm::angleAxis(glm::degrees(theta), axis);
	}
	if (slerping)
	{
		ship2->orientation = glm::mix(fromQuaternion, toQuaternion, t);
		t += timeDelta;
		if (t > 1.0f)
		{
			t = 0.0f;
			slerping = false;
		}
	}

	

	
		if (keyState[SDL_SCANCODE_K])
		{
			
			if(pressed == false)
			{
				dynamicsWorld->removeConstraint(plclasp);
				pressed = true;
			}
		}

		if (keyState[SDL_SCANCODE_L])
		{
			float f = 500.0f;
			glm::vec3 down = glm::vec3(0,-1,0);
			weight->rigidBody->applyCentralForce(GLToBtVector(down) * f);
			//weight->rigidBody->applyTorque(GLToBtVector(glm::vec3(0.0f,1.0f,0.0f)));
		}

		if (keyState[SDL_SCANCODE_R])
		{
			list<shared_ptr<GameComponent>>::iterator it = children.begin();
						dynamicsWorld->removeConstraint(hinge1);
			dynamicsWorld->removeConstraint(hinge2);
			dynamicsWorld->removeConstraint(hinge3);

			 while (it != children.end())
			 {
					 shared_ptr<GameComponent> component = * it;
					 string t = component->tag;
					 if ((component->tag == "tBox"))
						{
								shared_ptr<PhysicsController> physics = dynamic_pointer_cast<PhysicsController> (component->GetController());
								dynamicsWorld->removeRigidBody(physics->rigidBody);
								it = children.erase(it);
						}
						else
						{
								it ++;
						}
					


			 }



			/*
			dynamicsWorld->removeRigidBody(sling->rigidBody);
			dynamicsWorld->removeRigidBody(weight->rigidBody);
			dynamicsWorld->removeRigidBody(arm->rigidBody);
			dynamicsWorld->removeRigidBody(support->rigidBody);
			*/
		}
	// Now calculate the acceleration, new velocity and new position
	glm::vec3 accel = force / mass;
	ship1->velocity += accel * timeDelta;
	ship1->position += ship1->velocity * timeDelta;
	trail->position += ship1->velocity * timeDelta;
	// Check if the velocity length is > epsilon and if so create the look vector from the velocity
	if (glm::length(ship1->velocity) > epsilon)
	{
		ship1->look = glm::normalize(ship1->velocity);		
	}
	// Now check to see if the |look - basis| > epsilon
	// And if so calculate the quaternion
	if (glm::length(ship1->look - GameComponent::basisLook) > epsilon)
	{
		glm::vec3 axis = glm::cross(GameComponent::basisLook, ship1->look);
		axis = glm::normalize(axis);
		float theta = glm::acos(glm::dot(ship1->look, GameComponent::basisLook));
		ship1->orientation = glm::angleAxis(glm::degrees(theta), axis);
	}
	// Apply damping
	ship1->velocity *= 0.99f;
	// Reset the force accumulator
	force = glm::vec3(0,0,0);

	dynamicsWorld->stepSimulation(timeDelta,100);
	
	Game::Update(timeDelta);
}


void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
