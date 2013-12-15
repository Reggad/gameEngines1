#pragma once
#include "Game.h"
#include "GameComponent.h"
#include "FountainEffect.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>

namespace BGE
{
	class Assignment:
		public Game
	{
		private:
		btBroadphaseInterface* broadphase;
 
		// Set up the collision configuration and dispatcher
		btDefaultCollisionConfiguration * collisionConfiguration;
		btCollisionDispatcher * dispatcher;
 
		// The actual physics solver
		btSequentialImpulseConstraintSolver * solver;
		
		public:
			Assignment(void);
			~Assignment(void);
			bool Initialise();
			void Update(float timeDelta);
			void Cleanup();
			void CreateWall();

			std::shared_ptr<GameComponent> ship1;
			shared_ptr<GameComponent> ship2;
			shared_ptr<FountainEffect> trail;
			glm::vec3 force;
			float mass;
			float elapsed;
			bool slerping;
			glm::quat fromQuaternion;
			glm::quat toQuaternion;
			float t;
		
			// The world.
			std::shared_ptr<PhysicsFactory> physicsFactory;
			btDiscreteDynamicsWorld * dynamicsWorld;


	};

}