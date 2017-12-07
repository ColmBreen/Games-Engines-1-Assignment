#include "myAssignment.h" 
#include "Utils.h"

#include "PhysicsGame1.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "PhysicsFactory.h"
#include "Game.h" 
#include "Model.h"
#include "dirent.h"
#include "Capsule.h" 

using namespace BGE;

myAssignment::myAssignment(void)
{
}

myAssignment::~myAssignment(void)
{
}

bool myAssignment::Initialise()
{
	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	dynamicsWorld->setGravity(btVector3(0, -30, 0));
	shared_ptr<PhysicsController> spider = CreateSpider(glm::vec3(0, 8, 0), 5);
	if (!Game::Initialise()) {
		return false;
	}

	return true;
}

void myAssignment::Update()
{
	Game::Update(1);
}

void myAssignment::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> myAssignment::CreateSpider(glm::vec3 position, float scale)
{
	float width = 16;
	float height = 3;
	float length = 6;
	float legRadius = 3;
	float legWidth = 3;
	float legOffset = 3;

	shared_ptr<PhysicsController> body = physicsFactory->CreateBox(width, height, length, position, glm::quat());

	shared_ptr<PhysicsController> leg;
	shared_ptr<PhysicsController> attach;
	shared_ptr<PhysicsController> head;
	shared_ptr<PhysicsController> neck;
	
	glm::quat q = glm::angleAxis(0.0f, glm::vec3(0, 0, 0));
	glm::quat p = glm::angleAxis(270.0f, glm::vec3(1, 0, 0));

	glm::vec3 offset;
	glm::vec3 neckOffset;
	btHingeConstraint * hinge;
	
	offset = glm::vec3(0, 6.5f, 0);
	neckOffset = (position + glm::vec3(0, 6.5f, 0));
	neck = physicsFactory->CreateCylinder(1.5f, 8.0f, position + offset, p);
	hinge = new btHingeConstraint(*body->rigidBody, *neck->rigidBody, btVector3(0, 3.25f, 0), btVector3(0, -3.25f, 0), btVector3(0, 1, 0), btVector3(0, 1, 0), true);
	dynamicsWorld->addConstraint(hinge);
	
	offset = glm::vec3(0, 5.0f, 0);
	head = physicsFactory->CreateSphere(1.5f, neckOffset + offset, p);
	btPoint2PointConstraint * neckConstraint = new btPoint2PointConstraint(*neck->rigidBody, *head->rigidBody, btVector3(0, 2.5f, 0), btVector3(0, -2.5f, 0));
	dynamicsWorld->addConstraint(neckConstraint);
	

	//Leg stumps
	offset = glm::vec3(-5, 0, 7);
	attach = physicsFactory->CreateBox(7.5, 3, 3, position + offset, p);
	hinge = new btHingeConstraint(*body->rigidBody, *attach->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(5, 0, -4.5);
	attach = physicsFactory->CreateBox(3, 3, 3, position + offset, p);
	hinge = new btHingeConstraint(*body->rigidBody, *attach->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(5, 0, 4.5);
	attach = physicsFactory->CreateBox(3, 3, 3, position + offset, p);
	hinge = new btHingeConstraint(*body->rigidBody, *attach->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge);

	offset = glm::vec3(-5, 0, -7);
	attach = physicsFactory->CreateBox(7.5, 3, 3, position + offset, p);
	hinge = new btHingeConstraint(*body->rigidBody, *attach->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(1, 0, 0), true);
	dynamicsWorld->addConstraint(hinge);

	//Legs
	offset = glm::vec3(-5, 0, 12.5);
	leg = physicsFactory->CreateBox(3, 10, 3, position + offset, q);
	hinge = new btHingeConstraint(*body->rigidBody, *leg->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(5, 0, 0), true);
	hinge->enableAngularMotor(true, 30, 30);
	dynamicsWorld->addConstraint(hinge);
	
	offset = glm::vec3(5, 0, -7.6);
	leg = physicsFactory->CreateBox(3, 3, 10, glm::vec3(position.x + (width / 2) - legRadius, position.y, position.z - (length / 2) - legWidth), q);
	hinge = new btHingeConstraint(*body->rigidBody, *leg->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(5, 0, 0), true);
	hinge->enableAngularMotor(true, 30, 30);
	dynamicsWorld->addConstraint(hinge);
	
	offset = glm::vec3(-5, 0, -12.5);
	leg = physicsFactory->CreateBox(3, 3, 10, position + offset, q);
	hinge = new btHingeConstraint(*body->rigidBody, *leg->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(5, 0, 0), true);
	hinge->enableAngularMotor(true, 30, 30);
	dynamicsWorld->addConstraint(hinge);
	
	offset = glm::vec3(5, 0, 7.6);
	leg = physicsFactory->CreateBox(3, 3, 10, position + offset, q);
	hinge = new btHingeConstraint(*body->rigidBody, *leg->rigidBody, GLToBtVector(offset), btVector3(0, 0, 0), btVector3(0, 0, 1), btVector3(5, 0, 0), true);
	hinge->enableAngularMotor(true, 30, 30);
	dynamicsWorld->addConstraint(hinge);
	
	return body;
}