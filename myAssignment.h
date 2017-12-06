#include "Game.h" 
#include "PhysicsController.h"
#include "PhysicsFactory.h"

namespace BGE
{
	class myAssignment :
		public Game
	{
	private:

	public:
		myAssignment(void);
		~myAssignment(void);
		bool Initialise();
		void Update();
		void Cleanup();
		void CreateWall();
		shared_ptr<PhysicsController> spider;
		shared_ptr<PhysicsController> CreateSpider(glm::vec3 position, float scale = 5);
	};
}