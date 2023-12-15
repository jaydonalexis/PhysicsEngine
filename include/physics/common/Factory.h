#ifndef PHYSICS_FACTORY_H
#define PHYSICS_FACTORY_H

#include <physics/common/World.h>
#include <physics/memory/MemoryStrategy.h>
#include <physics/collision/PolygonShape.h>
#include <physics/collision/BoxShape.h>
#include <physics/collision/CircleShape.h>
#include <physics/common/Logger.h>

#define LOG(message) if (physics::Factory::getLogger() != nullptr) Factory::getLogger()->log(message);

namespace physics {

class Factory {

  private:
    /* -- Attributes -- */

    /* Memory strategy */
    MemoryStrategy mMemoryStrategy;

    /* Physics world */
    Set<World*> mWorlds;

    /* Polygon shapes */
    Set<PolygonShape*> mPolygonShapes;

    /* Box shapes */
    Set<BoxShape*> mBoxShapes;

    /* Circle shapes */
    Set<CircleShape*> mCircleShapes;

    /* Logger */
    static Logger* mLogger;

    /* -- Methods -- */

    /* Initialize */
    void initialize();

    /* Destroy all allocated objects */
    void destroy();

    /* Delete world */
    void deleteWorld(World* world);

    /* Delete polygon shape */
    void deletePolygon(PolygonShape* polygon);

    /* Delete box shape */
    void deleteBox(BoxShape* box);

    /* Delete circle shape */
    void deleteCircle(CircleShape* circle);

  public:
    /* -- Methods -- */

    /* Constructor */
    Factory(MemoryHandler* primaryMemoryHandler = nullptr);

    /* Destructor */
    ~Factory();

    /* Create world */
    World* createWorld(const World::Settings& settings = World::Settings());

    /* Destroy world */
    void destroyWorld(World* world);

    /* Create polygon shape */
    PolygonShape* createPolygon(const Vector2* points, uint32 numPoints);

    /* Destroy polygon shape */
    void destroyPolygon(PolygonShape* polygon);

    /* Create box shape */
    BoxShape* createBox(const float hx, const float hy);

    /* Destroy box shape */
    void destroyBox(BoxShape* box);

    /* Create circle shape */
    CircleShape* createCircle(const float radius);

    /* Destroy circle shape */
    void destroyCircle(CircleShape* circle);

    /* Get logger */
    static Logger* getLogger();

    /* Set logger */
    static void setLogger(Logger* logger);

    /* Create */

    /* -- Friends -- */

    friend class World;
};

}

#endif