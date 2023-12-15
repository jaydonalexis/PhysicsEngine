#include <physics/common/Factory.h>

using namespace physics;

Logger* Factory::mLogger = nullptr;

/* Constructor */
Factory::Factory(MemoryHandler* primaryMemoryHandler) :
                 mMemoryStrategy(primaryMemoryHandler),
                 mWorlds(mMemoryStrategy.getFreeListMemoryHandler()),
                 mPolygonShapes(mMemoryStrategy.getFreeListMemoryHandler()),
                 mBoxShapes(mMemoryStrategy.getFreeListMemoryHandler()),
                 mCircleShapes(mMemoryStrategy.getFreeListMemoryHandler()) {}

/* Destructor */
Factory::~Factory() {
  destroy();
}

/* Destroy all allocated objects */
void Factory::destroy() {
  /* Destory world */
  for(auto iter = mWorlds.begin(); iter != mWorlds.end(); ++iter) {
    deleteWorld(*iter);
  }

  mWorlds.clear();

  /* Destroy polygons */
  for(auto iter = mPolygonShapes.begin(); iter != mPolygonShapes.end(); ++iter) {
    deletePolygon(*iter);
  }

  mPolygonShapes.clear();

  /* Destroy boxes */
  for(auto iter = mBoxShapes.begin(); iter != mBoxShapes.end(); ++iter) {
    deleteBox(*iter);
  }

  mBoxShapes.clear();

  /* Destroy circles */
  for(auto iter = mCircleShapes.begin(); iter != mCircleShapes.end(); ++iter) {
    deleteCircle(*iter);
  }

  mCircleShapes.clear();
}

/* Delete world */
void Factory::deleteWorld(World* world) {
  world->~World();
  mMemoryStrategy.free(MemoryStrategy::HandlerType::FreeList, world, sizeof(World));
}

/* Delete polygon shape */
void Factory::deletePolygon(PolygonShape* polygon) {
  polygon->~PolygonShape();
  mMemoryStrategy.free(MemoryStrategy::HandlerType::ObjectPool, polygon, sizeof(PolygonShape));
}

/* Delete box shape*/
void Factory::deleteBox(BoxShape* box) {
  box->~BoxShape();
  mMemoryStrategy.free(MemoryStrategy::HandlerType::ObjectPool, box, sizeof(BoxShape));
}

/* Delete circle shape */
void Factory::deleteCircle(CircleShape* circle) {
  circle->~CircleShape();
  mMemoryStrategy.free(MemoryStrategy::HandlerType::ObjectPool, circle, sizeof(CircleShape));
}

/* Create world */
World* Factory::createWorld(const World::Settings& settings) {
  World* world = new (mMemoryStrategy.allocate(MemoryStrategy::HandlerType::FreeList, sizeof(World))) World(mMemoryStrategy, *this, settings);
  mWorlds.insert(world);
  return world;
}

/* Destory world */
void Factory::destroyWorld(World* world) {
  deleteWorld(world);
  mWorlds.remove(world);
}

/* Create polygon shape */
PolygonShape* Factory::createPolygon(const Vector2* points, uint32 numPoints) {
  PolygonShape* polygon = new (mMemoryStrategy.allocate(MemoryStrategy::HandlerType::ObjectPool, sizeof(PolygonShape))) PolygonShape(points, numPoints, mMemoryStrategy.getFreeListMemoryHandler());
  mPolygonShapes.insert(polygon);
  return polygon;
}

/* Destroy polygon shape */
void Factory::destroyPolygon(PolygonShape* polygon) {
  deletePolygon(polygon);
  mPolygonShapes.remove(polygon);
}

/* Create box shape */
BoxShape* Factory::createBox(const float hx, const float hy) {
  BoxShape* box = new (mMemoryStrategy.allocate(MemoryStrategy::HandlerType::ObjectPool, sizeof(BoxShape))) BoxShape(hx, hy, mMemoryStrategy.getFreeListMemoryHandler());
  mBoxShapes.insert(box);
  return box;
}

/* Destroy box shape */
void Factory::destroyBox(BoxShape* box) {
  deleteBox(box);
  mBoxShapes.remove(box);
}

/* Create circle shape */
CircleShape* Factory::createCircle(const float radius) {
  CircleShape* circle = new (mMemoryStrategy.allocate(MemoryStrategy::HandlerType::ObjectPool, sizeof(CircleShape))) CircleShape(radius, mMemoryStrategy.getFreeListMemoryHandler());
  mCircleShapes.insert(circle);
  return circle;
}

/* Destroy circle shape */
void Factory::destroyCircle(CircleShape* circle) {
  deleteCircle(circle);
  mCircleShapes.remove(circle);
}

/* Get logger */
Logger* Factory::getLogger() {
  return mLogger;
}

/* Set logger */
void Factory::setLogger(Logger* logger) {
  mLogger = logger;
}