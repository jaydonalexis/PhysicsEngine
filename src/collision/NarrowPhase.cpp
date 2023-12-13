#include <physics/collision/NarrowPhase.h>

using namespace physics;

/* Constructor */
NarrowPhase::NarrowPhase(OverlapPairs& overlapPairs, MemoryHandler& memoryHandler) : mOverlapPairs(overlapPairs), mMemoryHandler(memoryHandler), entries(memoryHandler) {}

/* Destructor */
NarrowPhase::~NarrowPhase() {
  clear();
}

/* Add narrow phase entry */
void NarrowPhase::addEntry(uint64 overlapPairIdentifier,
                           Entity firstColliderEntity,
                           Entity secondColliderEntity,
                           Shape* firstShape,
                           Shape* secondShape,
                           const Transform& firstShapeTransform,
                           const Transform& secondShapeTransform,
                           CollisionAlgorithm* algorithm) {
  ShapeType firstType = firstShape->getType();
  ShapeType secondType = secondShape->getType();

  /* Order entry such that the shapes are reversed relative to their order within the shape type enum */
  entries.emplace(overlapPairIdentifier,
                  firstType <= secondType ? secondColliderEntity : firstColliderEntity,
                  firstType <= secondType ? firstColliderEntity : secondColliderEntity,
                  firstType <= secondType ? secondShape : firstShape,
                  firstType <= secondType ? firstShape : secondShape,
                  firstType <= secondType ? secondShapeTransform : firstShapeTransform,
                  firstType <= secondType ? firstShapeTransform : secondShapeTransform,
                  algorithm);
}

/* Initialize using cached capacity */
void NarrowPhase::reserve() {
  entries.reserve(mCachedCapacity);
}

/* Clear all entries */
void NarrowPhase::clear() {
  /* Cached capacity to reserve memory for the next frame */
  mCachedCapacity = static_cast<uint32>(entries.capacity());
  entries.clear(true);
}