#include <physics/collision/algorithms/AlgorithmDispatch.h>

using namespace physics;

/* Constructor */
AlgorithmDispatch::AlgorithmDispatch(MemoryHandler& memoryHandler) : mMemoryHandler(memoryHandler) {
  mCircleVCircleAlgorithm = new (memoryHandler.allocate(sizeof(CircleVCircleAlgorithm))) CircleVCircleAlgorithm();
  mCircleVPolygonAlgorithm = new (memoryHandler.allocate(sizeof(CircleVPolygonAlgorithm))) CircleVPolygonAlgorithm();
  mPolygonVPolygonAlgorithm = new (memoryHandler.allocate(sizeof(PolygonVPolygonAlgorithm))) PolygonVPolygonAlgorithm();
  populateCollisionMatrix();
}

/* Destructor */
AlgorithmDispatch::~AlgorithmDispatch() {
  mMemoryHandler.free(mCircleVCircleAlgorithm, sizeof(CircleVCircleAlgorithm));
  mMemoryHandler.free(mCircleVPolygonAlgorithm, sizeof(CircleVPolygonAlgorithm));
  mMemoryHandler.free(mPolygonVPolygonAlgorithm, sizeof(PolygonVPolygonAlgorithm));
}

/* Populate collision matrix */
void AlgorithmDispatch::populateCollisionMatrix() {
  /* Covers every possible shape collision combination (3) */
  for(int i = 0; i < NUM_SHAPE_TYPES; i++) {
    for(int j = 0; j < NUM_SHAPE_TYPES; j++) {
      /* Shapes must be ordered in the collision matrix in the same order than they appear in the enum for shape type */
      if(i <= j) {
        ShapeType firstType = static_cast<ShapeType>(i);
        ShapeType secondType = static_cast<ShapeType>(j);

        if(firstType == ShapeType::Circle && secondType == ShapeType::Circle) {
          mCollisionMatrix[i][j] = CollisionAlgorithmType::CircleVCircle;
        }
        
        if(firstType == ShapeType::Circle && secondType == ShapeType::Polygon) {
          mCollisionMatrix[i][j] = CollisionAlgorithmType::CircleVPolygon;
        }

        if(firstType == ShapeType::Polygon && secondType == ShapeType::Polygon) {
          mCollisionMatrix[i][j] = CollisionAlgorithmType::PolygonVPolygon;
        }
      }
    }
  }
}

/* Get the collision algorithm type */
CollisionAlgorithmType AlgorithmDispatch::getCollisionAlgorithmType(const ShapeType& firstShapeType, const ShapeType& secondShapeType) const {
  uint32 firstShapeIndex = static_cast<uint32>(firstShapeType);
  uint32 secondShapeIndex = static_cast<uint32>(secondShapeType);

  /* Shapes must be ordered in the same order that they appear in the enum for shape type */
  if(firstShapeIndex > secondShapeIndex) {
    return mCollisionMatrix[secondShapeIndex][firstShapeIndex];
  }

  return mCollisionMatrix[firstShapeIndex][secondShapeIndex];
}

/* Get the collision algorithm */
CollisionAlgorithm* AlgorithmDispatch::getCollisionAlgorithm(const CollisionAlgorithmType& algorithmType) const {
  if(algorithmType == CollisionAlgorithmType::CircleVCircle) {
    return mCircleVCircleAlgorithm;
  }

  if(algorithmType == CollisionAlgorithmType::CircleVPolygon) {
    return mCircleVPolygonAlgorithm;
  }

  if(algorithmType == CollisionAlgorithmType::PolygonVPolygon) {
    return mPolygonVPolygonAlgorithm;
  }

  return nullptr;
}