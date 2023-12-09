#include <physics/common/Components.h>
#include <cassert>

using namespace physics;

/* Constructor */
Components::Components(MemoryHandler& memoryHandler, 
                       size_t componentByteSize) :
                       mMemoryHandler(memoryHandler),
                       mNumComponents(0),
                       mNumAllocatedComponents(0),
                       mComponentByteSize(componentByteSize),
                       mData(nullptr),
                       mEntityComponentMap(memoryHandler),
                       mSleepingStartIndex(0) {}

/* Destructor */
Components::~Components() {
  /* Erase all allocated components */
  if(mNumAllocatedComponents) {
    for(uint32 i = 0; i < mNumComponents; i++) {
      eraseComponent(i);
    }

    const size_t totalSize = mComponentByteSize * mNumAllocatedComponents;
    mMemoryHandler.free(mData, totalSize);
  }
}

/* Compute index to insert new component */
uint32 Components::computeInsertIndex(bool isSleeping) {
  uint32 insertIndex;

  /* Allocate more components */
  if(mNumComponents == mNumAllocatedComponents) {
    allocate(mNumAllocatedComponents * 2);
  }

  if(isSleeping) {
    /* Sleeping components are added to the end of the array */
    insertIndex = mNumComponents;
  }
  else {
    /* If disabled components exist, move first disabled component to the end of the array and replace with the new component */
    if(mSleepingStartIndex != mNumComponents) {
      moveComponent(mSleepingStartIndex, mNumComponents);
    }

    insertIndex = mSleepingStartIndex;
    mSleepingStartIndex++;
  }
  
  return insertIndex;
}

/* Erase component at the provided index */
void Components::eraseComponent(uint32 index) {
  assert(index < mNumComponents);
}

/* Remove component for given entity */
void Components::removeComponent(Entity entity) {
  assert(mEntityComponentMap.contains(entity));
  uint32 index = mEntityComponentMap[entity];
  assert(index < mNumComponents);
  eraseComponent(index);

  /* Ensure that enabled and disabled components remain adjacent to each other */
  if(index >= mSleepingStartIndex) {
    if(index != mNumComponents - 1) {
      /* Replace last disabled component with the disabled component that is currently being removed */
      moveComponent(mNumComponents - 1, index);
    }
  }
  else {
    if(index != mSleepingStartIndex - 1) {
      /* Replace enabled component currently being removed with the last enabled component */
      moveComponent(mSleepingStartIndex - 1, index);
    }

    if(mSleepingStartIndex != mNumComponents) {
      /* Now replace last enabled component with last disabled component */
      moveComponent(mNumComponents - 1, mSleepingStartIndex - 1);
    }

    mSleepingStartIndex--;
  }

  mNumComponents--;
  assert(mSleepingStartIndex <= mNumComponents);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}

/* Contains component */
bool Components::containsComponent(Entity entity) const {
  return mEntityComponentMap.contains(entity);
}

/* Get component entity index */
uint32 Components::getComponentEntityIndex(Entity entity) {
  assert(containsComponent(entity));
  return mEntityComponentMap[entity];
}

/* Get the number of components */
uint32 Components::getNumComponents() const {
  return mNumComponents;
}

/* Get the number of enabled components */
uint32 Components::getNumEnabledComponents() const {
  return mSleepingStartIndex;
}

/* Query whether an entity is disabled */
bool Components::getIsEntityDisabled(Entity entity) const {
  assert(containsComponent(entity));
  return mEntityComponentMap[entity] >= mSleepingStartIndex;
}

/* Set whether an entity is disabled */
void Components::setIsEntityDisabled(Entity entity, bool isDisabled) {
  const uint32 index = mEntityComponentMap[entity];

  /* Disabled -> Enabled */
  if(!isDisabled && index >= mSleepingStartIndex) {
    assert(mSleepingStartIndex < mNumComponents);

    /* Not the first disabled component */
    if(mSleepingStartIndex != index) {
      swapComponents(index, mSleepingStartIndex);
    }
    
    mSleepingStartIndex++;
  }
  /* Enabled -> Disabled */
  else if(isDisabled && index < mSleepingStartIndex) {
    assert(mSleepingStartIndex > 0);

    /* Not the only enabled component */
    if(index != mSleepingStartIndex - 1) {
      /* Swap with last enabled component */
      swapComponents(index, mSleepingStartIndex - 1);
    }

    mSleepingStartIndex--;
  }

  assert(mSleepingStartIndex <= mNumComponents);
  assert(mNumComponents == static_cast<uint32>(mEntityComponentMap.size()));
}