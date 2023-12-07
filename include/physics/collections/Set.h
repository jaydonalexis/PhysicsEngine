#ifndef PHYSICS_SET_H
#define PHYSICS_SET_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/collections/Pair.h>
#include <physics/collections/DynamicArray.h>
#include <physics/mathematics/Math.h>
#include <cassert>

#define MINIMUM_ALLOCATION_CAPACITY 16

namespace physics {

/* Hash set with a modified red-black tree as its underlying data structure */
template<typename V, class Hash = std::hash<V>, class KeyEqual = std::equal_to<V>>
class Set {

    private:
    /* -- Constants -- */

    /* Load factor */
    static constexpr float LOAD_FACTOR = 0.75;

    /* Poison index */
    static constexpr uint64 POISON_INDEX = INT_MAX;

    /* -- Attributes -- */

    /* Total number of allocated elements */
    uint64 mNumAllocatedElements;

    /* Number of elements */
    uint64 mNumElements;

    /* Hash table size */
    uint64 mHashSize;

    /* Array of buckets */
    uint64* mBuckets;

    /* Array of elements */
    V* mElements;

    /* Index of the next element in the same bucket for a given element */
    uint64* mNextElements;

    /* First free index */
    uint64 mFree;

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    public:
    /* -- Nested Classes -- */

    /* Iterator for the hash set */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to the hash set */
        const Set<V>* mSet;

        /* Index of the current bucket */
        uint64 mCurrentBucket;

        /* Index of the current element */
        uint64 mCurrentElement;

        /* Advance the iterator */
        void advance() {
          assert(mCurrentBucket < mSet->mHashSize);
          assert(mCurrentElement < mSet->mNumAllocatedElements);

          /* Try the next element */
          if(mSet->mNextElements[mCurrentElement] != POISON_INDEX) {
            mCurrentElement = mSet->mNextElements[mCurrentElement];
            return;
          }

          mCurrentElement = 0;
          mCurrentBucket++;

          while(mCurrentBucket < mSet->mHashSize && mSet->mBuckets[mCurrentBucket] == POISON_INDEX) {
            mCurrentBucket++;
          }

          if(mCurrentBucket < mSet->mHashSize) {
            mCurrentElement = mSet->mBuckets[mCurrentBucket];
          }
        }

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(const Set<V>* set, uint64 bucket, uint64 element) : mSet(set), mCurrentBucket(bucket), mCurrentElement(element) {}

        /* Dereference */
        V& operator*() const {
          assert(mCurrentElement < mSet->mNumAllocatedElements);
          assert(mCurrentElement != POISON_INDEX);
          return mSet->mElements[mCurrentElement];
        }

        /* Dereference */
        V* operator->() const {
          assert(mCurrentElement < mSet->mNumAllocatedElements);
          assert(mCurrentElement != POISON_INDEX);
          return &mSet->mElements[mCurrentElement];
        }

        /* Pre increment */
        Iterator& operator++() {
          advance();
          return *this;
        }

        /* Post increment */
        Iterator operator++(int) {
          Iterator temp = *this;
          advance();
          return temp;
        }

        /* Equality operator */
        bool operator==(const Iterator& iter) const {
          return mCurrentBucket == iter.mCurrentBucket &&
                 mCurrentElement == iter.mCurrentElement &&
                 mSet == iter.mSet;
        }

        /* Inequality operator */
        bool operator!=(const Iterator& iter) const {
          return !((*this) == iter);
        }
    };

    /* -- Methods -- */

    /* Find element in the set */
    uint64 findElement(const V& value) const;

    /* -- Methods -- */

    /* Constructor */
    Set(MemoryHandler& memoryHandler, uint64 capacity = 0);

    /* Copy constructor */
    Set(const Set<V>& set);

    /* Destructor */
    ~Set();

    /* Allocate memory for a given number of elements */
    void reserve(uint64 capacity);

    /* Query whether the set contains a particular value */
    bool contains(const V& value) const;

    /* Query whether the set contains an item with the given value and return an iterator to the found item */
    Iterator find(const V& value) const;

    /* Return iterator to the first item in the set */
    Iterator begin() const;

    /* Return iterator to the end of the set */
    Iterator end() const;

    /* Get the number of allocated elements in the set */
    uint64 capacity() const;

    /* Get number of elements in the set */
    uint64 size() const;

    /* Insert an element into the set */
    void insert(const V& value);

    /* Remove an element from the set pointed to by the provided iterator */
    Iterator remove(const Iterator& iter);

    /* Remove an element from the set with the given value */
    Iterator remove(const V& value);

    /* Remove all elements in the set */
    void clear(bool free = false);

    /* Create a dynamic array using the values in the set */
    DynamicArray<V> toArray(MemoryHandler& memoryHandler) const;

    /* Overloaded equality operator */
    bool operator==(const Set<V>& set) const;

    /* Overloaded inequality operator */
    bool operator!=(const Set<V>& set) const;

    /* Overloaded assignment operator */
    Set<V>& operator=(const Set<V>& set);
};

/* Constructor */
template<typename V, class Hash, class KeyEqual>
inline Set<V, Hash, KeyEqual>::Set(MemoryHandler& memoryHandler, uint64 capacity) : 
                               mNumAllocatedElements(0),
                               mNumElements(0),
                               mHashSize(0),
                               mBuckets(nullptr),
                               mElements(nullptr),
                               mNextElements(nullptr),
                               mFree(POISON_INDEX),
                               mMemoryHandler(memoryHandler) {
  if(capacity) {
    reserve(capacity);
  }
}

/* Copy constructor */
template<typename V, class Hash, class KeyEqual>
inline Set<V, Hash, KeyEqual>::Set(const Set<V>& set) : 
                               mNumAllocatedElements(set.mNumAllocatedElements),
                               mNumElements(set.mNumElements),
                               mHashSize(set.mHashSize),
                               mBuckets(nullptr),
                               mElements(nullptr),
                               mNextElements(nullptr),
                               mFree(set.mFree),
                               mMemoryHandler(set.mMemoryHandler) {
  if(mHashSize) {
    /* Memory allocation for buckets and elements */
    mBuckets = static_cast<uint64*>(mMemoryHandler.allocate(mHashSize * sizeof(uint64)));
    mElements = static_cast<V*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(V)));
    mNextElements = static_cast<uint64*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(uint64)));

    /* Copy */
    std::memcpy(mBuckets, set.mBuckets, mHashSize * sizeof(uint64));
    std::memcpy(mNextElements, set.mNextElements, mNumAllocatedElements * sizeof(uint64));

    for(uint64 i = 0; i < mHashSize; i++) {
      uint64 elementIndex = mBuckets[i];

      while(elementIndex != POISON_INDEX) {
        new (mElements + elementIndex) V(set.mElements[elementIndex]);
        elementIndex = mNextElements[elementIndex];
      }
    }
  }
}

/* Destructor */
template<typename V, class Hash, class KeyEqual>
inline Set<V, Hash, KeyEqual>::~Set() {
  clear(true);
}

/* Allocate memory for a given number of elements */
template<typename V, class Hash, class KeyEqual>
inline void Set<V, Hash, KeyEqual>::reserve(uint64 capacity) {
  if(capacity <= mHashSize) {
    return;
  }

if(capacity < MINIMUM_ALLOCATION_CAPACITY) {
    capacity = MINIMUM_ALLOCATION_CAPACITY;
  }

  if(!isPowerOfTwo(capacity)) {
    capacity = nextPowerOfTwo(capacity);
  }

  assert(capacity < POISON_INDEX);
  assert(capacity > mHashSize);
  
  /* Allocate memory for buckets and elements */
  uint64* newBuckets = static_cast<uint64*>(mMemoryHandler.allocate(capacity * sizeof(uint64)));
  const uint64 numAllocatedElements = static_cast<uint64>(capacity * float(LOAD_FACTOR));
  assert(numAllocatedElements);
  V* newElements = static_cast<V*>(mMemoryHandler.allocate(numAllocatedElements * sizeof(V)));
  uint64* newNextElements = static_cast<uint64*>(mMemoryHandler.allocate(numAllocatedElements * sizeof(uint64)));
  assert(newElements);
  assert(newNextElements);

  /* Initialize */
  for(uint64 i = 0; i < capacity; i++) {
    newBuckets[i] = POISON_INDEX;
  }

  if(mNumAllocatedElements) {
    assert(mNextElements);
    std::memcpy(newNextElements, mNextElements, mNumAllocatedElements * sizeof(uint64));
  }

  for(uint64 i = 0; i < mHashSize; i++) {
    uint64 elementIndex = mBuckets[i];

    while(elementIndex != POISON_INDEX) {
      const size_t hashCode = Hash()(mElements[elementIndex]);
      const size_t divider = capacity - 1;
      const uint64 bucketIndex = static_cast<uint64>(hashCode & divider);
      newNextElements[elementIndex] = newBuckets[bucketIndex];
      newBuckets[bucketIndex] = elementIndex;
      new (newElements + elementIndex) V(mElements[elementIndex]);
      mElements[elementIndex].~V();
      elementIndex = mNextElements[elementIndex];
    }
  }

  if(mNumAllocatedElements) {
    mMemoryHandler.free(mBuckets, mHashSize * sizeof(uint64));
    mMemoryHandler.free(mElements, mNumAllocatedElements * sizeof(V));
    mMemoryHandler.free(mNextElements, mNumAllocatedElements * sizeof(uint64));
  }

  for(uint64 i = mNumAllocatedElements; i < numAllocatedElements; i++) {
    if(i == numAllocatedElements - 1) {
      newNextElements[i] = mFree;
      mFree = mNumAllocatedElements;
      continue;
    }

    newNextElements[i] = i + 1;
  }

  mHashSize = capacity;
  mNumAllocatedElements = numAllocatedElements;
  mBuckets = newBuckets;
  mElements = newElements;
  mNextElements = newNextElements;
  assert(mFree != POISON_INDEX);
}

/* Find element in the set */
template<typename V, class Hash, class KeyEqual>
inline uint64 Set<V, Hash, KeyEqual>::findElement(const V& value) const {
  if(mHashSize > 0) {
    const size_t hashCode = Hash()(value);
    const size_t divider = mHashSize - 1;
    const uint64 bucket = static_cast<uint64>(hashCode & divider);
    auto keyEqual = KeyEqual();

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
      if(Hash()(mElements[i]) == hashCode && keyEqual(mElements[i], value)) {
        return i;
      }
    }
  }

  return POISON_INDEX;
}

/* Query whether the set contains a particular value */
template<typename V, class Hash, class KeyEqual>
inline bool Set<V, Hash, KeyEqual>::contains(const V& value) const {
  return findElement(value) != POISON_INDEX;
}

/* Query whether the set contains an item with the given value and return an iterator to the found item */
template<typename V, class Hash, class KeyEqual>
inline typename Set<V, Hash, KeyEqual>::Iterator Set<V, Hash, KeyEqual>::find(const V& value) const {
  uint64 bucket = mHashSize;
  uint64 element = POISON_INDEX;

  if (mHashSize > 0) {

      const size_t hashCode = Hash()(value);
      const size_t divider = mHashSize - 1;
      bucket = static_cast<uint64>(hashCode & divider);
      auto keyEqual = KeyEqual();

      for (uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
          if (Hash()(mElements[i]) == hashCode && keyEqual(mElements[i], value)) {
              element = i;
              break;
          }
      }
  }

  if (element == POISON_INDEX) {
      return end();
  }

  return Iterator(this, bucket, element);
}

/* Return an iterator to the first item in the set */
template<typename V, class Hash, class KeyEqual>
inline typename Set<V, Hash, KeyEqual>::Iterator Set<V, Hash, KeyEqual>::begin() const {
  if(!size()) {
    return end();
  }

  uint64 bucket = 0;

  while(mBuckets[bucket] == POISON_INDEX) {
    bucket++;
  }

  assert(bucket < mHashSize);
  assert(mBuckets[bucket] != POISON_INDEX);
  return Iterator(this, bucket, mBuckets[bucket]);
}

/* Return iterator to the end of the set */
template<typename V, class Hash, class KeyEqual>
inline typename Set<V, Hash, KeyEqual>::Iterator Set<V, Hash, KeyEqual>::end() const {
  return Iterator(this, mHashSize, 0);
}

/* Get the number of allocated elements in the set */
template<typename V, class Hash, class KeyEqual>
inline uint64 Set<V, Hash, KeyEqual>::capacity() const {
  return mHashSize;
}

/* Get number of elements in the set */
template<typename V, class Hash, class KeyEqual>
inline uint64 Set<V, Hash, KeyEqual>::size() const {
  return mNumElements;
}

/* Insert an element into the set */
template<typename V, class Hash, class KeyEqual>
inline void Set<V, Hash, KeyEqual>::insert(const V& value) {
  uint64 bucket = POISON_INDEX;
  const size_t hashCode = Hash()(value);

  if(mHashSize) {
    const size_t divider = mHashSize - 1;
    bucket = static_cast<uint64>(hashCode & divider);
    auto keyEqual = KeyEqual();

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
      if(Hash()(mElements[i]) == hashCode && keyEqual(mElements[i], value)) {
        return;
      }
    }
  }

  uint64 element;

  if(mFree == POISON_INDEX) {
    reserve(mHashSize == 0 ? MINIMUM_ALLOCATION_CAPACITY : mHashSize * 2);
    const size_t divider = mHashSize - 1;
    bucket = static_cast<uint64>(hashCode & divider);
  }

  assert(mNumElements < mNumAllocatedElements);
  assert(mFree != POISON_INDEX);
  element = mFree;
  mFree = mNextElements[element];
  mNumElements++;
  assert(bucket != POISON_INDEX);
  mNextElements[element] = mBuckets[bucket];
  new (mElements + element) V(value);
  mBuckets[bucket] = element;
}

/* Remove an element from the set pointed to by the provided iterator */
template<typename V, class Hash, class KeyEqual>
inline typename Set<V, Hash, KeyEqual>::Iterator Set<V, Hash, KeyEqual>::remove(const Iterator& iter) {
  return remove(*iter);
}

/* Remove an element from the set with the given value */
template<typename V, class Hash, class KeyEqual>
inline typename Set<V, Hash, KeyEqual>::Iterator Set<V, Hash, KeyEqual>::remove(const V& value) {
if(mHashSize) {
    const size_t hashCode = Hash()(value);
    auto keyEqual= KeyEqual();
    const size_t divider = mHashSize - 1;
    const uint64 bucket = static_cast<uint64>(hashCode & divider);
    uint64 prev = POISON_INDEX;

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; prev = i, i = mNextElements[i]) {
      if(Hash()(mElements[i]) == hashCode && keyEqual(mElements[i], value)) {
        if(prev == POISON_INDEX) {
          mBuckets[bucket] = mNextElements[i];
        }
        else {
          mNextElements[prev] = mNextElements[i];
        }

        uint64 nextElement = mNextElements[i];
        uint64 nextBucket = bucket;
        mElements[i].~V();
        mNextElements[i] = mFree;
        mFree = i;
        mNumElements--;

        if(nextElement == POISON_INDEX) {
          nextElement = 0;
          nextBucket++;

          while(nextBucket < mHashSize && mBuckets[nextBucket] == POISON_INDEX) {
            nextBucket++;
          }

          if(nextBucket < mHashSize) {
            nextElement = mBuckets[nextBucket];
          }
        }

        return Iterator(this, nextBucket, nextElement);
      }
    }
  }

  return end();
}

/* Remove all elements in the set */
template<typename V, class Hash, class KeyEqual>
inline void Set<V, Hash, KeyEqual>::clear(bool free) {
  for(uint64 i = 0; i < mHashSize; i++) {
    uint64 element = mBuckets[i];

    while(element != POISON_INDEX) {
      mElements[element].~V();
      uint64 nextElement = mNextElements[element];
      mNextElements[element] = mFree;
      mFree = element;
      element = nextElement;
    }

    mBuckets[i] = POISON_INDEX;
  }

  if(free && mNumAllocatedElements) {
    mMemoryHandler.free(mBuckets, mHashSize * sizeof(uint64));
    mMemoryHandler.free(mElements, mNumAllocatedElements * sizeof(V));
    mMemoryHandler.free(mNextElements, mNumAllocatedElements * sizeof(uint64));
    mBuckets = nullptr;
    mElements = nullptr;
    mNextElements = nullptr;
    mNumAllocatedElements = 0;
    mHashSize = 0;
  }

  mNumElements = 0;
}

/* Create a dynamic array using the values in the set */
template<typename V, class Hash, class KeyEqual>
inline DynamicArray<V> Set<V, Hash, KeyEqual>::toArray(MemoryHandler& memoryHandler) const {
  DynamicArray<V> array(memoryHandler);

  for(auto iter = begin(); iter != end(); ++iter) {
    array.add(*iter);
  }

  return array;
}

/* Overloaded equality operator */
template<typename V, class Hash, class KeyEqual>
inline bool Set<V, Hash, KeyEqual>::operator==(const Set<V>& set) const {
  if(size() != set.size()) {
    return false;
  }

  for(auto iter = begin(); iter != end(); ++iter) {
    if(!set.contains(*iter)) {
      return false;
    }
  }

  return true;
}

/* Overloaded inequality operator */
template<typename V, class Hash, class KeyEqual>
inline bool Set<V, Hash, KeyEqual>::operator!=(const Set<V>& set) const {
  return !((*this) == set);
}

/* Overloaded assignment operator */
template<typename V, class Hash, class KeyEqual>
inline Set<V>& Set<V, Hash, KeyEqual>::operator=(const Set<V>& set) {
if(this != &set) {
    clear(true);
    mNumAllocatedElements = set.mNumAllocatedElements;
    mNumElements = set.mNumElements;
    mHashSize = set.mHashSize;
    mFree = set.mFree;

    if(mHashSize) {
      /* Memory allocation for buckets and elements */
      mBuckets = static_cast<uint64*>(mMemoryHandler.allocate(mHashSize * sizeof(uint64)));
      mElements = static_cast<V*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(V)));
      mNextElements = static_cast<uint64*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(uint64)));

      /* Copy */
      std::memcpy(mBuckets, set.mBuckets, mHashSize * sizeof(uint64));
      std::memcpy(mNextElements, set.mNextElements, mNumAllocatedElements * sizeof(uint64));

      for(uint64 i = 0; i < mHashSize; i++) {
        uint64 elementIndex = mBuckets[i];

        while(elementIndex != POISON_INDEX) {
          new (mElements + elementIndex) V(set.mElements[elementIndex]);
          elementIndex = mNextElements[elementIndex];
        }
      }
    }
  }

  return *this;
}

}

#endif