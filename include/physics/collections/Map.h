#ifndef PHYSICS_MAP_H
#define PHYSICS_MAP_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/collections/Pair.h>
#include <physics/mathematics/Math.h>
#include <cassert>

#define MINIMUM_ALLOCATION_CAPACITY 16

namespace physics {

/* Hash map with a modified red-black tree as its underlying data structure */
template<typename K, typename V, class Hash = std::hash<K>, class KeyEqual = std::equal_to<K>>
class Map {

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
    Pair<K, V>* mElements;

    /* Index of the next element in the same bucket for a given element */
    uint64* mNextElements;

    /* First free index */
    uint64 mFree;

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* -- Methods -- */

    /* Find element in the map */
    uint64 findElement(const K& key) const;

  public:
    /* -- Nested Classes -- */

    /* Iterator for the hash map */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to the hash map */
        const Map<K, V>* mMap;

        /* Index of the current bucket */
        uint64 mCurrentBucket;

        /* Index of the current element */
        uint64 mCurrentElement;

        /* Advance the iterator */
        void advance() {
          assert(mCurrentBucket < mMap->mHashSize);
          assert(mCurrentElement < mMap->mNumAllocatedElements);

          /* Try the next element */
          if(mMap->mNextElements[mCurrentElement] != POISON_INDEX) {
            mCurrentElement = mMap->mNextElements[mCurrentElement];
            return;
          }

          mCurrentElement = 0;
          mCurrentBucket++;

          while(mCurrentBucket < mMap->mHashSize && mMap->mBuckets[mCurrentBucket] == POISON_INDEX) {
            mCurrentBucket++;
          }

          if(mCurrentBucket < mMap->mHashSize) {
            mCurrentElement = mMap->mBuckets[mCurrentBucket];
          }
        }

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(const Map<K, V>* map, uint64 bucket, uint64 element) : mMap(map), mCurrentBucket(bucket), mCurrentElement(element) {}

        /* Dereference */
        Pair<K, V>& operator*() const {
          assert(mCurrentElement < mMap->mNumAllocatedElements);
          assert(mCurrentElement != POISON_INDEX);
          return mMap->mElements[mCurrentElement];
        }

        /* Dereference */
        Pair<K, V>* operator->() const {
          assert(mCurrentElement < mMap->mNumAllocatedElements);
          assert(mCurrentElement != POISON_INDEX);
          return &mMap->mElements[mCurrentElement];
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
                 mMap == iter.mMap;
        }

        /* Inequality operator */
        bool operator!=(const Iterator& iter) const {
          return !((*this) == iter);
        }
    };

    /* -- Methods -- */

    /* Constructor */
    Map(MemoryHandler& memoryHandler, uint64 capacity = 0);

    /* Copy constructor */
    Map(const Map<K, V>& map);

    /* Destructor */
    ~Map();

    /* Allocate memory for a given number of elements */
    void reserve(uint64 capacity);

    /* Query whether the map contains a particular key */
    bool contains(const K& key) const;

    /* Query whether the map contains an item with the given key and return an iterator to the found item */
    Iterator find(const K& key) const;

    /* Return iterator to the first item in the map */
    Iterator begin() const;

    /* Return iterator to the end of the map */
    Iterator end() const;

    /* Get the number of allocated elements in the map */
    uint64 capacity() const;

    /* Get number of elements in the map */
    uint64 size() const;

    /* Insert an element into the map */
    void insert(const Pair<K, V>& keyValue);

    /* Remove an element from the map pointed to by the provided iterator */
    Iterator remove(const Iterator& iter);

    /* Remove an element from the map with the given key */
    Iterator remove(const K& key);

    /* Remove all elements in the map */
    void clear(bool free = false);

    /* Overloaded value access operator */
    V& operator[](const K& key);

    /* Overloaded value access operator */
    const V& operator[](const K& key) const;

    /* Overloaded equality operator */
    bool operator==(const Map<K, V>& map) const;

    /* Overloaded inequality operator */
    bool operator!=(const Map<K, V>& map) const;

    /* Overloaded assignemt operator */
    Map<K, V>& operator=(const Map<K, V>& map);

    /* -- Friends -- */

    friend class Iterator;
};

/* Constructor */
template<typename K, typename V, class Hash, class KeyEqual>
inline Map<K, V, Hash, KeyEqual>::Map(MemoryHandler& memoryHandler, uint64 capacity) : 
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
template<typename K, typename V, class Hash, class KeyEqual>
inline Map<K, V, Hash, KeyEqual>::Map(const Map<K, V>& map) :
                                  mNumAllocatedElements(map.mNumAllocatedElements),
                                  mNumElements(map.mNumElements),
                                  mHashSize(map.mHashSize),
                                  mBuckets(nullptr),
                                  mElements(nullptr),
                                  mNextElements(nullptr),
                                  mFree(map.mFree),
                                  mMemoryHandler(map.mMemoryHandler) {
  if(mHashSize) {
    /* Memory allocation for buckets and elements */
    mBuckets = static_cast<uint64*>(mMemoryHandler.allocate(mHashSize * sizeof(uint64)));
    mElements = static_cast<Pair<K, V>*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(Pair<K, V>)));
    mNextElements = static_cast<uint64*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(uint64)));

    /* Copy */
    std::memcpy(mBuckets, map.mBuckets, mHashSize * sizeof(uint64));
    std::memcpy(mNextElements, map.mNextElements, mNumAllocatedElements * sizeof(uint64));

    for(uint64 i = 0; i < mHashSize; i++) {
      uint64 elementIndex = mBuckets[i];

      while(elementIndex != POISON_INDEX) {
        new (mElements + elementIndex) Pair<K, V>(map.mElements[elementIndex]);
        elementIndex = mNextElements[elementIndex];
      }
    }
  }
}

/* Destructor */
template<typename K, typename V, class Hash, class KeyEqual>
inline Map<K, V, Hash, KeyEqual>::~Map() {
  clear(true);
}

/* Allocate memory for a given number of elements */
template<typename K, typename V, class Hash, class KeyEqual>
inline void Map<K, V, Hash, KeyEqual>::reserve(uint64 capacity) {
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
  Pair<K, V>* newElements = static_cast<Pair<K, V>*>(mMemoryHandler.allocate(numAllocatedElements * sizeof(Pair<K, V>)));
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
      const size_t hashCode = Hash()(mElements[elementIndex].first);
      const size_t divider = capacity - 1;
      const uint64 bucketIndex = static_cast<uint64>(hashCode & divider);
      newNextElements[elementIndex] = newBuckets[bucketIndex];
      newBuckets[bucketIndex] = elementIndex;
      new (newElements + elementIndex) Pair<K, V>(mElements[elementIndex]);
      mElements[elementIndex].~Pair<K, V>();
      elementIndex = mNextElements[elementIndex];
    }
  }

  if(mNumAllocatedElements) {
    mMemoryHandler.free(mBuckets, mHashSize * sizeof(uint64));
    mMemoryHandler.free(mElements, mNumAllocatedElements * sizeof(Pair<K, V>));
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

/* Find element in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline uint64 Map<K, V, Hash, KeyEqual>::findElement(const K& key) const {
  if(mHashSize > 0) {
    const size_t hashCode = Hash()(key);
    const size_t divider = mHashSize - 1;
    const uint64 bucket = static_cast<uint64>(hashCode & divider);
    auto keyEqual = KeyEqual();

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
      if(Hash()(mElements[i].first) == hashCode && keyEqual(mElements[i].first, key)) {
        return i;
      }
    }
  }

  return POISON_INDEX;
}

/* Query whether the map contains a particular key */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool Map<K, V, Hash, KeyEqual>::contains(const K& key) const {
  return findElement(key) != POISON_INDEX;
}

/* Query whether the map contains an item with the given key and return an iterator to the found item */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename Map<K, V, Hash, KeyEqual>::Iterator Map<K, V, Hash, KeyEqual>::find(const K& key) const {
  uint64 bucket = mHashSize;
  uint64 element = POISON_INDEX;

  if (mHashSize > 0) {

      const size_t hashCode = Hash()(key);
      const size_t divider = mHashSize - 1;
      bucket = static_cast<uint64>(hashCode & divider);
      auto keyEqual = KeyEqual();

      for (uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
          if (Hash()(mElements[i].first) == hashCode && keyEqual(mElements[i].first, key)) {
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

/* Return iterator to the first item in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename Map<K, V, Hash, KeyEqual>::Iterator Map<K, V, Hash, KeyEqual>::begin() const {
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

/* Return iterator to the end of the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename Map<K, V, Hash, KeyEqual>::Iterator Map<K, V, Hash, KeyEqual>::end() const {
  return Iterator(this, mHashSize, 0);
}

/* Get the number of allocated elements in the map */
template<typename K, typename V, class Hash, class KeyEqual>
uint64 Map<K, V, Hash, KeyEqual>::capacity() const {
  return mHashSize;
}

/* Get number of elements in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline uint64 Map<K, V, Hash, KeyEqual>::size() const {
  return mNumElements;
}

/* Insert an element into the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline void Map<K, V, Hash, KeyEqual>::insert(const Pair<K, V>& keyValue) {
  uint64 bucket = POISON_INDEX;
  const size_t hashCode = Hash()(keyValue.first);

  if(mHashSize) {
    const size_t divider = mHashSize - 1;
    bucket = static_cast<uint64>(hashCode & divider);
    auto keyEqual = KeyEqual();

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; i = mNextElements[i]) {
      if(Hash()(mElements[i].first) == hashCode && keyEqual(mElements[i].first, keyValue.first)) {
        mElements[i].~Pair<K, V>();
        new (mElements + i) Pair<K, V>(keyValue);
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
  new (mElements + element) Pair<K, V>(keyValue);
  mBuckets[bucket] = element;
}

/* Remove an element from the map pointed to by the provided iterator */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename Map<K, V, Hash, KeyEqual>::Iterator Map<K, V, Hash, KeyEqual>::remove(const Iterator& iter) {
  const K& key = iter->first;
  return remove(key);
}

/* Remove an element from the map with the given key */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename Map<K, V, Hash, KeyEqual>::Iterator Map<K, V, Hash, KeyEqual>::remove(const K& key) {
  if(mHashSize) {
    const size_t hashCode = Hash()(key);
    auto keyEqual= KeyEqual();
    const size_t divider = mHashSize - 1;
    const uint64 bucket = static_cast<uint64>(hashCode & divider);
    uint64 prev = POISON_INDEX;

    for(uint64 i = mBuckets[bucket]; i != POISON_INDEX; prev = i, i = mNextElements[i]) {
      if(Hash()(mElements[i].first) == hashCode && keyEqual(mElements[i].first, key)) {
        if(prev == POISON_INDEX) {
          mBuckets[bucket] = mNextElements[i];
        }
        else {
          mNextElements[prev] = mNextElements[i];
        }

        uint64 nextElement = mNextElements[i];
        uint64 nextBucket = bucket;
        mElements[i].~Pair<K, V>();
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

/* Remove all elements in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline void Map<K, V, Hash, KeyEqual>::clear(bool free) {
  for(uint64 i = 0; i < mHashSize; i++) {
    uint64 element = mBuckets[i];

    while(element != POISON_INDEX) {
      mElements[element].~Pair<K, V>();
      uint64 nextElement = mNextElements[element];
      mNextElements[element] = mFree;
      mFree = element;
      element = nextElement;
    }

    mBuckets[i] = POISON_INDEX;
  }

  if(free && mNumAllocatedElements) {
    mMemoryHandler.free(mBuckets, mHashSize * sizeof(uint64));
    mMemoryHandler.free(mElements, mNumAllocatedElements * sizeof(Pair<K, V>));
    mMemoryHandler.free(mNextElements, mNumAllocatedElements * sizeof(uint64));
    mBuckets = nullptr;
    mElements = nullptr;
    mNextElements = nullptr;
    mNumAllocatedElements = 0;
    mHashSize = 0;
  }

  mNumElements = 0;
}

/* Overloaded value access operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline V& Map<K, V, Hash, KeyEqual>::operator[](const K& key) {
  const uint64 element = findElement(key);

  if(element == POISON_INDEX) {
    throw std::runtime_error("Invalid key");
  }

  return mElements[element].second;
}

/* Overloaded value access operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline const V& Map<K, V, Hash, KeyEqual>::operator[](const K& key) const {
  const uint64 element = findElement(key);

  if(element == POISON_INDEX) {
    throw std::runtime_error("Invalid key");
  }

  return mElements[element].second;
}

/* Overloaded equality operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool Map<K, V, Hash, KeyEqual>::operator==(const Map<K, V>& map) const {
  if(size() != map.size()) {
    return false;
  }

  for(auto iter1 = begin(); iter1 != end(); ++iter1) {
    auto iter2 = map.find(iter1->first);

    if(iter2 == map.end() || iter2->second != iter1->second) {
      return false;
    }
  }

  for(auto iter1 = map.begin(); iter1 != map.end(); ++iter1) {
    auto iter2 = find(iter1->first);

    if(iter2 == end() || iter2->second != iter1->second) {
      return false;
    }
  }

  return true;
}

/* Overloaded inequality operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool Map<K, V, Hash, KeyEqual>::operator!=(const Map<K, V>& map) const {
  return !((*this) == map);
}

/* Overloaded assignment operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline Map<K, V>& Map<K, V, Hash, KeyEqual>::operator=(const Map<K, V>& map) {
  if(this != &map) {
    clear(true);
    mNumAllocatedElements = map.mNumAllocatedElements;
    mNumElements = map.mNumElements;
    mHashSize = map.mHashSize;
    mFree = map.mFree;

    if(mHashSize) {
      /* Memory allocation for buckets and elements */
      mBuckets = static_cast<uint64*>(mMemoryHandler.allocate(mHashSize * sizeof(uint64)));
      mElements = static_cast<Pair<K, V>*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(Pair<K, V>)));
      mNextElements = static_cast<uint64*>(mMemoryHandler.allocate(mNumAllocatedElements * sizeof(uint64)));

      /* Copy */
      std::memcpy(mBuckets, map.mBuckets, mHashSize * sizeof(uint64));
      std::memcpy(mNextElements, map.mNextElements, mNumAllocatedElements * sizeof(uint64));

      for(uint64 i = 0; i < mHashSize; i++) {
        uint64 elementIndex = mBuckets[i];

        while(elementIndex != POISON_INDEX) {
          new (mElements + elementIndex) Pair<K, V>(map.mElements[elementIndex]);
          elementIndex = mNextElements[elementIndex];
        }
      }
    }
  }

  return *this;
}

}

#endif