#ifndef PHYSICS_DYNAMIC_ARRAY_H
#define PHYSICS_DYNAMIC_ARRAY_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <cassert>
#include <cstring>
#include <iterator>
#include <memory>

namespace physics {

template<typename T>
class DynamicArray {

  private:
    /* -- Attributes -- */

    /* Array elements */
    T* mValues;
    
    /* Number of elements in the array */
    uint64 mSize;

    /* Number of allocated elements in the array */
    uint64 mCapacity;

    /* Memory Handler */
    MemoryHandler& mMemoryHandler;

  public:
    /* -- Traits -- */

    using iterator = T*;

    /* -- Constructors -- */

    DynamicArray(MemoryHandler& memoryHandler, uint64 capacity = 0);
    DynamicArray(const DynamicArray<T>& array);

    /* -- Destructor -- */

    ~DynamicArray();

    /* -- Element Access -- */

    T& front();
    T& back();
    T& at(uint64 i);
    iterator find(const T& element);

    /* -- Iterators -- */

    iterator begin();
    iterator end();

    /* -- Capacity -- */

    uint64 size() const;
    uint64 capacity() const;
    bool empty() const;
    void reserve(uint64 size);

    /* -- Operators -- */

    T& operator[](const uint64 i);
    const T& operator[](const uint64 i) const;
    bool operator==(const DynamicArray<T>& array) const;
    bool operator!=(const DynamicArray<T>& array) const;
    DynamicArray<T>& operator=(const DynamicArray<T>& array);

    /* -- Modifiers -- */

    template<typename... Ts>
    void emplace(Ts&&... args);
    void add(const T& element);
    void add(const DynamicArray<T>& array, uint64 start = 0);
    iterator remove(const T& element);
    iterator remove(const iterator& iter);
    iterator removeAt(uint64 i);
    void grow(uint64 numElements);
    void clear(bool free = false);
};

/* Template functions are implicitly inlined */

/* -- Constructors -- */

template<typename T>
inline DynamicArray<T>::DynamicArray(MemoryHandler& memoryHandler, uint64 capacity) : mValues(nullptr), mSize(0), mCapacity(0), mMemoryHandler(memoryHandler) {
  if(capacity > 0) {
    reserve(capacity);
  }
}

template<typename T>
inline DynamicArray<T>::DynamicArray(const DynamicArray<T>& array) : mValues(nullptr), mSize(0), mCapacity(0), mMemoryHandler(array.mMemoryHandler) {
  if(array.mCapacity > 0) {
    reserve(array.mCapacity);
  }

  add(array);
}

/* -- Destructor -- */

template<typename T>
inline DynamicArray<T>::~DynamicArray() {
  if(mCapacity > 0) {
    clear(true);
  }
}

/* -- Element Access -- */

template<typename T>
inline T& DynamicArray<T>::front() {
  assert(mSize > 0);
  return mValues[0];
}

template<typename T>
inline T& DynamicArray<T>::back() {
  assert(mSize > 0);
  return mValues[mSize - 1];
}

template<typename T>
inline T& DynamicArray<T>::at(uint64 i) {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::find(const T& element) {
  for(uint64 i = 0; i < mSize; i++) {
    if(element == mValues[i]) {
      return &mValues[i];
    }
  }
}

/* -- Iterators -- */

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::begin() {
  return mValues;
}

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::end() {
  return mValues + mSize;
}

/* -- Capacity -- */

template<typename T>
inline uint64 DynamicArray<T>::size() const {
  return mSize;
}

template<typename T>
inline uint64 DynamicArray<T>::capacity() const {
  return mCapacity;
}

template<typename T>
inline bool DynamicArray<T>::empty() const {
  return mSize == 0;
}

template<typename T>
inline void DynamicArray<T>::reserve(uint64 size) {
  if(size <= mCapacity) {
    return;
  }

  void* raw = mMemoryHandler.allocate(size * sizeof(T));
  T* cast = static_cast<T*>(raw);

  if(mValues) {
    if(mSize) {
      std::uninitialized_copy(mValues, mValues + mSize, cast);

      for(uint64 i = 0; i < mSize; i++) {
        mValues[i].~T();
      }
    }

    mMemoryHandler.free(mValues, mCapacity * sizeof(T));
  }

  mValues = cast;
  assert(mValues);
  mCapacity = size;
}

/* -- Operators -- */

template<typename T>
inline T& DynamicArray<T>::operator[](const uint64 i) {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

template<typename T>
inline const T& DynamicArray<T>::operator[](const uint64 i) const {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

template<typename T>
inline bool DynamicArray<T>::operator==(const DynamicArray<T>& array) const {
  if(mSize != array.mSize) {
    return false;
  }

  for(uint64 i = 0; i < mSize; i++) {
    if(mValues[i] != array[i]) {
      return false;
    }
  }

  return true;
}

template<typename T>
inline bool DynamicArray<T>::operator!=(const DynamicArray<T>& array) const {
  return !((*this) == array);2
}

template<typename T>
inline DynamicArray<T>& DynamicArray<T>::operator=(const DynamicArray<T>& array) {
  if(this != &array) {
    clear();
    add(array);
  }

  return *this;
}

/* -- Modifiers -- */

template<typename T>
template<typename...Ts>
inline void DynamicArray<T>::emplace(Ts&&... args) {
  if(mSize == mCapacity) {
    reserve(mCapacity == 0 ? 1 : mCapacity * 2);
  }

  new (reinterpret_cast<void*>(mValues + mSize)) T(std::forward<Ts>(args)...);
  mSize++;
}

template<typename T>
inline void DynamicArray<T>::add(const T& element) {
  if(mSize == mCapacity) {
    reserve(mCapacity == 0 ? 1 : mCapacity * 2);
  }

  new (reinterpret_cast<void*>(mValues + mSize)) T(element);
  mSize++;
}

template<typename T>
inline void DynamicArray<T>::add(const DynamicArray<T>& array, uint64 start) {
  assert(start >= 0 && start <= array.size());

  if(mSize + (array.size() - start) > mCapacity) {
    reserve(mSize + array.size() - start);
  }

  for(uint64 i = start; i < array.size(); i++) {
    new (reinterpret_cast<void*>(mValues + mSize)) T(array[i]);
    mSize++;
  }
}

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::remove(const T& element) {
  remove(find(element));
}

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::remove(const iterator& iter) {
  return removeAt(iter - begin());
}

template<typename T>
inline typename DynamicArray<T>::iterator DynamicArray<T>::removeAt(uint64 i) {
  assert(i >= 0 && i < mSize);
  mValues[i].~T();
  mSize--;

  if(i != mSize) {
    void* destination = reinterpret_cast<void*>(mValues + i);
    std::uintptr_t source = reinterpret_cast<std::uintptr_t>(destination) + sizeof(T);
    std::memmove(destination, reinterpret_cast<const void*>(source), (mSize - i) * sizeof(T));
  }

  return &mValues[i];
}

template<typename T>
inline void DynamicArray<T>::grow(uint64 numElements) {
  if((mSize + numElements) > mCapacity) {
    reserve(mCapacity == 0 ? numElements : (mCapacity + numElements) * 2);
  }

  mSize += numElements;
}

template<typename T>
inline void DynamicArray<T>::clear(bool free) {
  for(uint64 i = 0; i < mSize; i++) {
    mValues[i].~T();
  }

  mSize = 0;

  if(free && mCapacity) {
    mMemoryHandler.free(mValues, mCapacity * sizeof(T));
    mValues = nullptr;
    mCapacity = 0;
  }
}

}

#endif