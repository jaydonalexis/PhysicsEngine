#ifndef PHYSICS_DYNAMIC_ARRAY_H
#define PHYSICS_DYNAMIC_ARRAY_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <cassert>

namespace physics {

template<typename T>
class DynamicArray {

  public:
    /* -- Nested Classes -- */

    /* Iterator for the array */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to the aarray elements */
        T* mValues;

        /* Current index */
        uint64 mIndex;

        /* Size of the buffer */
        uint64 mSize;

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(void* values, uint64 index, uint64 size) : mValues(static_cast<T*>(values)), mIndex(index), mSize(size) {}

        /* Dereference */
        T& operator*() {
          assert(mIndex >= 0 && mIndex < mSize);
          return mValues[mIndex];
        }

        /* Constant dereference */
        const T& operator*() const {
          assert(mIndex >= 0 && mIndex < mSize);
          return mValues[mIndex];
        }

        /* Constant dereference */
        const T* operator->() const {
          assert(mIndex >= 0 && mIndex < mSize);
          return &(mValues[mIndex]);
        }

        /* Pre increment */
        Iterator& operator++() {
          assert(mIndex < mSize);
          mIndex++;
          return *this;
        }

        /* Post increment */
        Iterator operator++(int) {
          assert(mIndex < mSize);
          Iterator iter = *this;
          mIndex++;
          return iter;
        }

        /* Pre decrement */
        Iterator& operator--() {
          assert(mIndex > 0);
          mIndex--;
          return *this;
        }

        /* Post decrement */
        Iterator operator--(int) {
          assert(mIndex > 0);
          Iterator iter = *this;
          mIndex--;
          return iter;
        }

        /* Overloaded addition operator */
        Iterator operator+(const std::ptrdiff_t& diff) {
          return Iterator(mValues, mIndex + diff, mSize);
        }

        /* Overloaded addition operator with assignment */
        Iterator& operator+=(const std::ptrdiff_t& diff) {
          mIndex += diff;
          return *this;
        }

        /* Overloaded subtraction operator */
        Iterator operator-(const std::ptrdiff_t& diff) {
          return Iterator(mValues, mIndex - diff, mSize);
        }

        /* Overlaoded subtraction operator with assignment */
        Iterator& operator-=(const std::ptrdiff_t& diff) {
          mIndex -= diff;
          return *this;
        }

        /* Difference operator */
        std::ptrdiff_t operator-(const Iterator& iter) const {
          return mIndex - iter.mIndex;
        }

        /* Overloaded less than operator */
        bool operator<(const Iterator& iter) const {
          return mIndex < iter.mIndex;
        }

        /* Overloaded greater than operator */
        bool operator>(const Iterator& iter) const {
          return mIndex > iter.mIndex;
        }

        /* Overloaded less than or equal to operator */
        bool operator<=(const Iterator& iter) const {
          return mIndex <= iter.mIndex;
        }

        /* Overloaded greater than or equal to operator */
        bool operator>=(const Iterator& iter) const {
          return mIndex >= iter.mIndex;
        }

        /* Overloaded equality operator */
        bool operator==(const Iterator& iter) const {
          assert(mIndex >= 0 && mIndex <= mSize);

          if(mIndex == mSize && iter.mIndex == iter.mSize) {
            return true;
          }

          return &(mValues[mIndex]) == &(iter.mValues[iter.mIndex]);
        }

        /* Overloaded inequality operator */
        bool operator!=(const Iterator& iter) const {
            return !((*this) == iter);
        }

        /* -- Friends -- */

        /* Superclass */
        friend class DynamicArray<T>;
    };

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
    /* -- Methods -- */

    /* Constructor */
    DynamicArray(MemoryHandler& memoryHandler, uint64 capacity = 0);

    /* Copy constructor */
    DynamicArray(const DynamicArray<T>& array);

    /* Destructor */
    ~DynamicArray();

    /* Retrieve item at front of the array */
    T& front();

    /* Retrieve item at back of the array */
    T& back();

    /* Retrieve item at index i of the array */
    T& at(uint64 i);

    /* Find element in the array */
    Iterator find(const T& element);

    /* Return iterator to the beginning of the array */
    Iterator begin();

    /* Return iterator to the end of the array */
    Iterator end();

    /* Get number of elements in the array */
    uint64 size() const;

    /* Get allocated size of the array  */
    uint64 capacity() const;

    /* Return whether the array is empty */
    bool empty() const;

    /* Reserve more memory for the array */
    void reserve(uint64 size);

    /* Add uninitialized elements to the end of the array */
    void fill(uint64 numElements);

    /* Construct an item directly at the end of the array, joining it to the existing array */
    template<typename... Ts>
    void emplace(Ts&&... args);

    /* Add an item to the end of the array */
    void add(const T& element);

    /* Add either an entire array or a portion of an array to the end of the current array */
    void add(const DynamicArray<T>& array, uint64 start = 0);

    /* Remove a specific element from the array */
    Iterator remove(const T& element);

    /* Remove an element from the array associated with the provided iterator */
    Iterator remove(const Iterator& iter);

    /* Remove the element at index i in the array */
    Iterator erase(const uint64 i);

    /* Clear the contents of the array */
    void clear(bool free = false);

    /* Overloaded value access operator */
    T& operator[](const uint64 i);

    /* Overloaded value access operator */
    const T& operator[](const uint64 i) const;

    /* Overloaded equality operator */
    bool operator==(const DynamicArray<T>& array) const;

    /* Overloaded inequality operator */
    bool operator!=(const DynamicArray<T>& array) const;

    /* Overlaoded assignment operator */
    DynamicArray<T>& operator=(const DynamicArray<T>& array);
};

/* Template functions are implicitly inlined */

/* Constructor */
template<typename T>
inline DynamicArray<T>::DynamicArray(MemoryHandler& memoryHandler, uint64 capacity) : mValues(nullptr), mSize(0), mCapacity(0), mMemoryHandler(memoryHandler) {
  if(capacity > 0) {
    reserve(capacity);
  }
}

/* Copy constructor */
template<typename T>
inline DynamicArray<T>::DynamicArray(const DynamicArray<T>& array) : mValues(nullptr), mSize(0), mCapacity(0), mMemoryHandler(array.mMemoryHandler) {
  if(array.mCapacity > 0) {
    reserve(array.mCapacity);
  }

  add(array);
}

/* Destructor */
template<typename T>
inline DynamicArray<T>::~DynamicArray() {
  if(mCapacity > 0) {
    clear(true);
  }
}

/* Retrieve item at front of the array */
template<typename T>
inline T& DynamicArray<T>::front() {
  assert(mSize > 0);
  return mValues[0];
}

/* Retrieve item at back of the array */
template<typename T>
inline T& DynamicArray<T>::back() {
  assert(mSize > 0);
  return mValues[mSize - 1];
}

/* Retrieve item at index i of the array */
template<typename T>
inline T& DynamicArray<T>::at(uint64 i) {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

/* Find element in the array */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::find(const T& element) {
  for(uint64 i = 0; i < mSize; i++) {
    if(element == mValues[i]) {
      return Iterator(mValues, i, mSize);
    }
  }

  return end();
}

/* Return iterator to the beginning of the array */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::begin() {
  return Iterator(mValues, 0, mSize);
}

/* Return iterator to the end of the array */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::end() {
  return Iterator(mValues, mSize, mSize);
}

/* Get number of elements in the array */
template<typename T>
inline uint64 DynamicArray<T>::size() const {
  return mSize;
}

/* Get allocated size of the array */
template<typename T>
inline uint64 DynamicArray<T>::capacity() const {
  return mCapacity;
}

/* Return whether the array is empty */
template<typename T>
inline bool DynamicArray<T>::empty() const {
  return mSize == 0;
}

/* Reserve more memory for the array */
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

/* Add uninitialized elements to the end of the array */
template<typename T>
inline void DynamicArray<T>::fill(uint64 numElements) {
  if((mSize + numElements) > mCapacity) {
    reserve(mCapacity == 0 ? numElements : (mCapacity + numElements) * 2);
  }

  mSize += numElements;
}

/* Construct an item directly at the end of the array, joining it to the existing array */
template<typename T>
template<typename...Ts>
inline void DynamicArray<T>::emplace(Ts&&... args) {
  if(mSize == mCapacity) {
    reserve(mCapacity == 0 ? 1 : mCapacity * 2);
  }

  new (reinterpret_cast<void*>(mValues + mSize)) T(std::forward<Ts>(args)...);
  mSize++;
}

/* Add an item to the end of the array */
template<typename T>
inline void DynamicArray<T>::add(const T& element) {
  if(mSize == mCapacity) {
    reserve(mCapacity == 0 ? 1 : mCapacity * 2);
  }

  new (reinterpret_cast<void*>(mValues + mSize)) T(element);
  mSize++;
}

/* Add either an entire array or a portion of an array to the end of the current array */
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

/* Remove a specific element from the array */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::remove(const T& element) {
  return remove(find(element));
}

/* Debug */
/* Remove an element from the array associated with the provided iterator */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::remove(const Iterator& iter) {
  assert(mValues == iter.mValues);
  return erase(iter.mIndex);
}

/* Remove the element at index i in the array */
template<typename T>
inline typename DynamicArray<T>::Iterator DynamicArray<T>::erase(uint64 i) {
  assert(i >= 0 && i < mSize);
  mValues[i].~T();
  mSize--;

  if(i != mSize) {
    void* destination = reinterpret_cast<void*>(mValues + i);
    std::uintptr_t source = reinterpret_cast<std::uintptr_t>(destination) + sizeof(T);
    std::memmove(destination, reinterpret_cast<const void*>(source), (mSize - i) * sizeof(T));
  }

  return Iterator(mValues, i, mSize);
}

/* Clear the contents of the array */
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

/* Overloaded value access operator */
template<typename T>
inline T& DynamicArray<T>::operator[](const uint64 i) {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

/* Overloaded value access operator */
template<typename T>
inline const T& DynamicArray<T>::operator[](const uint64 i) const {
  assert(i >= 0 && i < mSize);
  return mValues[i];
}

/* Overloaded equality operator */
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

/* Overloaded inequality operator */
template<typename T>
inline bool DynamicArray<T>::operator!=(const DynamicArray<T>& array) const {
  return !((*this) == array);
}

/* Overloaded assignment operator */
template<typename T>
inline DynamicArray<T>& DynamicArray<T>::operator=(const DynamicArray<T>& array) {
  if(this != &array) {
    clear();
    add(array);
  }

  return *this;
}

}

#endif