#ifndef PHYSICS_QUEUE_H
#define PHYSICS_QUEUE_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/collections/List.h>
#include <cassert>

namespace physics {

template<typename T>
class Queue {

  private:
    /* -- Attributes -- */

    /* Underlying list data structure for queue */
    List<T> mList;

  public:
    /* -- Methods -- */

    /* Constructor */
    Queue(MemoryHandler& memoryHandler);

    /* Copy constructor */
    Queue(const Queue& queue);

    /* Destructor */
    ~Queue();

    /* Push an item to the end of the queue */
    void push(const T& data);

    /* Remove an element from the front of the queue */
    T pop();

    /* Get number of elements in the queue */
    uint64 size() const;

    /* Return whether the queue is empty */
    bool empty() const;

    /* Remove all elements in the queue */
    void clear();
};

/* Constructor */
template<typename T>
inline Queue<T>::Queue(MemoryHandler& memoryHandler) : mList(memoryHandler) {}

/* Copy constructor */
template<typename T>
inline Queue<T>::Queue(const Queue& queue) : mList(queue.mList) {}

/* Destructor */
template<typename T>
inline Queue<T>::~Queue()  {
  clear();
}

/* Push an item to the end of the queue */
template<typename T>
inline void Queue<T>::push(const T& data) {
  mList.addBack(data);
}

/* Remove an item from the front of the queue */
template<typename T>
inline T Queue<T>::pop() {
  T temp = mList.getHead()->data;
  mList.removeFront();
  return temp;
}

/* Get number of elements in the queue */
template<typename T>
inline uint64 Queue<T>::size() const {
  return mList.size();
}

/* Return whether the queue is empty */
template<typename T>
inline bool Queue<T>::empty() const {
  return mList.empty();
}

/* Remove all elements in the queue */
template<typename T>
inline void Queue<T>::clear() {
  mList.clear();
}

}

#endif