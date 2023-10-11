#ifndef PHYSICS_STACK_H
#define PHYSICS_STACK_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/collections/List.h>
#include <cassert>

namespace physics {

template<typename T>
class Stack {

  private:
    /* -- Attributes -- */

    /* Underlying list data structure for stack */
    List<T> mList;

  public:
    /* -- Methods -- */

    /* Constructor */
    Stack(MemoryHandler& memoryHandler);

    /* Copy constructor */
    Stack(const Stack& stack);

    /* Destructor */
    ~Stack();

    /* Push an item to the top of the stack */
    void push(const T& data);

    /* Remove an element from the top of the stack and return it */
    T pop();

    /* Get number of elements in the stack */
    uint64 size() const;

    /* Return whether the stack is empty */
    bool empty() const;

    /* Remove all elements in the stack */
    void clear();
};

/* Constructor */
template<typename T>
inline Stack<T>::Stack(MemoryHandler& memoryHandler) : mList(memoryHandler) {}

/* Copy constructor */
template<typename T>
inline Stack<T>::Stack(const Stack& stack) : mList(stack.mList) {}

/* Destructor */
template<typename T>
inline Stack<T>::~Stack() {
  clear();
}

/* Push an item to the top of the stack */
template<typename T>
inline void Stack<T>::push(const T& data) {
  mList.addFront(data);
}

/* Remove an element from the top of the stack and return it */
template<typename T>
inline T Stack<T>::pop() {
  T temp = mList.getHead()->data;
  mList.removeFront();
  return temp;
}

/* Get number of elements in the stack */
template<typename T>
inline uint64 Stack<T>::size() const {
  return mList.size();
}

/* Return whether the stack is empty */
template<typename T>
inline bool Stack<T>::empty() const {
  return mList.empty();
}

/* Remove all elements in the stack */
template<typename T>
inline void Stack<T>::clear() {
  mList.clear();
}

}

#endif