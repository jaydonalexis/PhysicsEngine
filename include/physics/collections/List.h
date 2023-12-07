#ifndef PHYSICS_LIST_H
#define PHYSICS_LIST_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <functional>
#include <cassert>

namespace physics {

template<typename T>
class List {

  public:
    /* -- Nested Classes -- */
    
    /* Node for the list */
    struct ListNode {

      public:
        /* -- Attributes -- */

        /* Data */
        T data;

        /* Pointer to the next node in the list */
        ListNode* next;

        /* Pointer to the previous node in the list */
        ListNode* prev;

        /* -- Methods -- */

        /* Constructor */
        ListNode(const T& data) : data(data), next(nullptr), prev(nullptr) {}

        /* Constructor */
        ListNode(const T& data, ListNode* next, ListNode* prev) : data(data), next(next), prev(prev) {}
    };

  private:
    /* -- Attributes -- */

    /* First element in list */
    ListNode* mHead;

    /* Last Element in list */
    ListNode* mTail;

    /* Number of elements in the list */
    uint64 mSize;

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

    /* -- Methods -- */

    /* Perform a deep copy of the given list */
    void copy(const List& list) {
      std::function<ListNode*(ListNode*)> copy =
      [&](ListNode* head) -> ListNode* {
        if(!head) {
          return head;
        }

        ListNode* node = new (mMemoryHandler.allocate(sizeof(ListNode))) ListNode(head->data);

        node->next = copy(head->next);

        if(node->next) {
          node->next->prev = node;
        }
        else {
          mTail = node;
        }

        return node;
      };

      mHead = copy(list.mHead);
      mSize = list.mSize;
    }

  public:
    /* -- Methods -- */

    /* Constructor */
    List(MemoryHandler& memoryHandler);

    /* Copy Constructor */
    List(const List<T>& list);

    /* Destructor*/
    ~List();

    /* Get the first element in the list */
    ListNode* getHead() const;

    /* Get the last element in the list */
    ListNode* getTail() const;

    /* Get number of elements in the list */
    uint64 size() const;

    /* Return whether the list is empty */
    bool empty() const;

    /* Insert an element at the beginning of the list */
    void addFront(const T& data);

    /* Insert an element at the end of the list */
    void addBack(const T& data);

    /* Remove an element from the beginning of the list */
    void removeFront();

    /* Remove an element from the end of the list */
    void removeBack();

    /* Remove all elements in the list */
    void clear();

    /* Overloaded assignment operator */
    List& operator=(const List& list);
};

/* Constructor */
template<typename T>
inline List<T>::List(MemoryHandler& memoryHandler) : mHead(nullptr), mTail(nullptr), mSize(0), mMemoryHandler(memoryHandler) {}

/* Copy constructor */
template<typename T>
inline List<T>::List(const List<T>& list) : mHead(nullptr), mTail(nullptr), mSize(list.mSize), mMemoryHandler(list.mMemoryHandler) {
  copy(list);
}

/* Destructor */
template<typename T>
inline List<T>::~List() {
  clear();
}

/* Get the first element in the list */
template<typename T>
inline typename List<T>::ListNode* List<T>::getHead() const {
  return mHead;
}

/* Get the last element in the list */
template<typename T>
inline typename List<T>::ListNode* List<T>::getTail() const {
  return mTail;
}

/* Get number of elements in the list */
template<typename T>
inline uint64 List<T>::size() const {
  return mSize;
}

/* Return whether the list is empty */
template<typename T>
inline bool List<T>::empty() const {
  return (!mHead && !mTail);
}

/* Insert an element at the beginning of the list */
template<typename T>
inline void List<T>::addFront(const T& data) {
  ListNode* node = new (mMemoryHandler.allocate(sizeof(ListNode))) ListNode(data);

  if(!mTail) {
    mTail = node;
  }
  else {
    node->next = mHead;
    mHead->prev = node;
  }

  mHead = node;
  mSize++;
}

/* Insert an element at the end of the list */
template<typename T>
inline void List<T>::addBack(const T& data) {
  ListNode* node = new (mMemoryHandler.allocate(sizeof(ListNode))) ListNode(data);

  if(!mHead) {
    mHead = node;
  }
  else {
    mTail->next = node;
    node->prev = mTail;
  }
  
  mTail = node;
  mSize++;
}

/* Remove an element from the beginning of the list */
template<typename T>
inline void List<T>::removeFront() {
  ListNode* node = mHead;

  if(!mHead) {
    return;
  }
  else if(mHead == mTail) {
    mHead = mTail = nullptr;
  }
  else {
    mHead = node->next;
    mHead->prev = nullptr;
  }

  mMemoryHandler.free(node, sizeof(ListNode));
  mSize--;
}

/* Remove an element from the end of the list */
template<typename T>
inline void List<T>::removeBack() {
  ListNode* node = mTail;

  if(!mTail) {
    return;
  }
  else if(mHead == mTail) {
    mHead = mTail = nullptr;
  }
  else {
    mTail = node->prev;
    mTail->next = nullptr;
  }

  mMemoryHandler.free(node, sizeof(ListNode));
  mSize--;
}

/* Remove all elements in the list */
template<typename T>
inline void List<T>::clear() {
  ListNode* node = mHead;

  while(node) {
    ListNode* next = node->next;
    mMemoryHandler.free(node, sizeof(ListNode));
    node = next;
  }

  mHead = mTail = nullptr;
  mSize = 0;
}

/* Overloaded assignment operator */
template<typename T>
inline List<T>& List<T>::operator=(const List& list) {
  if(this != &list) {
    clear();
    copy(list);
  }  

  return *this;
}

}

#endif