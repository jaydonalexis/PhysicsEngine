#ifndef PHYSICS_DOUBLY_LINKED_LIST_H
#define PHYSICS_DOUBLY_LINKED_LIST_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <cassert>

namespace physics {

template<typename T>
class DoublyLinkedList {

  public:
    /* -- Nested Classes -- */
    
    /* Node of the doubly linked list */
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
        ListNode(T data) : data(data), next(nullptr), prev(nullptr) {}

        /* Constructor */
        ListNode(T data, ListNode* next, ListNode* prev) : data(data), next(next), prev(prev) {}
    };

    /* Iterator for the doubly linked list */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to a node */
        ListNode* mNode;

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(const ListNode* node) : mNode(node) {}

        /* Dereferance */
        T operator*() const {
          return mNode->data;
        }

        /* Pre increment */
        Iterator& operator++() {
          mNode = mNode->next;
          return *this;
        }

        /* Post increment */
        Iterator operator++(int) {
          Iterator iter = *this;
          mNode = mNode->next;
          return iter;
        }

        /* Decrement */
        Iterator& operator--() {
          mNode = mNode->prev;
          retun *this;
        }

        /* Overloaded equality operator */
        bool operator==(const Iterator& iter) const {
          return mNode == iter.mNode;
        }

        /* Overloaded inequality operator */
        bool operator!=(const Iterator& iter) const {
          return mNode != iter.mNode;
        }

        /* Get the current node */
        ListNode* getNode() {
          return mNode;
        }
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

  public:
    /* -- Methods -- */

    /* Constructor */
    DoublyLinkedList(MemoryHandler& memoryHandler);

    /* Copy Constructor */
    DoublyLinkedList(const DoublyLinkedList<T>& list);

    /* Destructor*/
    ~DoublyLinkedList();

    /* Get the first element in the list */
    ListNode* getHead() const;

    /* Get the last element in the list */
    ListNode* getTail() const;

    /* Return iterator to the beginning of the list */
    Iterator begin() const;

    /* Return iterator to the end of the list */
    Iterator end() const ;

    /* Get number of elements in the list */
    uint64 size() const;

    /* Return whether the list is empty */
    bool empty() const;

    /* Insert an element at the beginning of the list */
    void addFront(const T& data);

    /* Insert an element at the end of the list */
    void addBack(const T& data);

    /* Insert an element into the list associated with the provided iterator */
    void insert(const Iterator& iter, const T& data);

    /* Remove an element from the beginning of the list */
    void removeFront();

    /* Remove an element from the end of the list */
    void removeBack();

    /* Remove an element from the list associated with the provided iterator */
    void erase(const Iterator& iter);

    /* Remove all elements in the list */
    void clear();

    /* -- Friends -- */

    friend class Iterator;
};

/* Constructor */
template<typename T>
inline DoublyLinkedList<T>::DoublyLinkedList(MemoryHandler& memoryHandler) : mHead(nullptr), mTail(nullptr), mSize(0), mMemoryHandler(memoryHandler) {}

/* Copy constructor */
template<typename T>
inline DoublyLinkedList<T>::DoublyLinkedList(const DoublyLinkedList<T>& list) : mHead(list.mHead), mTail(list.mTail), mSize(list.mSize), mMemoryHandler(list.mMemoryHandler) {}

/* Destructor */
template<typename T>
inline DoublyLinkedList<T>::~DoublyLinkedList() {
  clear();
}

/* Get the first element in the list */
template<typename T>
inline typename DoublyLinkedList<T>::ListNode* DoublyLinkedList<T>::getHead() const {
  return mHead;
}

/* Get the last element in the list */
template<typename T>
inline typename DoublyLinkedList<T>::ListNode* DoublyLinkedList<T>::getTail() const {
  return mTail;
}

/* Return iterator to the beginning of the list */
template<typename T>
inline typename DoublyLinkedList<T>::Iterator DoublyLinkedList<T>::begin() const {
  return Iterator(mHead);
}

/* Return iterator to the end of the list */
template<typename T>
inline typename DoublyLinkedList<T>::Iterator DoublyLinkedList<T>::end() const {
  return Iterator(mTail);
}

/* Get number of elements in the list */
template<typename T>
inline uint64 DoublyLinkedList<T>::size() const {
  return mSize;
}

/* Return whether the list is empty */
template<typename T>
inline bool DoublyLinkedList<T>::empty() const {
  return (!mHead && !mTail);
}

/* Insert an element at the beginning of the list */
template<typename T>
inline void DoublyLinkedList<T>::addFront(const T& data) {
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
inline void DoublyLinkedList<T>::addBack(const T& data) {
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

/* Insert an element into the list associated with the provided iterator */
template<typename T>
inline void DoublyLinkedList<T>::insert(const Iterator& iter, const T& data) {
  if(iter) {
    switch(iter) {
      case begin():
        addFront(data);
        break;

      default:
        ListNode* node = new (mMemoryHandler.allocate(sizeof(ListNode))) ListNode(data);
        node->next = iter.getNode();
        node->prev = iter.getNode()->prev;
        node->prev->next = node;
        node->next->prev = node;
        mSize++;
        break;
    }
  }
}

/* Remove an element from the beginning of the list */
template<typename T>
inline void DoublyLinkedList<T>::removeFront() {
  ListNode* node = mHead;

  switch(mHead) {
    case nullptr:
      return;

    case mTail:
      mHead = mTail = nullptr;
      break;

    default:
      mHead = node->next;
      mHead->prev = nullptr;
      break;
  }

  mMemoryHandler.free(node, sizeof(ListNode));
  mSize--;
}

/* Remove an element from the end of the list */
template<typename T>
inline void DoublyLinkedList<T>::removeBack() {
  ListNode* node = mTail;

  switch(mTail) {
    case nullptr:
      return;

    case mHead:
      mHead = mTail = nullptr;
      break;
    
    default:
      mTail = node->prev;
      mTail->next = nullptr;
      break;
  }

  mMemoryHandler.free(node, sizeof(ListNode));
  mSize--;
}

/* Remove an element from the list associated with the provided iterator */
template<typename T>
inline void DoublyLinkedList<T>::erase(const Iterator& iter) {
  if(iter) {
    swtich(iter) {
      case begin():
        removeFront();
        break;
      
      case end():
        removeBack();
        break;

      default:
        ListNode* node = iter.getNode();
        node->prev->next = node->next;
        node->next->prev = node->prev;
        mMemoryHandler.free(node, sizeof(ListNode));
        mSize--;
        break;
    }
  }
}

/* Remove all elements in the list */
template<typename T>
inline void DoublyLinkedList<T>::clear() {
  ListNode* node = mHead;

  while(node) {
    ListNode* next = node->next;
    mMemoryHandler.free(node, sizeof(ListNode));
    node = next;
  }

  mHead = mTail = nullptr;
  mSize = 0;
}

}

#endif