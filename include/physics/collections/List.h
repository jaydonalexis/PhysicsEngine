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

    /* Iterator for the list */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to the list */
        const List<T>* mList;

        /* Pointer to a list node */
        ListNode* mNode;

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(const List<T>* list, ListNode* node) : mList(list), mNode(node) {}

        /* Dereference */
        T& operator*() {
          assert(mNode);
          return mNode->data;
        }
        
        /* Constant dereference */
        const T& operator*() const {
          assert(mNode);
          return mNode->data;
        }

        /* Constant dereference */
        const T* operator->() const {
          assert(mNode);
          return &(mNode->data);
        }

        /* Pre increment */
        Iterator& operator++() {
          if(mNode) {
            mNode = mNode->next;
          }

          return *this;
        }

        /* Post increment */
        Iterator operator++(int) {
          Iterator iter = *this;

          if(mNode) {
            mNode = mNode->next;
          }
          
          return iter;
        }

        /* Pre decrement */
        Iterator& operator--() {
          if(mNode) {
            mNode = mNode->prev;
          }
          else {
            mNode = mList->mTail;
          }

          return *this;
        }

        /* Post decrement */
        Iterator operator--(int) {
          Iterator iter = *this;

          if(mNode) {
            mNode = mNode->prev;
          }
          else {
            mNode = mList->mTail;
          }

          return iter;
        }

        /* Equality operator */
        bool operator==(const Iterator& iter) const {
          assert(mList && iter.mList);
          return mNode == iter.mNode && mList == iter.mList;
        }

        /* Inequality operator */
        bool operator!=(const Iterator& iter) const {
          return !((*this) == iter);
        }

        /* -- Friends -- */

        /* Superclass */
        friend class List<T>;
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

/* Return iterator to the beginning of the list */
template<typename T>
inline typename List<T>::Iterator List<T>::begin() const {
  return Iterator(this, mHead);
}

/* Return iterator to the end of the list */
template<typename T>
inline typename List<T>::Iterator List<T>::end() const {
  return Iterator(this, nullptr);
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

/* Insert an element into the list associated with the provided iterator */
template<typename T>
inline void List<T>::insert(const Iterator& iter, const T& data) {
  if(iter == begin()) {
    addFront(data);
  }
  else if(iter == end()) {
    addBack(data);
  }
  else {
    ListNode* node = new (mMemoryHandler.allocate(sizeof(ListNode))) ListNode(data);
    node->next = iter.mNode;
    node->prev = iter.mNode->prev;
    node->prev->next = node;
    node->next->prev = node;
    mSize++;
  }
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


/* Remove an element from the list associated with the provided iterator */
template<typename T>
inline void List<T>::erase(const Iterator& iter) {
  if(!iter.mNode) {
    return;
  }
  else if(iter.mNode == mHead) {
    removeFront();
  }
  else if(iter.mNode == mTail) {
    removeBack();
  }
  else {
    ListNode* node = iter.mNode;
    node->prev->next = node->next;
    node->next->prev = node->prev;
    mMemoryHandler.free(node, sizeof(ListNode));
    mSize--;
  }
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