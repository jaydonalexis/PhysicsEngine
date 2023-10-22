#ifndef PHYSICS_HASH_MAP_H
#define PHYSICS_HASH_MAP_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <physics/collections/Pair.h>
#include <functional>
#include <iostream>
#include <fstream>
#include <cassert>

namespace physics {

template<typename K, typename V, class Hash = std::hash<K>, class KeyEqual = std::equal_to<K>>
class HashMap {

  private:
    /* -- Nested Classes -- */

    /* Color for red-black tree nodes */
    enum class Color {Red, Black};

    /* Node for the red-black tree */
    struct TreeNode {

      public:
        /* -- Attributes -- */

        /* Key-Value pair */
        Pair<K, V> keyValue;

        /* Pointer to the parent node */
        TreeNode* parent;

        /* Pointer to the left child node */
        TreeNode* left;

        /* Pointer to the right child node */
        TreeNode* right;

        /* Pointer to the next node in the list of nodes with the same hash values */
        TreeNode* next;

        /* Pointer to the previous node in the list of nodes with the same hash values */
        TreeNode* prev;

        /* Node color */
        Color color;

        /* -- Methods -- */

        /* Constructor */
        TreeNode(const Pair<K, V>& keyValue) : keyValue(keyValue), parent(nullptr), left(nullptr), right(nullptr), next(nullptr), prev(nullptr), color(Color::Red) {}
        
        /* Return the minimum node relative to the current node */
        TreeNode* min() const {
          TreeNode* node = (TreeNode*)this;
          
          while(node->left) {
            node = node->left;
          }

          return node;
        }

        /* Return the neighbouring node in the inorder sequence relative to the current node */
        TreeNode* neighbour() const {
          TreeNode* node = (TreeNode*)this;

          while(node->prev) {
            node = node->prev;
          }

          if(node->right) {
            return node->right->min();
          }

          while(node->parent && node == node->parent->right) {
            node = node->parent;
          }

          return node->parent;
        }

        /* Get the sibling of the current node */
        TreeNode* sibling() const {
          TreeNode* node = (TreeNode*)this;
          assert(node && node->parent);

          if(node == node->parent->left) {
            return node->parent->right;
          }
          else {
            return node->parent->left;
          }
        }

        /* Get the grandparent of the current node */
        TreeNode* grandparent() const {
          TreeNode* node = (TreeNode*)this;
          assert(node && node->parent && node->parent->parent);
          return node->parent->parent;
        }

        /* Get the uncle of the current node */
        TreeNode* uncle() const {
          TreeNode* node = (TreeNode*)this;
          assert(node && node->parent && node->parent->parent);
          return node->parent->sibling();
        }
    };

    /* Red-Black tree underlying data structure */
    class Tree {
      
      private:
        /* -- Attributes -- */

        /* Root node of the tree */
        TreeNode* mRoot;

        /* Number of nodes in the tree */
        uint64 mSize;

        /* Memory handler */
        MemoryHandler& mMemoryHandler;

        /* -- Methods -- */

        /* Number of nodes in the tree */
        uint64 size() const {
          return mSize;
        }

        /* Get the color of the given node */
        Color getColor(TreeNode* node) const {
          if(!node) {
            /* null nodes are black by default */
            return Color::Black;
          }

          return node->color;
        }

        /* Set the color of the given node */
        void setColor(TreeNode* node, Color color) {
          if(!node) {
            return;
          }

          node->color = color;
        }

        /* Replace an old node in the tree with a new node */
        void replace(TreeNode* oldNode, TreeNode* newNode) {
          assert(oldNode && newNode);

          if(mRoot == oldNode) {
            mRoot = newNode;
          }
          else if(oldNode == oldNode->parent->left) {
            oldNode->parent->left = newNode;
          }
          else {
            oldNode->parent->right = newNode;
          }

          if(oldNode->left) {
            oldNode->left->parent = newNode;
          }

          if(oldNode->right) {
            oldNode->right->parent = newNode;
          }

          oldNode->keyValue.second = newNode->keyValue.second;
          *newNode = *oldNode;
          mMemoryHandler.free(oldNode, sizeof(TreeNode));
        }

        /* Transplant an old node in the tree with a new node */
        void transplant(TreeNode* oldNode, TreeNode* newNode) {
          assert(oldNode);

          if(!oldNode->parent) {
            mRoot = newNode;
          }
          else if(oldNode == oldNode->parent->left) {
            oldNode->parent->left = newNode;
          }
          else {
            oldNode->parent->right = newNode;
          }

          if(newNode) {
            newNode->parent = oldNode->parent;
          }
        }

        /* Swap two nodes in the tree */
        void swap(TreeNode* highNode, TreeNode* lowNode) {
          assert(highNode && lowNode);

          if(!highNode->parent) {
            mRoot = lowNode;
          }
          else if(highNode->parent->left == highNode) {
            highNode->parent->left = lowNode;
          }
          else {
            highNode->parent->right = lowNode;
          }

          if(lowNode->left) {
            lowNode->left->parent = highNode;
          }

          if(lowNode->right) {
            lowNode->right->parent = highNode;
          }

          if(highNode->left == lowNode) {
            if(highNode->right) {
              highNode->right->parent = lowNode;
            }

            highNode->left = highNode;
            lowNode->parent = lowNode;
          }
          else if(highNode->right == lowNode) {
            if(highNode->left) {
              highNode->left->parent = lowNode;
            }

            highNode->right = highNode;
            lowNode->parent = lowNode;
          }
          else {
            if(highNode->left) {
              highNode->left->parent = lowNode;
            }

            if(highNode->right) {
              highNode->right->parent = lowNode;
            }

            if(lowNode->parent->left == lowNode) {
              lowNode->parent->left = highNode;
            }
            else {
              lowNode->parent->right = highNode;
            }
          }

          Pair<K, V> highKeyValue = highNode->keyValue;
          Pair<K, V> lowKeyValue = lowNode->keyValue;
          TreeNode highCopy = *highNode;
          *highNode = *lowNode;
          *lowNode = highCopy;
          lowNode->keyValue = lowKeyValue;
          highNode->keyValue = highKeyValue;
        }

        /* Rotate left with the given node as the pivot  */
        void rotateLeft(TreeNode* node) {
          TreeNode* n;
          assert(node);
          n = node->right;
          transplant(node, n);
          node->right = n->left;

          if(n->left) {
            n->left->parent = node;
          }

          n->left = node;
          node->parent = n;
        }

        /* Rotate right with the given node as the pivot */
        void rotateRight(TreeNode* node) {
          TreeNode* n;
          assert(node);
          n = node->left;
          transplant(node, n);
          node->left = n->right;

          if(n->right) {
            n->right->parent = node;
          }

          n->right = node;
          node->parent = n;
        }

        /* Find a node in the tree with the given key */
        TreeNode* findNode(const K& key) const {
          TreeNode* n = mRoot;

          while(n) {
            if(Hash()(key) < Hash()(n->keyValue.first)) {
              n = n->left;
            }
            else if(Hash()(key) > Hash()(n->keyValue.first)) {
              n = n->right;
            }
            else {
              while(n) {
                int count = 0;
                if(KeyEqual()(key, n->keyValue.first)) {
                  return n;
                }

                n = n->next;
                count++;
              }
            }
          }

          return n;
        }

        /* Insert a node into the tree based on BST criteria */
        void insertNode(TreeNode* node) {
          TreeNode* n = mRoot;
          assert(node);

          if(n) {
            while(true) {
              if(Hash()(node->keyValue.first) < Hash()(n->keyValue.first)) {
                if(n->left) {
                  n = n->left;
                }
                else {
                  n->left = node;
                  break;
                }
              }
              else if(Hash()(node->keyValue.first) > Hash()(n->keyValue.first)) {
                if(n->right) {
                  n = n->right;
                }
                else {
                  n->right = node;
                  break;
                }
              }
              else {
                if(KeyEqual()(node->keyValue.first, n->keyValue.first)) {
                  replace(n, node);
                  return;
                }
                else {
                  node->next = n->next;

                  if(n->next) {
                    n->next->prev = node;
                  }

                  n->next = node;
                  node->prev = n;
                  mSize++;
                  return;
                }
              }
            }
          }

          node->parent = n;
          node->left = nullptr;
          node->right = nullptr;
          setColor(node, Color::Red);

          if(!n) {
            mRoot = node;
          }

          insertFixup(node);
          mSize++;
        }

        /* Remove the given node from the tree if it exists */
        void removeNode(TreeNode* node) {
          TreeNode* n;

          if(!node) {
            return;
          }

          if(node != mRoot && !node->parent) {
            if(node->prev) {
              node->prev->next = node->next;
            }

            if(node->next) {
              node->next->prev = node->prev;
            }

            mMemoryHandler.free(node, sizeof(TreeNode));
            mSize--;
            return;
          }
          else if(node->parent || node == mRoot) {
            if(node->next) {
              if(node->parent && node == node->parent->left) {
                node->parent->left = node->next;
              }
              else if(node->parent && node == node->parent->right) {
                node->parent->right = node->next;
              }

              if(node->left) {
                node->left->parent = node->next;
              }

              if(node->right) {
                node->right->parent = node->next;
              }

              node->next->parent = node->parent;
              node->next->left = node->left;
              node->next->right = node->right;
              node->next->prev = nullptr;
              setColor(node->next, getColor(node));
              
              if(node == mRoot) {
                mRoot = node->next;
              }

              mMemoryHandler.free(node, sizeof(TreeNode));
              mSize--;
              return;
            }
          }

          if(node->left && node->right) {
            TreeNode* k = node->left;

            while(k->right) {
              k = k->right;
            }

            swap(node, k);
          }

          n = node->right ? node->right : node->left;

          if(getColor(node) == Color::Black) {
            setColor(node, getColor(n));
            removeFixup(node);
          }

          transplant(node, n);

          if(!node->parent && n) {
            setColor(n, Color::Black);
          }

          mMemoryHandler.free(node, sizeof(TreeNode));
          mSize--;
        }

        /* Perform rotation and recoloring as appropriate to maintain the red-black tree properties after insertion */
        void insertFixup(TreeNode* node) {
          assert(node);
          
          while(true) {
            if(!node->parent) {
              setColor(node, Color::Black);
              break;
            }

            if(getColor(node->parent) == Color::Black) {
              break;
            }

            if(getColor(node->uncle()) == Color::Red) {
              setColor(node->parent, Color::Black);
              setColor(node->uncle(), Color::Black);
              setColor(node->grandparent(), Color::Red);
              node = node->grandparent();
              continue;
            }

            if(node == node->parent->right && node->parent == node->grandparent()->left) {
              rotateLeft(node->parent);
              node = node->left;
            }
            else if(node == node->parent->left && node->parent == node->grandparent()->right) {
              rotateRight(node->parent);
              node = node->right;
            }

            setColor(node->parent, Color::Black);
            setColor(node->grandparent(), Color::Red);

            if(node == node->parent->left && node->parent == node->grandparent()->left) {
              rotateRight(node->grandparent());
            }
            else {
              rotateLeft(node->grandparent());
            }

            break;
          }
        }

        /* Perform rotation and recoloring as appropriate to maintain the red-black tree properties after deletion */

        /* 
         * Step 1 (implemented in removeNode())
         * - If the node deleted has two null children, its replacement x is null
         * - If the node deleted has 1 null child and 1 non null child, its replacement x is the non null child
         * - If the node deleted has two non null children, consider x as the replacement's right child before splice out
         * 
         * Step 2 (Shared implementation between here and in removeNode())
         * - If the node deleted is red and its replacement is red or null; we are done
         * - If the node deleted is red and its replacement is black and non null, color the replacement red and proceed to the appropriate case
         * - If the node deleted is black and its replacement is red, color the replacement black; we are done
         * - If the node deleted is black, its replacement is black, and x is the root of the tree, we are done
         * - If the node we deleted is black, its replacement is black, and x is not the root of the tree, proceed to the appropriate case
         * 
         * Step 3
         * - Case 0 where x is red: 
         *   Color x black; we are done
         * - Case 1 where x is black and its sibling w is red: 
         *   Color w black
         *   Color x's parent red
         *   Rotate x's parent
         *   Change w
         *   With the new x and w, decide on case 2, 3 or 4 from here
         * - Case 2 where x is black and its sibling w is red
         *   Color w red
         *   Set x to x's parent
         * - Case 3 where x is black, its sibling w is blackm and if x is the left child, w's left child
         * - is red and w's right child is black or vice versa
         *   Color w's child black
         *   Color w red
         *   Rotate w
         *   Change w
         *   Proceed to case 4
         * - Case 4 where x is black, w is black and if x is the left child, w's right child is red or vice versa
         *   Color w the same color as x's parent
         *   Color x's parent black
         *   Color w's child black
         *   Rotate x's parent
         */
        void removeFixup(TreeNode* node) {
          assert(node);

          while(true) {
            if(!node->parent) {
              break;
            }

            if(getColor(node->sibling()) == Color::Red) {
              setColor(node->parent, Color::Red);
              setColor(node->sibling(), Color::Black);

              if(node == node->parent->left) {
                rotateLeft(node->parent);
              }
              else {
                rotateRight(node->parent);
              }
            }

            if(getColor(node->parent) == Color::Black &&
               getColor(node->sibling()) == Color::Black &&
               getColor(node->sibling()->left) == Color::Black &&
               getColor(node->sibling()->right) == Color::Black) {
              setColor(node->sibling(), Color::Red);
              node = node->parent;
              continue;
            }

            if(getColor(node->parent) == Color::Red &&
               getColor(node->sibling()) == Color::Black &&
               getColor(node->sibling()->left) == Color::Black &&
               getColor(node->sibling()->right) == Color::Black) {
              setColor(node->sibling(), Color::Red);
              setColor(node->parent, Color::Black);
              break;
            }

            if(node == node->parent->left &&
               getColor(node->sibling()) == Color::Black &&
               getColor(node->sibling()->left) == Color::Red && 
               getColor(node->sibling()->right) == Color::Black) {
              setColor(node->sibling(), Color::Red);
              setColor(node->sibling()->left, Color::Black);
              rotateRight(node->sibling());
            }
            else if(node == node->parent->right &&
                    getColor(node->sibling()) == Color::Black &&
                    getColor(node->sibling()->left) == Color::Black &&
                    getColor(node->sibling()->right) == Color::Red) {
              setColor(node->sibling(), Color::Red);
              setColor(node->sibling()->right, Color::Black);
              rotateLeft(node->sibling());
            }

            setColor(node->sibling(), getColor(node->parent));
            setColor(node->parent, Color::Black);

            if(node == node->parent->left) {
              setColor(node->sibling()->right, Color::Black);
              rotateLeft(node->parent);
            }
            else {
              setColor(node->sibling()->left, Color::Black);
              rotateRight(node->parent);
            }

            break;
          }
        }

        /* Perform a deep copy of the given tree */
        void copy(const Tree& tree) {
          std::function<TreeNode*(TreeNode*)> copy =
          [&](TreeNode* root) -> TreeNode* {
            if(!root) {
              return root;
            }

            TreeNode* head = nullptr;
            TreeNode* tail = nullptr;
            TreeNode* n = root;

            while(n) {
              TreeNode* k = new (mMemoryHandler.allocate(sizeof(TreeNode))) TreeNode(n->keyValue);

              if(!head) {
                head = k;
                tail = k;
              }
              else {
                tail->next = k;
                k->prev = tail;
                tail = k;
              }

              n = n->next;
            }

            head->left = copy(root->left);
            head->right = copy(root->right);
            return head;
          };

          mRoot = copy(tree.mRoot);
          mSize = tree.mSize;
        }

      public:
        /* -- Methods -- */

        /* Constructor */
        Tree(MemoryHandler& memoryHandler) : mRoot(nullptr), mSize(0), mMemoryHandler(memoryHandler) {}

        /* Copy constructor */
        Tree(const Tree& tree) : mRoot(nullptr), mSize(tree.mSize), mMemoryHandler(tree.mMemoryHandler) {
          copy(tree);
        }

        /* Query whether a node with a specific key is present in the tree */
        bool contains(const K& key) const {
          TreeNode* n = mRoot;

          while(n) {
            if(Hash()(key) < Hash()(n->keyValue.first)) {
              n = n->left;
            }
            else if(Hash()(key) > Hash()(n->keyValue.first)) {
              n = n->right;
            }
            else {
              while(n) {
                if(KeyEqual()(key, n->keyValue.first)) {
                  return true;
                }

                n = n->next;
              }
            }
          }

          return false;
        }

        /* Perform a key-value pair insertion into the tree */
        void insert(const Pair<K, V>& keyValue) {
          TreeNode* node = new (mMemoryHandler.allocate(sizeof(TreeNode))) TreeNode(keyValue);
          insertNode(node);
        }

        /* Perform a removal of the node with the specified key */
        void remove(const K& key) {
          removeNode(findNode(key));
        }

        /* Remove all nodes in the tree */
        void clear() {
          if(!mRoot) {
            return;
          }

          std::function<void(TreeNode* node)> clear =
          [&](TreeNode* node) -> void {
            if(!node) {
              return;
            }

            TreeNode* left = node->left;
            TreeNode* right = node->right;

            while(node) {
              TreeNode* next = node->next;
              mMemoryHandler.free(node, sizeof(TreeNode));
              node = next;
            }

            clear(left);
            clear(right);
          };

          clear(mRoot);
          mRoot = nullptr;
          mSize = 0;
        }

        /* Overloaded equality operator */
        bool operator==(const Tree& tree) const {
          if(mSize != tree.mSize) {
            return false;
          }

          std::function<bool(TreeNode*, TreeNode*)> isEqual =
          [&](TreeNode* node1, TreeNode* node2) -> bool {
            if(!node1 && !node2) {
              return true;
            }

            if(!node1 || !node2) {
              return false;
            }

            TreeNode* n = node1;
            TreeNode* k = node2;

            while(n || k) {
              if(!n || !k) {
                return false;
              }

              if(n->keyValue != k->keyValue) {
                return false;
              }

              n = n->next;
              k = k->next;
            }

            return isEqual(node1->left, node2->left) && isEqual(node1->right, node2->right);
          };

          return isEqual(mRoot, tree.mRoot);
        }

        /* Overloaded assignment operator */
        Tree& operator=(const Tree& tree) {
          if(this != &tree) {
            clear();
            copy(tree);
          }

          return *this;
        }

        /* -- Friends -- */
        
        /* Superclass */
        friend class HashMap<K, V>;
    };

    /* -- Attributes -- */

    /* Underlying red-black tree data structure for hash map */
    Tree mTree;

    /* Number of elements in the map */
    uint64 mSize;

    /* Memory handler */
    MemoryHandler& mMemoryHandler;

  public:
    /* -- Nested Classes -- */

    /* Iterator for the hash map */
    class Iterator {

      private:
        /* -- Attributes -- */

        /* Pointer to the hash map */
        const HashMap<K, V>* mMap;

        /* Pointer to a tree node */
        TreeNode* mNode;

      public:
        /* -- Methods -- */

        /* Constructor */
        Iterator() = default;

        /* Constructor */
        Iterator(const HashMap<K, V>* map, TreeNode* node) : mMap(map), mNode(node) {}

        /* Dereference */
        Pair<K, V>& operator*() const {
          assert(mNode);
          return mNode->keyValue;
        }

        /* Dereference */
        Pair<K, V>* operator->() const {
          assert(mNode);
          return &(mNode->keyValue);
        }

        /* Pre increment */
        Iterator& operator++() {
          if(mNode) {
            if(mNode->next) {
              mNode = mNode->next;
            }
            else {
              mNode = mNode->neighbour();
            }
          }

          return *this;
        }

        /* Post increment */
        Iterator operator++(int) {
          Iterator iter = *this;

          if(mNode) {
            if(mNode->next) {
              mNode = mNode->next;
            }
            else {
              mNode = mNode->neighbour();
            }
          }

          return iter;
        }

        /* Equality operator */
        bool operator==(const Iterator& iter) const {
          assert(mMap && iter.mMap);
          return mMap == iter.mMap && mNode == iter.mNode;
        }

        /* Inequality operator */
        bool operator!=(const Iterator& iter) const {
          return !((*this) == iter);
        }

        /* -- Friends -- */

        /* Superclass */
        friend class HashMap<K, V>;
    };

    /* -- Methods -- */

    /* Constructor */
    HashMap(MemoryHandler& memoryHandler);

    /* Copy constructor */
    HashMap(const HashMap<K, V>& map);

    /* Destructor */
    ~HashMap();

    /* Query whether the map contains a particular key */
    bool contains(const K& key) const;

    /* Query whether the map contains an item with the given key and return an iterator to the found item */
    Iterator find(const K& key) const;

    /* Return iterator to the first item in the map */
    Iterator begin() const;

    /* Return iterator to the end of the map */
    Iterator end() const;

    /* Get number of elements in the map */
    uint64 size() const;

    /* Insert an element into the map */
    void insert(const Pair<K, V>& keyValue);

    /* Remove an element from the map pointed to by the provided iterator */
    Iterator remove(const Iterator& iter);

    /* Remove an element from the map with the given key */
    Iterator remove(const K& key);

    /* Remove all elements in the map */
    void clear();

    /* Overloaded value access operator */
    V& operator[](const K& key);

    /* Overloaded value access operator */
    const V& operator[](const K& key) const;

    /* Overloaded equality operator */
    bool operator==(const HashMap<K, V>& map) const;

    /* Overloaded inequality operator */
    bool operator!=(const HashMap<K, V>& map) const;

    /* Overloaded assignemt operator */
    HashMap<K, V>& operator=(const HashMap<K, V>& map);
};

/* Constructor */
template<typename K, typename V, class Hash, class KeyEqual>
inline HashMap<K, V, Hash, KeyEqual>::HashMap(MemoryHandler& memoryHandler) : mTree(memoryHandler), mSize(0), mMemoryHandler(memoryHandler) {}

/* Copy constructor */
template<typename K, typename V, class Hash, class KeyEqual>
inline HashMap<K, V, Hash, KeyEqual>::HashMap(const HashMap<K, V>& map) : mTree(map.mTree), mSize(map.mSize), mMemoryHandler(map.mMemoryHandler) {}

/* Destructor */
template<typename K, typename V, class Hash, class KeyEqual>
inline HashMap<K, V, Hash, KeyEqual>::~HashMap() {
  clear();
}

/* Query whether the map contains a particular key */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool HashMap<K, V, Hash, KeyEqual>::contains(const K& key) const {
  return mTree.contains(key);
}

/* Query whether the map contains an item with the given key and return an iterator to the found item */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename HashMap<K, V, Hash, KeyEqual>::Iterator HashMap<K, V, Hash, KeyEqual>::find(const K& key) const {
  return Iterator(this, mTree.findNode(key));
}

/* Return iterator to the first item in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename HashMap<K, V, Hash, KeyEqual>::Iterator HashMap<K, V, Hash, KeyEqual>::begin() const {
  return Iterator(this, mTree.mRoot ? mTree.mRoot->min() : nullptr);
}

/* Return iterator to the end of the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename HashMap<K, V, Hash, KeyEqual>::Iterator HashMap<K, V, Hash, KeyEqual>::end() const {
  return Iterator(this, nullptr);
}

/* Get number of elements in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline uint64 HashMap<K, V, Hash, KeyEqual>::size() const {
  return mTree.size();
}

/* Insert an element into the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline void HashMap<K, V, Hash, KeyEqual>::insert(const Pair<K, V>& keyValue) {
  mTree.insert(keyValue);
}

/* Remove an element from the map pointed to by the provided iterator */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename HashMap<K, V, Hash, KeyEqual>::Iterator HashMap<K, V, Hash, KeyEqual>::remove(const Iterator& iter) {
  /* Debug: Can we use the iterators only? */
  if(!iter.mNode) {
    return end();
  }

  TreeNode* next = iter.mNode->next ? iter.mNode->next : iter.mNode->neighbour();
  mTree.remove(iter.mNode->keyValue.first);
  return Iterator(this, next);
}

/* Remove an element from the map with the given key */
template<typename K, typename V, class Hash, class KeyEqual>
inline typename HashMap<K, V, Hash, KeyEqual>::Iterator HashMap<K, V, Hash, KeyEqual>::remove(const K& key) {
  /* Debug: Can we use the iterators only */
  TreeNode* node = mTree.findNode(key);

  if(node) {
    TreeNode* next = node->next ? node->next : node->neighbour();
    mTree.remove(key);
    return Iterator(this, next);
  }
  
  return end();
}

/* Remove all elements in the map */
template<typename K, typename V, class Hash, class KeyEqual>
inline void HashMap<K, V, Hash, KeyEqual>::clear() {
  mTree.clear();
}

/* Overloaded value access operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline V& HashMap<K, V, Hash, KeyEqual>::operator[](const K& key) {
  TreeNode* node = mTree.findNode(key);

  if(!node) {
    assert(false);
    throw std::runtime_error("Invalid key");
  }

  return node->keyValue.second;
}

/* Overloaded value access operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline const V& HashMap<K, V, Hash, KeyEqual>::operator[](const K& key) const {
  TreeNode* node = mTree.findNode(key);

  if(!node) {
    assert(false);
    throw std::runtime_error("Invalid key");
  }

  return node->keyValue.second;
}

/* Overloaded equality operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool HashMap<K, V, Hash, KeyEqual>::operator==(const HashMap<K, V>& map) const {
  return (mTree == map.mTree);
}

/* Overloaded inequality operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline bool HashMap<K, V, Hash, KeyEqual>::operator!=(const HashMap<K, V>& map) const {
  return !((*this) == map);
}

/* Overloaded assignment operator */
template<typename K, typename V, class Hash, class KeyEqual>
inline HashMap<K, V>& HashMap<K, V, Hash, KeyEqual>::operator=(const HashMap<K, V>& map) {
  if(this != &map) {
    mTree = map.mTree;
  }

  return *this;
}

}

#endif