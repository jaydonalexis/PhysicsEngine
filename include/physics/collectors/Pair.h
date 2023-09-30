#ifndef PHYSICS_PAIR_H
#define PHYSICS_PAIR_H

#include <physics/Configuration.h>
#include <physics/memory/MemoryHandler.h>
#include <cstring>
#include <iterator>

namespace physics {

template<typename T1, typename T2>
class Pair {

  public:
    /* -- Attributes -- */
    
    /* First element */
    T1 first;

    /* Second element */
    T2 second;

    /* -- Methods -- */

    /* Constructor */
    Pair(const T1& first, const T2& second);

    /* Overloaded equality operator */
    bool operator==(const Pair<T1, T2>& pair);

    /* Overloaded inequality operator */
    bool operator!=(const Pair<T1, T2>& pair);
};

/* Constructor */
template<typename T1, typename T2>
inline Pair<T1, T2>::Pair(const T1& first, const T2& second) : first(first), second(second) {}

/* Overloaded equality operator */
template<typename T1, typename T2>
inline bool Pair<T1, T2>::operator==(const Pair<T1, T2>& pair) {
  return first == pair.first && second == pair.second;
}

/* Overloaded inequality operator */
template<typename T1, typename T2>
inline bool Pair<T1, T2>::operator!=(const Pair<T1, T2>& pair) {
  return !((*this) == pair);
}

}

namespace std {
  
template<typename T1, typename T2> 
struct hash<physics::Pair<T1, T2>> {

  public:
    /* -- Methods -- */

    /* Hash function for pair collector */
    size_t operator()(const physics::Pair<T1, T2>& pair) const {
      size_t hash1 = hash<T1>()(pair.first);
      size_t hash2 = hash<T2>()(pair.second);
      return hash1 ^ (hash2 + 0x9e3779b9 + (hash1 << 6) + (hash1 >> 2));
    }
};

}

#endif