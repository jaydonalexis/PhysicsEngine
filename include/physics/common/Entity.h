#ifndef PHYSICS_ENTITY_H
#define PHYSICS_ENTITY_H

#include <physics/Configuration.h>

namespace physics {
  
// https://bitsquid.blogspot.com/2014/08/building-data-oriented-entity-system.html
struct Entity {

  private:
    /* -- Constants -- */

    static const uint32 NUM_INDEX_BITS = 24;
    static const uint32 INDEX_MASK = (1 << NUM_INDEX_BITS) - 1;
    static const uint32 NUM_GENERATION_BITS = 8;
    static const uint32 GENERATION_MASK = (1 << NUM_GENERATION_BITS) - 1;
    static const uint32 NUM_MIN_FREE_INDICES = 1024;

  public:
    /* -- Attributes -- */

    /* ID */
    uint32 identifier;

    /* -- Methods -- */

    /* Constructor */
    Entity(uint32 index, uint32 generation);

    /* Index portion of identifier */
    uint32 getIndex() const;

    /* Generation portion of identifier */
    uint32 getGeneration() const;

    /* Overloaded equality operator */
    bool operator==(const Entity& entity) const;

    /* Overloaded inequality operator */
    bool operator!=(const Entity& Entity) const;

    /* -- Friends -- */

    friend class EntityHandler;
};

/* Constructor */
inline Entity::Entity(uint32 index, uint32 generation) : identifier((index & INDEX_MASK) | (generation & GENERATION_MASK) << NUM_INDEX_BITS) {}

/* Index portion of identifier*/
inline uint32 Entity::getIndex() const {
  return identifier & INDEX_MASK;
}

/* Generation portion of identifier */
inline uint32 Entity::getGeneration() const {
  return identifier & GENERATION_MASK;
}

/* Overloaded equality operator */
inline bool Entity::operator==(const Entity& entity) const {
  return identifier == entity.identifier;
}

/* Overloaded inequality operator */
inline bool Entity::operator!=(const Entity& entity) const {
  return identifier != entity.identifier;
}

}

namespace std {
  
template<> 
struct hash<physics::Entity> {

  public:
    /* -- Methods -- */

    /* Hash function for pair collector */
    size_t operator()(const physics::Entity& entity) const {
      return hash<physics::uint32>{}(entity.identifier);
    }
};

}

#endif