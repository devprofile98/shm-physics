#ifndef CYCLON_COLLIDE_FINE_H
#define CYCLON_COLLIDE_FINE_H

#include "contacts.hpp"

namespace cyclon {

struct CollisionData{
    Contact *contacts;
    unsigned contactLeft;
};

}


#endif //CYCLON_COLLIDE_FINE_H
