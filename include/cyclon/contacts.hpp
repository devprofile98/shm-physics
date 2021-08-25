#ifndef CYCLON_CONTACTS_H
#define CYCLON_CONTACTS_H

#include "core.hpp"

namespace cyclon {

// data that is needed to be with contacts generated
class Contact{
    Vector3 contactPoint;
    Vector3 contactNormal;
    real penetration;
};
}

#endif //CYCLON_CONTACTS_H
