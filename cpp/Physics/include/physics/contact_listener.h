#pragma once
#include "physics/physics_type.h"

namespace neko
{

class ContactListener
{
public:
    virtual ~ContactListener() = default;
    virtual void OnTriggerEnter(const ColliderPair& p) = 0;
    virtual void OnTriggerExit(const ColliderPair& p) = 0;
    virtual void OnCollisionEnter(const ColliderPair& p) = 0;
    virtual void OnCollisionExit(const ColliderPair& p) = 0;
};



}
