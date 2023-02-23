#pragma once
#include "physics/physics_type.h"

namespace neko
{

class ContactListener
{
public:
    virtual ~ContactListener() = default;
    virtual void OnTriggerEnter(const TriggerPair& p) = 0;
    virtual void OnTriggerExit(const TriggerPair& p) = 0;
};



}
