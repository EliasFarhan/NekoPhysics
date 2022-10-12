using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Neko
{

public struct Body
{
    public Vec2f position;
    public Vec2f velocity;
    public float angle;
    public float angularVelocity;

    Body(Vec2f position)
    {
        this.position = position;
        velocity = Vec2f.zero;
        angle = 0.0f;
        angularVelocity = 0.0f;
    }
}
}