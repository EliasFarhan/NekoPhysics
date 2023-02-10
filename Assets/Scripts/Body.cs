using System;
using Neko;
using UnityEngine;
public class Body : MonoBehaviour
{
    private BodyCpp bodyCpp_;

    public Vector2f Position
    {
        get => bodyCpp_.Position;
        set => bodyCpp_.Position = value;
    }
    
    public Vector2f Velocity
    {
        get => bodyCpp_.Velocity;
        set => bodyCpp_.Velocity = value;
    }

    private PhysicsManager physicsManager_;
    private void Start()
    {
        physicsManager_ = FindObjectOfType<PhysicsManager>();
        bodyCpp_ = physicsManager_.physicsManagerPtr.AddBody();
        bodyCpp_.Mass = 1.0f;
    }

}
