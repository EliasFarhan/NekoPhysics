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

    private Transform transform_;
    private void Start()
    {
        physicsManager_ = FindObjectOfType<PhysicsManager>();
        bodyCpp_ = physicsManager_.physicsManagerPtr.AddBody();
        bodyCpp_.Mass = 1.0f;
        
        transform_ = transform;
    }

    private void Update()
    {
        transform_.position = new Vector3(Position.x, Position.y, 0.0f);
    }
}
