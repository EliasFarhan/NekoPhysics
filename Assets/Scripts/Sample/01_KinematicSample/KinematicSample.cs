using System;
using System.Collections;
using System.Collections.Generic;
using Neko;
using UnityEngine;

public class KinematicSample : MonoBehaviour
{
    [SerializeField] private Body body;

    private const float maxX = 9.0f;
    private const float maxY = 5.0f;
    private void Start()
    {
        body.Velocity = new Vector2f(UnityEngine.Random.Range(-2.0f, 2.0f), UnityEngine.Random.Range(-2.0f, 2.0f));
    }

    private void Update()
    {
        if ((body.Position.x > maxX && body.Velocity.x > 0.0f) || (body.Position.x < -maxX && body.Velocity.x < 0.0f))
        {
            var vel = body.Velocity;
            vel.x = -vel.x;
            body.Velocity = vel;
        }
        if ((body.Position.y > maxY && body.Velocity.y > 0.0f) || (body.Position.y < -maxY && body.Velocity.y < 0.0f))
        {
            var vel = body.Velocity;
            vel.y = -vel.y;
            body.Velocity = vel;
        }
    }
}
