using System;
using System.Runtime.InteropServices;
using UnityEngine;
using Neko;

public class PhysicsManager : MonoBehaviour
{
    private PhysicsManagerCpp physicsManager_;
    public PhysicsManagerCpp physicsManagerPtr => physicsManager_;

    [SerializeField] private bool useFixedDeltaTime;
    // Start is called before the first frame update
    void Awake()
    {
        physicsManager_ = new PhysicsManagerCpp();
    }

    private void Update()
    {
        if (!useFixedDeltaTime)
        {
            physicsManager_.Step(Time.deltaTime);
        }
    }

    private void FixedUpdate()
    {
        if (useFixedDeltaTime)
        {
            physicsManager_.Step(Time.fixedDeltaTime);
        }
    }
}
