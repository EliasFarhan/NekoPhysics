using System.Runtime.InteropServices;
using UnityEngine;
using Neko;

public class PhysicsManager : MonoBehaviour
{
    private PhysicsManagerCpp physicsManager_;

    System.IntPtr world_;

    public PhysicsManagerCpp physicsManagerPtr => physicsManager_;
    // Start is called before the first frame update
    void Awake()
    {
        physicsManager_ = new PhysicsManagerCpp();
    }
    
    
}
