using System.Runtime.InteropServices;
using UnityEngine;
using Neko;

public class PhysicsManager : MonoBehaviour
{
    private PhysicsManagerCpp physicsManager_;

    System.IntPtr world_;

    // Start is called before the first frame update
    void Start()
    {
        physicsManager_ = new PhysicsManagerCpp();
        
    }
}
