using System.Runtime.InteropServices;

namespace Neko
{
    public class BodyCpp
    {
        
        System.IntPtr world_;
        private int bodyIndex_ = -1;
        
        [DllImport("NekoPhysics")]
        private static extern System.IntPtr GetBody(System.IntPtr instance, int index);
        
        [DllImport("NekoPhysics")]
        private static extern void SetPosition(System.IntPtr instance, Vector2f newPos);
        
        [DllImport("NekoPhysics")]
        private static extern void SetVelocity(System.IntPtr instance, Vector2f newVel);
        
        [DllImport("NekoPhysics")]
        private static extern void AddForce(System.IntPtr instance, Vector2f force);
        
        [DllImport("NekoPhysics")]
        private static extern void SetMass(System.IntPtr instance, float mass);
        
        [DllImport("NekoPhysics")]
        private static extern Vector2f GetPosition(System.IntPtr instance);
        
        [DllImport("NekoPhysics")]
        private static extern Vector2f GetVelocity(System.IntPtr instance);
        
        [DllImport("NekoPhysics")]
        private static extern Vector2f GetForce(System.IntPtr instance);
        
        [DllImport("NekoPhysics")]
        private static extern float GetMass(System.IntPtr instance);
        
        public BodyCpp(System.IntPtr world, int bodyIndex)
        {
            world_ = world;
            bodyIndex_ = bodyIndex;
        }
        private System.IntPtr BodyPtr => GetBody(world_, bodyIndex_);
        
        public void AddForce(Vector2f force)
        {
            AddForce(BodyPtr, force);
        }

        public Vector2f Force => GetForce(BodyPtr);

        public Vector2f Position
        {
            get => GetPosition(BodyPtr);
            set => SetPosition(BodyPtr, value);
        }

        public Vector2f Velocity
        {
            get => GetVelocity(BodyPtr);
            set => SetVelocity(BodyPtr, value);
        }

        public float Mass
        {
            get => GetMass(BodyPtr);
            set => SetMass(BodyPtr, value);
        }

    }
}