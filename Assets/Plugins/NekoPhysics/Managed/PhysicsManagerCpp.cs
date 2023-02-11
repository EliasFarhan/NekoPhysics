using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

namespace Neko
{
	public class PhysicsManagerCpp
	{
		
		[DllImport("NekoPhysics")]
		private static extern void Step(System.IntPtr instance, float dt);
		[DllImport("NekoPhysics")]
		private static extern System.IntPtr CreateWorld();

		[DllImport("NekoPhysics")]
		private static extern void DestroyWorld(System.IntPtr instance);
		
		[DllImport("NekoPhysics")]
		private static extern int CreateBody(System.IntPtr instance);

		System.IntPtr world_;

		private List<BodyCpp> bodies_;

		// Start is called before the first frame update
		public PhysicsManagerCpp()
		{
			world_ = CreateWorld();
			bodies_ = new List<BodyCpp>();
		}

		public BodyCpp AddBody()
		{
			var bodyIndex = CreateBody(world_);
			var body = new BodyCpp(world_, bodyIndex);
			bodies_.Add(body);
			return body;
		}

		public void Step(float dt)
		{
			Step(world_, dt);
		}
		~PhysicsManagerCpp()
		{
			DestroyWorld(world_);
		}
	}
}