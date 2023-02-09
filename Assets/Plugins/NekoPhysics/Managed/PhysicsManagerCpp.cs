using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.Runtime.InteropServices;

namespace Neko
{
	public class PhysicsManagerCpp
	{
		[DllImport("NekoPhysics")]
		private static extern System.IntPtr CreateWorld();

		[DllImport("NekoPhysics")]
		private static extern void DestroyWorld(System.IntPtr instance);

		System.IntPtr world_;

		// Start is called before the first frame update
		public PhysicsManagerCpp()
		{
			world_ = CreateWorld();
		}

		~PhysicsManagerCpp()
		{
			DestroyWorld(world_);
		}
	}
}