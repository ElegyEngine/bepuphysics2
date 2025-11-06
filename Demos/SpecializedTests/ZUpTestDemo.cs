using System.Numerics;
using BepuPhysics;
using BepuPhysics.Collidables;
using BepuUtilities;
using DemoContentLoader;
using DemoRenderer;

namespace Demos;

public class ZUpTestDemo : Demo
{
	public override void Initialize( ContentArchive content, Camera camera )
	{
		camera.Position = new( 0.0f, 5.0f, 0.0f );
		camera.Pitch = MathHelper.Pi * 0.15f;

		Simulation = Simulation.Create( BufferPool, new DemoNarrowPhaseCallbacks(),
			new DemoPoseIntegratorCallbacks( new( 0, -10, 0 ) ), new SolveDescription( 8, 1 ) );

		// Baseplate
		var floor = new Box( 10.0f, 1.0f, 10.0f );
		var floorIndex = Simulation.Shapes.Add( floor );
		Simulation.Statics.Add( new( Vector3.Zero, Quaternion.Identity, floorIndex ) );

		// Objects
		var box = new Box( 4.0f, 1.0f, 1.0f );

		Simulation.Bodies.Add(
			BodyDescription.CreateConvexDynamic( 
				(Vector3.UnitY * 5.0f, Quaternion.Identity), 10.0f, Simulation.Shapes, box ) );

		
	}
}
