using System;
using System.Numerics;
using System.Runtime.CompilerServices;
using BepuUtilities.Memory;
using BepuUtilities;
using BepuPhysics.CollisionDetection;

namespace BepuPhysics.Collidables
{
    /// <summary>
    /// Collision shape representing a cylinder.
    /// </summary>
    public struct Cylinder : IConvexShape
    {
        /// <summary>
        /// Radius of the cylinder.
        /// </summary>
        public float Radius;
        /// <summary>
        /// Half length of the cylinder along its local Y axis.
        /// </summary>
        public float HalfLength;

        /// <summary>
        /// Gets or sets the length of the cylinder along its local Y axis.
        /// </summary>
        public float Length { get { return HalfLength * 2; } set { HalfLength = value * 0.5f; } }

        /// <summary>
        /// Creates a cylinder shape.
        /// </summary>
        /// <param name="radius">Radius of the cylinder.</param>
        /// <param name="length">Length of the cylinder along its local Y axis.</param>
        public Cylinder(float radius, float length)
        {
            Radius = radius;
            HalfLength = length * 0.5f;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void ComputeAngularExpansionData(out float maximumRadius, out float maximumAngularExpansion)
        {
            maximumRadius = (float)Math.Sqrt(HalfLength * HalfLength + Radius * Radius);
            maximumAngularExpansion = maximumRadius - Math.Min(HalfLength, Radius);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public readonly void ComputeBounds(Quaternion orientation, out Vector3 min, out Vector3 max)
        {
            //The bounding box is composed of the contribution from the axis line segment and the disc cap.
            //The bounding box of the disc cap can be found by sampling the extreme point in each of the three directions:
            //extremePointDirection = sampleDirection - dot(sampleDirection, axis) * axis
            //extremePoint = extremePointDirection * radius / extremePointDirection.Length()
            //offsetAlongSampleDirection = dot(extremePoint, sampleDirection)
            //This simplifies:
            //offsetAlongSampleDirection = dot(extremePointDirection * radius / extremePointDirection.Length(), sampleDirection)
            //offsetAlongSampleDirection = dot((sampleDirection - dot(sampleDirection, axis) * axis) * radius / extremePointDirection.Length(), sampleDirection)
            //dot(sampleDirection, sampleDirection - dot(sampleDirection, axis) * axis) = 1 - dot(sampleDirection, axis)^2
            //offsetAlongSampleDirection = (1 - dot(sampleDirection, axis)^2) * radius / extremePointDirection.Length()
            //extremePointDirection.Length() = sqrt(dot(sampleDirection - dot(sampleDirection, axis) * axis, sampleDirection - dot(sampleDirection, axis) * axis)
            //extremePointDirection.Length() = sqrt(1 - dot(sampleDirection, axis)^2 - dot(sampleDirection, axis)^2 + dot(sampleDirection, axis)^2 * 1
            //extremePointDirection.Length() = sqrt(1 - dot(sampleDirection, axis)^2
            //offsetAlongSampleDirection = (1 - dot(sampleDirection, axis)^2) * radius / sqrt(1 - dot(sampleDirection, axis)^2)
            //offsetAlongSampleDirection = sqrt(1 - dot(sampleDirection, axis)^2) * radius

            //We want to compute 3 different sample directions: (1,0,0), (0,1,0), and (0,0,1). This is equivalent to simply accessing the component X Y or Z out of the axis.
            //Using this fact, we can compute all three directions together:
            QuaternionEx.TransformUnitZ(orientation, out var z); // TODO: Z-up
            var positiveDiscBoundOffsets = Vector3.SquareRoot(Vector3.Max(Vector3.Zero, Vector3.One - z * z)) * Radius; // TODO: Z-up
            max = Vector3.Abs(HalfLength * z) + positiveDiscBoundOffsets; // TODO: Z-up
            //Cylinders are symmetric.
            min = -max;
        }


        public readonly bool RayTest(in RigidPose pose, Vector3 origin, Vector3 direction, out float t, out Vector3 normal)
        {
            //It's convenient to work in local space, so pull the ray into the cylinder's local space.
            Matrix3x3.CreateFromQuaternion(pose.Orientation, out var orientation);
            var o = origin - pose.Position;
            Matrix3x3.TransformTranspose(o, orientation, out o);
            Matrix3x3.TransformTranspose(direction, orientation, out var d);

            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            var inverseDLength = 1f / d.Length();
            d *= inverseDLength;

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            var tOffset = float.Max(0, -Vector3.Dot(o, d) - (HalfLength + Radius));
            o += d * tOffset;
            var oh = o with { Z = 0 }; // TODO: Z-up
            var dh = d with { Z = 0 }; // TODO: Z-up
            var a = Vector3.Dot(dh, dh);
            var b = Vector3.Dot(oh, dh);
            var radiusSquared = Radius * Radius;
            var c = Vector3.Dot(oh, oh) - radiusSquared;
            if (b > 0 && c > 0)
            {
                //Ray is outside and pointing away, no hit.
                t = 0;
                normal = new Vector3();
                return false;
            }

            float discY;
            if (a > 1e-8f)
            {
                var discriminant = b * b - a * c;
                if (discriminant < 0)
                {
                    //The infinite cylinder isn't hit, so the finite cylinder can't be hit.
                    t = 0;
                    normal = new Vector3();
                    return false;
                }
                t = (-b - MathF.Sqrt(discriminant)) / a;
                t = float.Max(t, -tOffset);
                var cylinderHitLocation = o + d * t;
                if (cylinderHitLocation.Z < -HalfLength) // TODO: Z-up
                {
                    discY = -HalfLength;
                }
                else if (cylinderHitLocation.Z > HalfLength) // TODO: Z-up
                {
                    discY = HalfLength;
                }
                else
                {
                    //The hit is on the side of the cylinder.
                    normal = cylinderHitLocation with { Z = 0 } / Radius; // TODO: Z-up
                    Matrix3x3.Transform(normal, orientation, out normal);
                    t = (t + tOffset) * inverseDLength;
                    return true;
                }
            }
            else
            {
                //The ray is parallel to the axis; the impact is on a disc or nothing.
                //If the ray is inside the cylinder, we want t = 0, so just set the discY to match the ray's origin in that case and it'll shake out like we want.
                discY = float.MinMagnitude(d.Z > 0 ? -HalfLength : HalfLength, o.Z); // TODO: Z-up
            }

            //Intersect the ray with the plane anchored at discY with normal equal to (0,1,0).
            //t = dot(rayOrigin - (0,discY,0), (0,1,0)) / dot(rayDirection, (0,1,0)
            if (float.Abs(o.Z) > HalfLength && o.Z * d.Z >= 0) // TODO: Z-up
            {
                //The ray can only hit the disc if the ray is inside the cylinder or the direction points toward the cylinder.
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = (discY - o.Z) / d.Z; // TODO: Z-up
            var hitLocation = o + d * t;
            if (hitLocation.X * hitLocation.X + hitLocation.Y * hitLocation.Y > radiusSquared) // TODO: Z-up
            {
                //The hit missed the cap.
                t = 0;
                normal = new Vector3();
                return false;
            }
            t = (t + tOffset) * inverseDLength;
            normal = d.Z < 0 ? orientation.Z : -orientation.Z; // TODO: Z-up
            return true;
        }

        public readonly BodyInertia ComputeInertia(float mass)
        {
            BodyInertia inertia;
            inertia.InverseMass = 1f / mass;
            float diagValue = inertia.InverseMass / ((4 * .0833333333f) * HalfLength * HalfLength + .25f * Radius * Radius);
            inertia.InverseInertiaTensor.XX = diagValue;
            inertia.InverseInertiaTensor.YX = 0;
            inertia.InverseInertiaTensor.YY = diagValue;
            inertia.InverseInertiaTensor.ZX = 0;
            inertia.InverseInertiaTensor.ZY = 0;
            inertia.InverseInertiaTensor.ZZ = 2f * inertia.InverseMass / (Radius * Radius); // TODO: Z-up
            return inertia;
        }

        public static ShapeBatch CreateShapeBatch(BufferPool pool, int initialCapacity, Shapes shapeBatches)
        {
            return new ConvexShapeBatch<Cylinder, CylinderWide>(pool, initialCapacity);
        }

        /// <summary>
        /// Type id of cylinder shapes.
        /// </summary>
        public const int Id = 4;
        public static int TypeId { [MethodImpl(MethodImplOptions.AggressiveInlining)] get { return Id; } }
    }

    public struct CylinderWide : IShapeWide<Cylinder>
    {
        public Vector<float> Radius;
        public Vector<float> HalfLength;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void Broadcast(in Cylinder shape)
        {
            Radius = new Vector<float>(shape.Radius);
            HalfLength = new Vector<float>(shape.HalfLength);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteFirst(in Cylinder source)
        {
            Unsafe.As<Vector<float>, float>(ref Radius) = source.Radius;
            Unsafe.As<Vector<float>, float>(ref HalfLength) = source.HalfLength;
        }

        public bool AllowOffsetMemoryAccess => true;
        public int InternalAllocationSize => 0;
        public void Initialize(in Buffer<byte> memory) { }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void WriteSlot(int index, in Cylinder source)
        {
            GatherScatter.GetOffsetInstance(ref this, index).WriteFirst(source);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetBounds(ref QuaternionWide orientations, int countInBundle, out Vector<float> maximumRadius, out Vector<float> maximumAngularExpansion, out Vector3Wide min, out Vector3Wide max)
        {
            var z = QuaternionWide.TransformUnitZ(orientations); // TODO: Z-up
            Vector3Wide.Multiply(z, z, out var zz); // TODO: Z-up
            Vector3Wide.Subtract(Vector<float>.One, zz, out var squared); // TODO: Z-up
            max.X = Vector.Abs(HalfLength * z.X) + Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squared.X)) * Radius;
            max.Y = Vector.Abs(HalfLength * z.Y) + Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squared.Y)) * Radius;
            max.Z = Vector.Abs(HalfLength * z.Z) + Vector.SquareRoot(Vector.Max(Vector<float>.Zero, squared.Z)) * Radius;
            //Cylinders are symmetric.
            Vector3Wide.Negate(max, out min);

            maximumRadius = Vector.SquareRoot(HalfLength * HalfLength + Radius * Radius);
            maximumAngularExpansion = maximumRadius - Vector.Min(HalfLength, Radius);
        }

        public static int MinimumWideRayCount
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get
            {
                return 2;
            }
        }

        public void RayTest(ref RigidPoseWide pose, ref RayWide ray, out Vector<int> intersected, out Vector<float> t, out Vector3Wide normal)
        {
            //It's convenient to work in local space, so pull the ray into the capsule's local space.
            Matrix3x3Wide.CreateFromQuaternion(pose.Orientation, out var orientation);
            Vector3Wide.Subtract(ray.Origin, pose.Position, out var oWorld);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(oWorld, orientation, out var o);
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(ray.Direction, orientation, out var d);

            //Normalize the direction. Sqrts aren't *that* bad, and it both simplifies things and helps avoid numerical problems.
            Vector3Wide.Length(d, out var dLength);
            var inverseDLength = Vector<float>.One / dLength;
            Vector3Wide.Scale(d, inverseDLength, out d);

            //Move the origin up to the earliest possible impact time. This isn't necessary for math reasons, but it does help avoid some numerical problems.
            Vector3Wide.Dot(o, d, out var od);
            var tOffset = Vector.Max(-od - (HalfLength + Radius), Vector<float>.Zero);
            Vector3Wide.Scale(d, tOffset, out var oOffset);
            Vector3Wide.Add(o, oOffset, out o);
            var a = d.X * d.X + d.Y * d.Y; // TODO: Z-up
            var b = o.X * d.X + o.Y * d.Y;
            var radiusSquared = Radius * Radius;
            var c = (o.X * o.X + o.Y * o.Y) - radiusSquared; // TODO: Z-up

            var rayIsntParallel = Vector.GreaterThan(a, new Vector<float>(1e-8f));
            var discriminant = b * b - a * c;
            var cylinderIntersected = Vector.BitwiseAnd(
                Vector.BitwiseOr(
                    Vector.LessThanOrEqual(b, Vector<float>.Zero),
                    Vector.LessThanOrEqual(c, Vector<float>.Zero)),
                Vector.GreaterThanOrEqual(discriminant, Vector<float>.Zero));
            var cylinderT = Vector.Max(-tOffset, (-b - Vector.SquareRoot(discriminant)) / a);
            Vector3Wide.Scale(d, cylinderT, out oOffset);
            Vector3Wide.Add(o, oOffset, out var cylinderHitLocation);
            var inverseRadius = Vector<float>.One / Radius;
            var cylinderNormalX = cylinderHitLocation.X * inverseRadius;
            var cylinderNormalY = cylinderHitLocation.Y * inverseRadius; // TODO: Z-up
            var useCylinder = Vector.BitwiseAnd(Vector.GreaterThanOrEqual(cylinderHitLocation.Z, -HalfLength), Vector.LessThanOrEqual(cylinderHitLocation.Z, HalfLength)); // TODO: Z-up

            //Intersect the disc cap for any lane which ended up not using the cylinder.
            Vector<float> discZ = Vector.ConditionalSelect(
                Vector.BitwiseOr(
                    Vector.BitwiseAnd(Vector.GreaterThan(cylinderHitLocation.Z, HalfLength), rayIsntParallel), // TODO: Z-up
                    Vector.AndNot(Vector.LessThanOrEqual(d.Z, Vector<float>.Zero), rayIsntParallel)), HalfLength, -HalfLength); // TODO: Z-up

            //Intersect the ray with the plane anchored at discY with normal equal to (0,1,0).
            //t = dot(rayOrigin - (0,discY,0), (0,1,0)) / dot(rayDirection, (0,1,0)
            //The ray can only hit the disc if the ray is inside the cylinder or the direction points toward the cylinder.
            var withinDiscsOrRayPointsTowardDisc = Vector.BitwiseOr(Vector.LessThanOrEqual(Vector.Abs(o.Z), HalfLength), Vector.LessThan(o.Z * d.Z, Vector<float>.Zero)); // TODO: Z-up

            var capT = (discZ - o.Z) / d.Z; // TODO: Z-up

            var hitLocationX = o.X + d.X * capT; // TODO: Z-up
            var hitLocationZ = o.Y + d.Y * capT;
            var capHitWithinRadius = Vector.LessThanOrEqual(hitLocationX * hitLocationX + hitLocationZ * hitLocationZ, radiusSquared);
            var hitCap = Vector.BitwiseAnd(withinDiscsOrRayPointsTowardDisc, capHitWithinRadius);

            t = (tOffset + Vector.ConditionalSelect(useCylinder, cylinderT, Vector.ConditionalSelect(hitCap, capT, Vector<float>.Zero))) * inverseDLength;
            var capUsesUpwardFacingNormal = Vector.LessThan(d.Z, Vector<float>.Zero); // TODO: Z-up
            Vector3Wide localNormal;
            localNormal.X = Vector.ConditionalSelect(useCylinder, cylinderNormalX, Vector<float>.Zero);
            localNormal.Y = Vector.ConditionalSelect(useCylinder, cylinderNormalY, Vector<float>.Zero);
            localNormal.Z = Vector.ConditionalSelect(useCylinder, Vector<float>.Zero, Vector.ConditionalSelect(capUsesUpwardFacingNormal, Vector<float>.One, new Vector<float>(-1))); // TODO: Z-up
            Matrix3x3Wide.TransformWithoutOverlap(localNormal, orientation, out normal);
            intersected = Vector.ConditionalSelect(useCylinder, cylinderIntersected, hitCap);
        }
    }

    public struct CylinderSupportFinder : ISupportFinder<Cylinder, CylinderWide>
    {
        public bool HasMargin
        {
            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            get { return false; }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void GetMargin(in CylinderWide shape, out Vector<float> margin)
        {
            margin = default;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeSupport(in CylinderWide shape, in Matrix3x3Wide orientation, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            Matrix3x3Wide.TransformByTransposedWithoutOverlap(direction, orientation, out var localDirection);
            ComputeLocalSupport(shape, localDirection, terminatedLanes, out var localSupport);
            Matrix3x3Wide.TransformWithoutOverlap(localSupport, orientation, out support);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public void ComputeLocalSupport(in CylinderWide shape, in Vector3Wide direction, in Vector<int> terminatedLanes, out Vector3Wide support)
        {
            support.Z = Vector.ConditionalSelect(Vector.GreaterThan(direction.Z, Vector<float>.Zero), shape.HalfLength, -shape.HalfLength); // TODO: Z-up
            //TODO: Using a hardware accelerated reciprocal sqrt approximation would be hugely beneficial here.
            //It would actually be meaningful to full frame time in simulations that rely on cylinders.
            var horizontalLength = Vector.SquareRoot(direction.X * direction.X + direction.Y * direction.Y); // TODO: Z-up
            var normalizeScale = shape.Radius / horizontalLength;
            var useHorizontal = Vector.GreaterThan(horizontalLength, new Vector<float>(1e-8f));
            support.X = Vector.ConditionalSelect(useHorizontal, direction.X * normalizeScale, Vector<float>.Zero); // TODO: Z-up
            support.Y = Vector.ConditionalSelect(useHorizontal, direction.Y * normalizeScale, Vector<float>.Zero);
        }
    }
}
