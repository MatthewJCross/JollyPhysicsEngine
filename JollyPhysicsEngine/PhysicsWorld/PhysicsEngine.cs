using System.Windows;
using JollyPhysicsEngine.PhysicsWorld;

namespace JollyPhysicsEngine.Physics
{
    public class PhysicsEngine
    {
        private List<PhysicsObject> objects = new List<PhysicsObject>();
        private Vector gravity = new Vector(0, 500); // pixels per second squared
        private double width;
        private double height;

        // Damping constants
        private const double AIR_RESISTANCE = 0.995;
        private const double SURFACE_FRICTION = 0.98;
        private const double ROLLING_RESISTANCE = 0.99;

        // Ground detection threshold
        private const double GROUND_TOLERANCE = 2.0; // pixels
        private const double RESTING_VELOCITY_THRESHOLD = 10.0; // pixels per second
        private const double RESTING_ANGULAR_THRESHOLD = 0.1; // radians per second

        public event EventHandler<CollisionEventArgs> CollisionDetected;

        public PhysicsEngine(double width, double height)
        {
            this.width = width;
            this.height = height;
        }

        public void AddObject(PhysicsObject obj)
        {
            objects.Add(obj);
        }

        public void RemoveObject(PhysicsObject obj)
        {
            objects.Remove(obj);
        }

        public void ClearObjects()
        {
            objects.Clear();
        }

        public List<PhysicsObject> GetObjects()
        {
            return objects;
        }

        public void Update(double deltaTime, bool enableGravity, bool enableCollisions, int subSteps = 2)
        {
            if (deltaTime > 0.1) deltaTime = 0.1;

            double subDeltaTime = deltaTime / subSteps;

            for (int step = 0; step < subSteps; step++)
            {
                // Apply gravity
                if (enableGravity)
                {
                    foreach (var obj in objects)
                    {
                        if (!obj.IsStatic)
                        {
                            obj.ApplyForce(gravity * obj.Mass, subDeltaTime);
                        }
                    }
                }

                // Apply spin damping
                foreach (var obj in objects)
                {
                    if (!obj.IsStatic && Math.Abs(obj.AngularVelocity) > 0.001)
                    {
                        ApplySpinDamping(obj, subDeltaTime);
                    }
                }

                // Update positions and rotations
                foreach (var obj in objects)
                {
                    obj.UpdatePosition(subDeltaTime);
                }

                // Check ground contact for flat surfaces
                CheckGroundContact();

                // Check collisions
                if (enableCollisions)
                {
                    CheckCollisions();
                }

                // Check boundaries
                CheckBoundaries();
            }
        }

        private void CheckGroundContact()
        {
            foreach (var obj in objects)
            {
                if (obj.IsStatic) 
                    continue;

                // Get the bottom-most point of the object
                double lowestPoint = GetLowestPoint(obj);

                // Check if object is on or slightly below ground
                if (lowestPoint >= height - GROUND_TOLERANCE)
                {
                    HandleGroundContact(obj);
                }
            }
        }

        private double GetLowestPoint(PhysicsObject obj)
        {
            double lowestY = double.MinValue;

            if (obj is Box box)
            {
                // For box, lowest point is bottom edge
                Rect bounds = box.GetBounds();
                lowestY = bounds.Y + bounds.Height;
            }
            else if (obj is Triangle triangle)
            {
                // For triangle, check all vertices
                Point[] points = triangle.GetTransformedPoints();
                foreach (Point p in points)
                {
                    lowestY = Math.Max(lowestY, p.Y);
                }
            }
            else if (obj is PolygonObject polygon)
            {
                // For polygon, check all vertices
                Point[] points = polygon.GetTransformedPoints();
                foreach (Point p in points)
                {
                    lowestY = Math.Max(lowestY, p.Y);
                }
            }
            else if (obj is Ball ball)
            {
                // For ball, lowest point is bottom of circle
                lowestY = ball.Position.Y + ball.Radius * 2;
            }

            return lowestY;
        }

        private void HandleGroundContact(PhysicsObject obj)
        {
            // Check if object is really resting (very low velocity)
            bool isResting = Math.Abs(obj.Velocity.Y) < RESTING_VELOCITY_THRESHOLD && Math.Abs(obj.Velocity.X) < RESTING_VELOCITY_THRESHOLD * 2 && Math.Abs(obj.AngularVelocity) < RESTING_ANGULAR_THRESHOLD;

            if (isResting)
            {
                // Object is resting on ground
                obj.Velocity = new Vector(obj.Velocity.X * 0.95, 0);
                obj.AngularVelocity *= 0.95;

                // For objects with flat bottoms, ensure they're level
                if (obj is Box box)
                {
                    // Boxes should sit flat - snap to nearest 90 degrees
                    double pi = Math.PI;
                    double targetRotation = Math.Round(box.Rotation / (pi / 2)) * (pi / 2);
                    box.Rotation = targetRotation;
                }
                else if (obj is Triangle triangle)
                {
                    // Triangles should sit on their base
                    Point[] points = triangle.GetTransformedPoints();

                    // Find the lowest edge (should be the base)
                    int lowestEdgeIndex = -1;
                    double lowestEdgeY = double.MinValue;

                    for (int i = 0; i < 3; i++)
                    {
                        Point p1 = points[i];
                        Point p2 = points[(i + 1) % 3];

                        double edgeY = Math.Max(p1.Y, p2.Y);
                        if (edgeY > lowestEdgeY)
                        {
                            lowestEdgeY = edgeY;
                            lowestEdgeIndex = i;
                        }
                    }

                    if (lowestEdgeIndex >= 0)
                    {
                        // Make this edge horizontal
                        Point p1 = points[lowestEdgeIndex];
                        Point p2 = points[(lowestEdgeIndex + 1) % 3];

                        double edgeAngle = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X);
                        triangle.Rotation = -edgeAngle;
                    }
                }
                else if (obj is PolygonObject polygon)
                {
                    // For regular polygons, find the lowest edge
                    Point[] points = polygon.GetTransformedPoints();

                    int lowestEdgeIndex = -1;
                    double lowestEdgeY = double.MinValue;

                    for (int i = 0; i < points.Length; i++)
                    {
                        Point p1 = points[i];
                        Point p2 = points[(i + 1) % points.Length];

                        double edgeY = Math.Max(p1.Y, p2.Y);
                        if (edgeY > lowestEdgeY)
                        {
                            lowestEdgeY = edgeY;
                            lowestEdgeIndex = i;
                        }
                    }

                    if (lowestEdgeIndex >= 0)
                    {
                        Point p1 = points[lowestEdgeIndex];
                        Point p2 = points[(lowestEdgeIndex + 1) % points.Length];

                        double edgeAngle = Math.Atan2(p2.Y - p1.Y, p2.X - p1.X);
                        polygon.Rotation = -edgeAngle;
                    }
                }
            }
            else
            {
                // Object is still moving - let it bounce naturally
                // Don't modify velocity here - let CheckBoundaries handle the bounce
            }
        }

        private void ApplySpinDamping(PhysicsObject obj, double deltaTime)
        {
            // Base air resistance
            double airDamping = Math.Pow(AIR_RESISTANCE, deltaTime * 60);

            // Size-based air resistance
            double size = Math.Max(obj.Width, obj.Height);
            double sizeFactor = Math.Min(1.0, size / 100);
            double sizeDamping = 1.0 - (0.01 * sizeFactor * deltaTime * 60);

            // Combined air resistance
            double totalAirDamping = airDamping * sizeDamping;

            // Apply air resistance
            obj.AngularVelocity *= totalAirDamping;

            // Check if object is near ground
            double lowestPoint = GetLowestPoint(obj);
            bool nearGround = lowestPoint >= height - GROUND_TOLERANCE * 2;

            if (nearGround)
            {
                // Ground friction significantly reduces spin
                obj.AngularVelocity *= 0.95;

                // Additional scraping effect for fast spin
                if (Math.Abs(obj.AngularVelocity) > 2)
                {
                    obj.AngularVelocity *= 0.98;
                }
            }

            // Minimum spin threshold
            if (Math.Abs(obj.AngularVelocity) < 0.01)
            {
                obj.AngularVelocity = 0;
            }
        }

        private void CheckCollisions()
        {
            for (int i = 0; i < objects.Count; i++)
            {
                for (int j = i + 1; j < objects.Count; j++)
                {
                    var obj1 = objects[i];
                    var obj2 = objects[j];

                    if (obj1.IsStatic && obj2.IsStatic) continue;

                    Vector collisionNormal = new Vector(0, 0);
                    double penetration = 0;
                    Vector contactPoint = new Vector(0, 0);

                    if (obj1.IntersectsWith(obj2, out collisionNormal, out penetration, out contactPoint))
                    {
                        // Make sure normal points from obj1 to obj2
                        Vector center1 = GetCenter(obj1);
                        Vector center2 = GetCenter(obj2);
                        Vector direction = center2 - center1;

                        if (Vector.Multiply(direction, collisionNormal) < 0)
                        {
                            collisionNormal = -collisionNormal;
                        }

                        ResolveCollisionWithSize(obj1, obj2, collisionNormal, penetration, contactPoint);
                        CollisionDetected?.Invoke(this, new CollisionEventArgs(obj1, obj2));
                    }
                }
            }
        }

        private void ResolveCollisionWithSize(PhysicsObject obj1, PhysicsObject obj2, Vector normal, double penetration, Vector contactPoint)
        {
            // Skip if both objects are static
            if (obj1.IsStatic && obj2.IsStatic) return;

            // Calculate size ratio
            double size1 = Math.Max(obj1.Width, obj1.Height);
            double size2 = Math.Max(obj2.Width, obj2.Height);
            double sizeRatio = size1 / size2;

            // Apply size-based separation
            if (!obj1.IsStatic && !obj2.IsStatic)
            {
                double totalSize = size1 + size2;
                double obj1Factor = size2 / totalSize;
                double obj2Factor = size1 / totalSize;

                obj1.Position -= normal * (penetration * obj1Factor);
                obj2.Position += normal * (penetration * obj2Factor);
            }
            else if (!obj1.IsStatic)
            {
                obj1.Position -= normal * penetration;
            }
            else if (!obj2.IsStatic)
            {
                obj2.Position += normal * penetration;
            }

            // Calculate relative velocity
            Vector r1 = contactPoint - GetCenter(obj1);
            Vector r2 = contactPoint - GetCenter(obj2);

            Vector vel1 = obj1.Velocity;
            Vector vel2 = obj2.Velocity;

            if (!obj1.IsStatic)
            {
                vel1 += new Vector(-obj1.AngularVelocity * r1.Y, obj1.AngularVelocity * r1.X);
            }
            if (!obj2.IsStatic)
            {
                vel2 += new Vector(-obj2.AngularVelocity * r2.Y, obj2.AngularVelocity * r2.X);
            }

            Vector relativeVelocity = vel2 - vel1;
            double velocityAlongNormal = Vector.Multiply(relativeVelocity, normal);

            if (velocityAlongNormal > 0) return;

            // Calculate restitution
            double e1 = obj1.Restitution * (1 - 0.1 * Math.Min(1, size1 / 100));
            double e2 = obj2.Restitution * (1 - 0.1 * Math.Min(1, size2 / 100));
            double e = Math.Min(e1, e2);

            // Calculate inverse mass
            double invMass1 = obj1.IsStatic ? 0 : 1 / obj1.Mass;
            double invMass2 = obj2.IsStatic ? 0 : 1 / obj2.Mass;

            // Calculate inverse inertia
            double invInertia1 = obj1.IsStatic ? 0 : 1 / obj1.MomentOfInertia;
            double invInertia2 = obj2.IsStatic ? 0 : 1 / obj2.MomentOfInertia;

            // Calculate angular effect
            double r1CrossN = r1.X * normal.Y - r1.Y * normal.X;
            double r2CrossN = r2.X * normal.Y - r2.Y * normal.X;

            double angularEffect1 = r1CrossN * r1CrossN * invInertia1;
            double angularEffect2 = r2CrossN * r2CrossN * invInertia2;

            // Calculate impulse
            double impulse = -(1 + e) * velocityAlongNormal;
            impulse /= (invMass1 + invMass2 + angularEffect1 + angularEffect2);

            // Size-based impulse scaling
            double sizeImpulseFactor = 1.0;
            if (!obj1.IsStatic && !obj2.IsStatic)
            {
                sizeImpulseFactor = Math.Min(2.0, Math.Max(0.5, sizeRatio));
            }
            impulse *= sizeImpulseFactor;

            // Apply linear impulse
            Vector impulseVector = impulse * normal;

            if (!obj1.IsStatic)
            {
                obj1.Velocity -= impulseVector * invMass1;
                double angularImpulse1 = r1CrossN * impulse * invInertia1 * (1 + 0.2 * (1 - sizeRatio));
                obj1.AngularVelocity -= angularImpulse1;
            }

            if (!obj2.IsStatic)
            {
                obj2.Velocity += impulseVector * invMass2;
                double angularImpulse2 = r2CrossN * impulse * invInertia2 * (1 + 0.2 * (sizeRatio));
                obj2.AngularVelocity += angularImpulse2;
            }

            // Add friction
            Vector tangent = new Vector(-normal.Y, normal.X);
            double velocityAlongTangent = Vector.Multiply(relativeVelocity, tangent);

            if (Math.Abs(velocityAlongTangent) > 0.01)
            {
                double frictionCoeff1 = obj1.Friction * (1 + 0.2 * Math.Min(1, size1 / 100));
                double frictionCoeff2 = obj2.Friction * (1 + 0.2 * Math.Min(1, size2 / 100));
                double frictionCoeff = (frictionCoeff1 + frictionCoeff2) / 2;

                double maxFriction = Math.Abs(impulse) * frictionCoeff;
                double frictionImpulse = -velocityAlongTangent;

                double r1CrossT = r1.X * tangent.Y - r1.Y * tangent.X;
                double r2CrossT = r2.X * tangent.Y - r2.Y * tangent.X;

                double frictionAngularEffect1 = r1CrossT * r1CrossT * invInertia1;
                double frictionAngularEffect2 = r2CrossT * r2CrossT * invInertia2;

                frictionImpulse /= (invMass1 + invMass2 + frictionAngularEffect1 + frictionAngularEffect2);
                frictionImpulse = Math.Max(-maxFriction, Math.Min(maxFriction, frictionImpulse));

                if (!obj1.IsStatic && !obj2.IsStatic)
                {
                    frictionImpulse *= Math.Min(1.5, Math.Max(0.5, sizeRatio));
                }

                Vector frictionVector = frictionImpulse * tangent;

                if (!obj1.IsStatic)
                {
                    obj1.Velocity -= frictionVector * invMass1;
                    double frictionAngularImpulse1 = r1CrossT * frictionImpulse * invInertia1;
                    obj1.AngularVelocity -= frictionAngularImpulse1 * (1 + 0.1 * (1 - sizeRatio));
                }

                if (!obj2.IsStatic)
                {
                    obj2.Velocity += frictionVector * invMass2;
                    double frictionAngularImpulse2 = r2CrossT * frictionImpulse * invInertia2;
                    obj2.AngularVelocity += frictionAngularImpulse2 * (1 + 0.1 * sizeRatio);
                }
            }
        }

        private Vector GetCenter(PhysicsObject obj)
        {
            Rect bounds = obj.GetBounds();
            return new Vector(bounds.X + bounds.Width / 2, bounds.Y + bounds.Height / 2);
        }

        private void CheckBoundaries()
        {
            foreach (var obj in objects)
            {
                Rect bounds = obj.GetBounds();
                Vector normal = new Vector(0, 0);
                double size = Math.Max(obj.Width, obj.Height);

                // Left boundary
                if (bounds.X < 0)
                {
                    obj.Position = new Vector(0, obj.Position.Y);
                    if (!obj.IsStatic)
                    {
                        double bounceFactor = Math.Max(0.5, 1 - size / 200);
                        obj.Velocity = new Vector(Math.Abs(obj.Velocity.X) * obj.Restitution * bounceFactor, obj.Velocity.Y);
                        normal = new Vector(1, 0);
                        obj.AngularVelocity *= 0.7;
                    }
                }

                // Right boundary
                if (bounds.X + bounds.Width > width)
                {
                    obj.Position = new Vector(width - bounds.Width, obj.Position.Y);
                    if (!obj.IsStatic)
                    {
                        double bounceFactor = Math.Max(0.5, 1 - size / 200);
                        obj.Velocity = new Vector(-Math.Abs(obj.Velocity.X) * obj.Restitution * bounceFactor, obj.Velocity.Y);
                        normal = new Vector(-1, 0);
                        obj.AngularVelocity *= 0.7;
                    }
                }

                // Top boundary
                if (bounds.Y < 0)
                {
                    obj.Position = new Vector(obj.Position.X, 0);
                    if (!obj.IsStatic)
                    {
                        double bounceFactor = Math.Max(0.5, 1 - size / 200);
                        obj.Velocity = new Vector(obj.Velocity.X, Math.Abs(obj.Velocity.Y) * obj.Restitution * bounceFactor);
                        normal = new Vector(0, 1);
                        obj.AngularVelocity *= 0.7;
                    }
                }

                // Bottom boundary - FIXED: Always bounce when hitting floor
                if (bounds.Y + bounds.Height > height)
                {
                    // Position correction
                    obj.Position = new Vector(obj.Position.X, height - bounds.Height);

                    if (!obj.IsStatic)
                    {
                        // ALWAYS bounce when hitting the floor with gravity enabled
                        // Only check for resting state if velocity is VERY low
                        bool isResting = Math.Abs(obj.Velocity.Y) < 5.0 && obj.Velocity.Y > 0;

                        if (isResting && Math.Abs(obj.Velocity.X) < 1.0)
                        {
                            // Object is resting - minimal bounce
                            obj.Velocity = new Vector(obj.Velocity.X * 0.95, 0);
                            obj.AngularVelocity *= 0.95;

                            // For objects with flat bottoms, ensure they're level
                            if (obj is Box)
                            {
                                // Boxes should sit flat - adjust rotation to nearest 90 degrees
                                double pi = Math.PI;
                                double targetRotation = Math.Round(obj.Rotation / (pi / 2)) * (pi / 2);
                                obj.Rotation = targetRotation;
                            }
                        }
                        else
                        {
                            // PROPER BOUNCE - reverse vertical velocity with restitution
                            double bounceFactor = obj.Restitution;

                            // Add a small boost to ensure objects leave the ground
                            if (obj.Velocity.Y > 0) // Moving down
                            {
                                // Slight horizontal damping
                                obj.Velocity = new Vector(obj.Velocity.X * 0.98, -Math.Abs(obj.Velocity.Y) * bounceFactor);

                                // Add spin based on horizontal velocity
                                double spinAmount = obj.Velocity.X * 0.05;
                                obj.AngularVelocity += spinAmount;

                                System.Diagnostics.Debug.WriteLine($"Bouncing! Y velocity: {obj.Velocity.Y:F2}");
                            }
                        }

                        normal = new Vector(0, -1);
                    }
                }
            }
        }
    }    

    public class CollisionEventArgs : EventArgs
    {
        public PhysicsObject Object1 { get; }
        public PhysicsObject Object2 { get; }
        public double SizeRatio { get; set; }

        public CollisionEventArgs(PhysicsObject obj1, PhysicsObject obj2)
        {
            Object1 = obj1;
            Object2 = obj2;
            SizeRatio = Math.Max(obj1.Width, obj1.Height) / Math.Max(obj2.Width, obj2.Height);
        }
    }
}
