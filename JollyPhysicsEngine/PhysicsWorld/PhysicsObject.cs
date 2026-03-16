using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;

namespace JollyPhysicsEngine.PhysicsWorld
{
    public abstract class PhysicsObject
    {
        public Shape Shape { get; set; }
        public Vector Position { get; set; }
        public Vector Velocity { get; set; }
        public double Mass { get; set; }
        public double Restitution { get; set; } = 0.7; // Bounciness
        public bool IsStatic { get; set; }

        // New properties for rotation
        public double Rotation { get; set; } = 0; // Current rotation in radians
        public double AngularVelocity { get; set; } = 0; // Rotational speed in radians per second
        public double MomentOfInertia { get; set; } // Rotational inertia
        public double Friction { get; set; } = 0.3; // Surface friction coefficient

        public abstract double Width { get; }
        public abstract double Height { get; }

        public virtual void UpdatePosition(double deltaTime)
        {
            if (IsStatic) return;

            Position += Velocity * deltaTime;

            // Update rotation based on angular velocity
            Rotation += AngularVelocity * deltaTime;

            // Keep rotation in range [-PI, PI] for stability
            Rotation = NormalizeAngle(Rotation);

            // Update shape position and rotation on canvas
            UpdateShapeTransform();
        }

        protected virtual void UpdateShapeTransform()
        {
            Canvas.SetLeft(Shape, Position.X);
            Canvas.SetTop(Shape, Position.Y);

            // Apply rotation transform
            Shape.RenderTransformOrigin = new Point(0.5, 0.5);
            Shape.RenderTransform = new RotateTransform(Rotation * 180 / Math.PI);
        }

        public virtual void ApplyForce(Vector force, double deltaTime)
        {
            if (IsStatic) 
                return;

            // F = ma -> a = F/m
            Vector acceleration = force / Mass;
            Velocity += acceleration * deltaTime;
        }

        // Apply force at a point (for torque calculation)
        public virtual void ApplyForceAtPoint(Vector force, Vector point, double deltaTime)
        {
            if (IsStatic) 
                return;

            // Linear acceleration
            Vector acceleration = force / Mass;
            Velocity += acceleration * deltaTime;

            // Calculate torque and angular acceleration
            Vector center = new Vector(Position.X + Width / 2, Position.Y + Height / 2);
            Vector r = point - center; // Lever arm

            double torque = r.X * force.Y - r.Y * force.X; // Cross product in 2D
            double angularAcceleration = torque / MomentOfInertia;
            AngularVelocity += angularAcceleration * deltaTime;

            // Apply friction/damping
            AngularVelocity *= (1 - Friction * deltaTime);
        }

        public virtual Rect GetBounds()
        {
            return new Rect(Position.X, Position.Y, Width, Height);
        }

        protected double NormalizeAngle(double angle)
        {
            while (angle > Math.PI) angle -= 2 * Math.PI;
            while (angle < -Math.PI) angle += 2 * Math.PI;
            return angle;
        }

        public virtual bool IntersectsWith(PhysicsObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            // Default AABB collision
            Rect bounds1 = GetBounds();
            Rect bounds2 = other.GetBounds();

            if (!bounds1.IntersectsWith(bounds2))
                return false;

            Vector center1 = new Vector(bounds1.X + bounds1.Width / 2, bounds1.Y + bounds1.Height / 2);
            Vector center2 = new Vector(bounds2.X + bounds2.Width / 2, bounds2.Y + bounds2.Height / 2);

            Vector diff = center2 - center1;

            double xOverlap = (bounds1.Width + bounds2.Width) / 2 - Math.Abs(diff.X);
            double yOverlap = (bounds1.Height + bounds2.Height) / 2 - Math.Abs(diff.Y);

            if (xOverlap > 0 && yOverlap > 0)
            {
                if (xOverlap < yOverlap)
                {
                    normal = new Vector(Math.Sign(diff.X), 0);
                    penetration = xOverlap;
                    contactPoint = new Vector(center1.X + normal.X * (bounds1.Width / 2), center1.Y);
                }
                else
                {
                    normal = new Vector(0, Math.Sign(diff.Y));
                    penetration = yOverlap;
                    contactPoint = new Vector(center1.X, center1.Y + normal.Y * (bounds1.Height / 2));
                }
                return true;
            }

            return false;
        }

        public virtual double GetSize()
        {
            return Math.Max(Width, Height);
        }

        public virtual double GetVolume()
        {
            return Width * Height;
        }
    }

    public class Ball : PhysicsObject
    {
        public double Radius { get; set; }

        public override double Width => Radius * 2;
        public override double Height => Radius * 2;

        public Ball(double radius, double x, double y)
        {
            Radius = radius;
            Position = new Vector(x, y);
            Mass = radius * radius * Math.PI; // Area as mass
            MomentOfInertia = Mass * radius * radius / 2; // Solid disk moment of inertia
            Velocity = new Vector(0, 0);

            // Create visual representation
            Shape = new Ellipse
            {
                Width = radius * 2,
                Height = radius * 2,
                Fill = new SolidColorBrush(Color.FromRgb((byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255))),
                Stroke = Brushes.Black,
                StrokeThickness = 1
            };

            // Add a line to visualize rotation
            AddRotationIndicator();
        }

        private void AddRotationIndicator()
        {
            var line = new Line
            {
                X1 = Radius,
                Y1 = Radius,
                X2 = Radius * 1.8,
                Y2 = Radius,
                Stroke = Brushes.Black,
                StrokeThickness = 2
            };

            // We'll need to handle this differently - for now, just note that we'd add rotation indicators
            // This would require a custom control or multiple shapes
        }

        protected override void UpdateShapeTransform()
        {
            base.UpdateShapeTransform();
        }

        public override bool IntersectsWith(PhysicsObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            if (other is Ball otherBall)
            {
                return IntersectsWithBall(otherBall, out normal, out penetration, out contactPoint);
            }
            else if (other is Box otherBox)
            {
                return IntersectsWithBox(otherBox, out normal, out penetration, out contactPoint);
            }
            else if (other is Triangle otherTriangle)
            {
                return IntersectsWithTriangle(otherTriangle, out normal, out penetration, out contactPoint);
            }

            return base.IntersectsWith(other, out normal, out penetration, out contactPoint);
        }

        private bool IntersectsWithBall(Ball other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            Vector center1 = new Vector(Position.X + Radius, Position.Y + Radius);
            Vector center2 = new Vector(other.Position.X + other.Radius, other.Position.Y + other.Radius);

            Vector diff = center2 - center1;
            double distance = diff.Length;
            double minDistance = Radius + other.Radius;

            if (distance < minDistance)
            {
                normal = distance > 0 ? diff / distance : new Vector(1, 0);
                penetration = minDistance - distance;

                // Contact point is on the line between centers
                contactPoint = center1 + normal * Radius;
                return true;
            }

            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);
            return false;
        }

        private bool IntersectsWithBox(Box box, out Vector normal, out double penetration, out Vector contactPoint)
        {
            Vector circleCenter = new Vector(Position.X + Radius, Position.Y + Radius);
            Rect boxBounds = box.GetBounds();

            // Find closest point on box to circle
            double closestX = Math.Max(boxBounds.X, Math.Min(circleCenter.X, boxBounds.X + boxBounds.Width));
            double closestY = Math.Max(boxBounds.Y, Math.Min(circleCenter.Y, boxBounds.Y + boxBounds.Height));

            Vector closestPoint = new Vector(closestX, closestY);
            Vector diff = circleCenter - closestPoint;
            double distance = diff.Length;

            if (distance < Radius)
            {
                if (distance > 0)
                {
                    normal = diff / distance;
                    penetration = Radius - distance;
                    contactPoint = closestPoint;
                }
                else
                {
                    // Circle center is inside box - find nearest edge
                    double left = Math.Abs(circleCenter.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - circleCenter.X);
                    double top = Math.Abs(circleCenter.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - circleCenter.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min == left)
                    {
                        normal = new Vector(-1, 0);
                        contactPoint = new Vector(boxBounds.X, circleCenter.Y);
                    }
                    else if (min == right)
                    {
                        normal = new Vector(1, 0);
                        contactPoint = new Vector(boxBounds.X + boxBounds.Width, circleCenter.Y);
                    }
                    else if (min == top)
                    {
                        normal = new Vector(0, -1);
                        contactPoint = new Vector(circleCenter.X, boxBounds.Y);
                    }
                    else
                    {
                        normal = new Vector(0, 1);
                        contactPoint = new Vector(circleCenter.X, boxBounds.Y + boxBounds.Height);
                    }

                    penetration = Radius;
                }
                return true;
            }

            contactPoint = new Vector(0, 0);
            penetration = 0;
            return false;
        }

        private bool IntersectsWithTriangle(Triangle triangle, out Vector normal, out double penetration, out Vector contactPoint)
        {
            Vector circleCenter = new Vector(Position.X + Radius, Position.Y + Radius);
            Point[] points = triangle.GetTransformedPoints();

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check collision with each edge
            for (int i = 0; i < 3; i++)
            {
                Point p1 = points[i];
                Point p2 = points[(i + 1) % 3];

                Vector edge = new Vector(p2.X - p1.X, p2.Y - p1.Y);
                Vector toCircle = new Vector(circleCenter.X - p1.X, circleCenter.Y - p1.Y);

                // Project circle center onto edge
                double t = Vector.Multiply(toCircle, edge) / edge.LengthSquared;
                t = Math.Max(0, Math.Min(1, t));

                Vector closestPoint = new Vector(p1.X + t * edge.X, p1.Y + t * edge.Y);
                Vector diff = circleCenter - closestPoint;
                double distance = diff.Length;

                if (distance < Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        bestNormal = distance > 0 ? diff / distance : GetEdgeNormal(edge);
                        bestContact = closestPoint;
                    }
                }
            }

            // Check collision with vertices
            for (int i = 0; i < 3; i++)
            {
                Vector toVertex = circleCenter - new Vector(points[i].X, points[i].Y);
                double distance = toVertex.Length;

                if (distance < Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        bestNormal = distance > 0 ? toVertex / distance : new Vector(1, 0);
                        bestContact = new Vector(points[i].X, points[i].Y);
                    }
                }
            }

            if (collision)
            {
                normal = bestNormal;
                penetration = Radius - minDistance;
                contactPoint = bestContact;
                return true;
            }

            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);
            return false;
        }

        private Vector GetEdgeNormal(Vector edge)
        {
            return new Vector(-edge.Y, edge.X) / edge.Length;
        }
    }

    public class Box : PhysicsObject
    {
        public double BoxWidth { get; set; }
        public double BoxHeight { get; set; }

        public override double Width => BoxWidth;
        public override double Height => BoxHeight;

        public Box(double width, double height, double x, double y)
        {
            BoxWidth = width;
            BoxHeight = height;
            Position = new Vector(x, y);
            Mass = width * height; // Area as mass
            MomentOfInertia = Mass * (width * width + height * height) / 12; // Rectangle moment of inertia
            Velocity = new Vector(0, 0);

            // Create visual representation
            Shape = new System.Windows.Shapes.Rectangle
            {
                Width = width,
                Height = height,
                Fill = new SolidColorBrush(Color.FromRgb((byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255))),
                Stroke = Brushes.Black,
                StrokeThickness = 1
            };
        }

        public override bool IntersectsWith(PhysicsObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            if (other is Ball otherBall)
            {
                bool result = otherBall.IntersectsWith(this, out normal, out penetration, out contactPoint);
                if (result) 
                    normal = -normal;
                return result;
            }
            else if (other is Triangle otherTriangle)
            {
                return IntersectsWithTriangle(otherTriangle, out normal, out penetration, out contactPoint);
            }
            else if (other is Box otherBox)
            {
                return IntersectsWithBox(otherBox, out normal, out penetration, out contactPoint);
            }

            return base.IntersectsWith(other, out normal, out penetration, out contactPoint);
        }

        private bool IntersectsWithBox(Box other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            Rect bounds1 = GetBounds();
            Rect bounds2 = other.GetBounds();

            Vector center1 = new Vector(bounds1.X + bounds1.Width / 2, bounds1.Y + bounds1.Height / 2);
            Vector center2 = new Vector(bounds2.X + bounds2.Width / 2, bounds2.Y + bounds2.Height / 2);

            Vector diff = center2 - center1;

            double xOverlap = (bounds1.Width + bounds2.Width) / 2 - Math.Abs(diff.X);
            double yOverlap = (bounds1.Height + bounds2.Height) / 2 - Math.Abs(diff.Y);

            if (xOverlap > 0 && yOverlap > 0)
            {
                if (xOverlap < yOverlap)
                {
                    normal = new Vector(Math.Sign(diff.X), 0);
                    penetration = xOverlap;
                    contactPoint = new Vector(center1.X + normal.X * (bounds1.Width / 2), center1.Y);
                }
                else
                {
                    normal = new Vector(0, Math.Sign(diff.Y));
                    penetration = yOverlap;
                    contactPoint = new Vector(center1.X, center1.Y + normal.Y * (bounds1.Height / 2));
                }
                return true;
            }

            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);
            return false;
        }

        private bool IntersectsWithTriangle(Triangle triangle, out Vector normal, out double penetration, out Vector contactPoint)
        {
            Rect boxBounds = GetBounds();
            Point[] triPoints = triangle.GetTransformedPoints();

            // Quick AABB test first
            Rect triBounds = triangle.GetBounds();
            if (!boxBounds.IntersectsWith(triBounds))
            {
                normal = new Vector(0, 0);
                penetration = 0;
                contactPoint = new Vector(0, 0);
                return false;
            }

            // Check triangle vertices against box
            foreach (Point p in triPoints)
            {
                if (boxBounds.Contains(p))
                {
                    // Find closest edge
                    double left = Math.Abs(p.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - p.X);
                    double top = Math.Abs(p.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - p.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min == left)
                    {
                        normal = new Vector(-1, 0);
                        contactPoint = new Vector(boxBounds.X, p.Y);
                    }
                    else if (min == right)
                    {
                        normal = new Vector(1, 0);
                        contactPoint = new Vector(boxBounds.X + boxBounds.Width, p.Y);
                    }
                    else if (min == top)
                    {
                        normal = new Vector(0, -1);
                        contactPoint = new Vector(p.X, boxBounds.Y);
                    }
                    else
                    {
                        normal = new Vector(0, 1);
                        contactPoint = new Vector(p.X, boxBounds.Y + boxBounds.Height);
                    }

                    penetration = min;
                    return true;
                }
            }

            // Check box corners against triangle
            Point[] boxCorners = new Point[]
            {
                new Point(boxBounds.X, boxBounds.Y),
                new Point(boxBounds.X + boxBounds.Width, boxBounds.Y),
                new Point(boxBounds.X, boxBounds.Y + boxBounds.Height),
                new Point(boxBounds.X + boxBounds.Width, boxBounds.Y + boxBounds.Height)
            };

            foreach (Point p in boxCorners)
            {
                if (IsPointInTriangle(p, triPoints))
                {
                    normal = new Vector(0, 1); // Default upward normal
                    penetration = 5;
                    contactPoint = new Vector(p.X, p.Y);
                    return true;
                }
            }

            // Check edge intersections
            for (int i = 0; i < 3; i++)
            {
                Point p1 = triPoints[i];
                Point p2 = triPoints[(i + 1) % 3];

                if (LineIntersectsRect(p1, p2, boxBounds, out Point intersection))
                {
                    normal = GetEdgeNormal(new Vector(p2.X - p1.X, p2.Y - p1.Y));
                    penetration = 5;
                    contactPoint = new Vector(intersection.X, intersection.Y); ;
                    return true;
                }
            }

            penetration = 0;
            return false;
        }

        private bool IsPointInTriangle(Point p, Point[] triangle)
        {
            double area = 0.5 * (-triangle[1].Y * triangle[2].X + triangle[0].Y * (-triangle[1].X + triangle[2].X) + triangle[0].X * (triangle[1].Y - triangle[2].Y) + triangle[1].X * triangle[2].Y);

            double sign = Math.Sign(area);
            double s = (triangle[0].Y * triangle[2].X - triangle[0].X * triangle[2].Y + (triangle[2].Y - triangle[0].Y) * p.X + (triangle[0].X - triangle[2].X) * p.Y) * sign;
            double t = (triangle[0].X * triangle[1].Y - triangle[0].Y * triangle[1].X + (triangle[0].Y - triangle[1].Y) * p.X + (triangle[1].X - triangle[0].X) * p.Y) * sign;

            return s >= 0 && t >= 0 && (s + t) <= 2 * area * sign;
        }

        private bool LineIntersectsRect(Point p1, Point p2, Rect rect, out Point intersection)
        {
            intersection = new Point(0, 0);
            double dx = p2.X - p1.X;
            double dy = p2.Y - p1.Y;

            double[] p = { -dx, dx, -dy, dy };
            double[] q = { p1.X - rect.X, rect.X + rect.Width - p1.X, p1.Y - rect.Y, rect.Y + rect.Height - p1.Y };

            double u1 = 0, u2 = 1;

            for (int i = 0; i < 4; i++)
            {
                if (p[i] == 0)
                {
                    if (q[i] < 0) return false;
                }
                else
                {
                    double t = q[i] / p[i];
                    if (p[i] < 0 && u1 < t) u1 = t;
                    if (p[i] > 0 && u2 > t) u2 = t;
                }
            }

            if (u1 <= u2)
            {
                intersection = new Point(p1.X + u1 * dx, p1.Y + u1 * dy);
                return true;
            }

            return false;
        }

        private Vector GetEdgeNormal(Vector edge)
        {
            return new Vector(-edge.Y, edge.X) / edge.Length;
        }
    }

    public class Triangle : PhysicsObject
    {
        public double BaseWidth { get; set; }
        public double TriangleHeight { get; set; }
        private PointCollection originalPoints;
        private PointCollection transformedPoints;

        public override double Width => BaseWidth;
        public override double Height => TriangleHeight;

        // Store the centroid for rotation
        private Vector Centroid
        {
            get
            {
                // Calculate centroid from transformed points
                Point[] points = GetTransformedPoints();
                if (points.Length == 3)
                {
                    return new Vector((points[0].X + points[1].X + points[2].X) / 3, (points[0].Y + points[1].Y + points[2].Y) / 3);
                }
                return new Vector(Position.X + BaseWidth / 2, Position.Y + Height * 2 / 3);
            }
        }

        public Triangle(double baseWidth, double height, double x, double y)
        {
            BaseWidth = baseWidth;
            TriangleHeight = height;
            Position = new Vector(x, y);
            Mass = (baseWidth * height) / 2; // Area as mass

            // Moment of inertia for triangle about its centroid
            MomentOfInertia = Mass * (baseWidth * baseWidth + height * height) / 18;

            Velocity = new Vector(0, 0);
            Restitution = 0.7;
            Friction = 0.3;

            // Define triangle points relative to top-left corner
            originalPoints = new PointCollection
            {
                new Point(0, height),           // Bottom-left
                new Point(baseWidth / 2, 0),     // Top-middle
                new Point(baseWidth, height)      // Bottom-right
            };

            transformedPoints = new PointCollection();
            UpdateTransformedPoints();

            // Create visual representation
            Shape = new Polygon
            {
                Points = transformedPoints,
                Fill = new SolidColorBrush(Color.FromRgb((byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255))), Stroke = Brushes.Black,
                StrokeThickness = 1,
                StrokeLineJoin = PenLineJoin.Round
            };
        }

        private void UpdateTransformedPoints()
        {
            transformedPoints.Clear();

            // Calculate rotation center (centroid of triangle in local coordinates)
            double centerX = BaseWidth / 2;
            double centerY = Height * 2 / 3; // Centroid of triangle is at average of vertices

            foreach (Point p in originalPoints)
            {
                // Translate point to origin (relative to centroid)
                double x = p.X - centerX;
                double y = p.Y - centerY;

                // Rotate
                double rotatedX = x * Math.Cos(Rotation) - y * Math.Sin(Rotation);
                double rotatedY = x * Math.Sin(Rotation) + y * Math.Cos(Rotation);

                // Translate back and add position offset
                transformedPoints.Add(new Point(Position.X + rotatedX + centerX, Position.Y + rotatedY + centerY));
            }
        }

        protected override void UpdateShapeTransform()
        {
            UpdateTransformedPoints();

            if (Shape is Polygon polygon)
            {
                polygon.Points = transformedPoints;
                // Don't use RenderTransform since we're manually updating points
                polygon.RenderTransform = null;
            }

            // Update canvas position (though points already include position)
            Canvas.SetLeft(Shape, 0);
            Canvas.SetTop(Shape, 0);
        }

        public Point[] GetTransformedPoints()
        {
            if (transformedPoints == null || transformedPoints.Count == 0)
            {
                UpdateTransformedPoints();
            }

            Point[] points = new Point[transformedPoints.Count];
            transformedPoints.CopyTo(points, 0);
            return points;
        }

        public override Rect GetBounds()
        {
            Point[] points = GetTransformedPoints();

            if (points.Length == 0)
                return new Rect(Position.X, Position.Y, Width, Height);

            double minX = double.MaxValue;
            double minY = double.MaxValue;
            double maxX = double.MinValue;
            double maxY = double.MinValue;

            foreach (Point p in points)
            {
                minX = Math.Min(minX, p.X);
                minY = Math.Min(minY, p.Y);
                maxX = Math.Max(maxX, p.X);
                maxY = Math.Max(maxY, p.Y);
            }

            // Add a small buffer to prevent sticking
            return new Rect(minX - 0.1, minY - 0.1, maxX - minX + 0.2, maxY - minY + 0.2);
        }

        public override bool IntersectsWith(PhysicsObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            // Quick bounds check first
            Rect myBounds = GetBounds();
            Rect otherBounds = other.GetBounds();

            if (!myBounds.IntersectsWith(otherBounds))
                return false;

            if (other is Ball otherBall)
            {
                return IntersectsWithBall(otherBall, out normal, out penetration, out contactPoint);
            }
            else if (other is Box otherBox)
            {
                return IntersectsWithBox(otherBox, out normal, out penetration, out contactPoint);
            }
            else if (other is Triangle otherTriangle)
            {
                return IntersectsWithTriangle(otherTriangle, out normal, out penetration, out contactPoint);
            }

            return false;
        }

        private bool IntersectsWithBall(Ball ball, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            Point[] triPoints = GetTransformedPoints();
            Vector ballCenter = new Vector(ball.Position.X + ball.Radius, ball.Position.Y + ball.Radius);

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check collision with each edge
            for (int i = 0; i < 3; i++)
            {
                Point p1 = triPoints[i];
                Point p2 = triPoints[(i + 1) % 3];

                Vector edge = new Vector(p2.X - p1.X, p2.Y - p1.Y);
                Vector toBall = new Vector(ballCenter.X - p1.X, ballCenter.Y - p1.Y);

                // Project ball center onto edge
                double edgeLengthSquared = edge.LengthSquared;
                if (edgeLengthSquared < 0.0001) continue;

                double t = Vector.Multiply(toBall, edge) / edgeLengthSquared;
                t = Math.Max(0, Math.Min(1, t));

                Vector closestPoint = new Vector(p1.X + t * edge.X, p1.Y + t * edge.Y);
                Vector diff = ballCenter - closestPoint;
                double distance = diff.Length;

                if (distance < ball.Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        if (distance > 0.0001)
                        {
                            bestNormal = diff / distance;
                        }
                        else
                        {
                            // Ball center is exactly on edge - use edge normal
                            bestNormal = GetEdgeNormal(edge);
                        }
                        bestContact = closestPoint;
                    }
                }
            }

            // Check collision with vertices
            for (int i = 0; i < 3; i++)
            {
                Vector toVertex = ballCenter - new Vector(triPoints[i].X, triPoints[i].Y);
                double distance = toVertex.Length;

                if (distance < ball.Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        if (distance > 0.0001)
                        {
                            bestNormal = toVertex / distance;
                        }
                        else
                        {
                            bestNormal = new Vector(1, 0);
                        }
                        bestContact = new Vector(triPoints[i].X, triPoints[i].Y);
                    }
                }
            }

            // Check if ball center is inside triangle
            if (!collision && IsPointInTriangle(ballCenter, triPoints))
            {
                // Find closest edge to push ball out
                for (int i = 0; i < 3; i++)
                {
                    Point p1 = triPoints[i];
                    Point p2 = triPoints[(i + 1) % 3];

                    Vector edge = new Vector(p2.X - p1.X, p2.Y - p1.Y);
                    Vector toBall = new Vector(ballCenter.X - p1.X, ballCenter.Y - p1.Y);

                    double t = Vector.Multiply(toBall, edge) / edge.LengthSquared;
                    t = Math.Max(0, Math.Min(1, t));

                    Vector closestPoint = new Vector(p1.X + t * edge.X, p1.Y + t * edge.Y);
                    Vector diff = ballCenter - closestPoint;
                    double distance = diff.Length;

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        bestNormal = GetEdgeNormal(edge);
                        bestContact = closestPoint;
                    }
                }

                collision = true;
            }

            if (collision)
            {
                // Ensure normal points from triangle to ball
                Vector centerToBall = ballCenter - Centroid;
                if (Vector.Multiply(centerToBall, bestNormal) < 0)
                {
                    bestNormal = -bestNormal;
                }

                normal = bestNormal;
                penetration = ball.Radius - minDistance;
                contactPoint = bestContact;
                return true;
            }

            return false;
        }

        private bool IntersectsWithBox(Box box, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            Point[] triPoints = GetTransformedPoints();
            Rect boxBounds = box.GetBounds();

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check triangle vertices against box
            for (int i = 0; i < 3; i++)
            {
                Point p = triPoints[i];

                if (p.X >= boxBounds.X && p.X <= boxBounds.X + boxBounds.Width && p.Y >= boxBounds.Y && p.Y <= boxBounds.Y + boxBounds.Height)
                {
                    // Point is inside box - find closest edge
                    double left = Math.Abs(p.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - p.X);
                    double top = Math.Abs(p.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - p.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min < minDistance)
                    {
                        minDistance = min;

                        if (min == left)
                        {
                            bestNormal = new Vector(-1, 0);
                            bestContact = new Vector(boxBounds.X, p.Y);
                        }
                        else if (min == right)
                        {
                            bestNormal = new Vector(1, 0);
                            bestContact = new Vector(boxBounds.X + boxBounds.Width, p.Y);
                        }
                        else if (min == top)
                        {
                            bestNormal = new Vector(0, -1);
                            bestContact = new Vector(p.X, boxBounds.Y);
                        }
                        else
                        {
                            bestNormal = new Vector(0, 1);
                            bestContact = new Vector(p.X, boxBounds.Y + boxBounds.Height);
                        }

                        collision = true;
                    }
                }
            }

            // Check box corners against triangle
            Point[] boxCorners = new Point[]
            {
            new Point(boxBounds.X, boxBounds.Y),
            new Point(boxBounds.X + boxBounds.Width, boxBounds.Y),
            new Point(boxBounds.X, boxBounds.Y + boxBounds.Height),
            new Point(boxBounds.X + boxBounds.Width, boxBounds.Y + boxBounds.Height)
            };

            foreach (Point corner in boxCorners)
            {
                if (IsPointInTriangle(new Vector(corner.X, corner.Y), triPoints))
                {
                    // Find closest triangle edge
                    for (int i = 0; i < 3; i++)
                    {
                        Point p1 = triPoints[i];
                        Point p2 = triPoints[(i + 1) % 3];

                        Vector edge = new Vector(p2.X - p1.X, p2.Y - p1.Y);
                        Vector toCorner = new Vector(corner.X - p1.X, corner.Y - p1.Y);

                        double t = Vector.Multiply(toCorner, edge) / edge.LengthSquared;
                        t = Math.Max(0, Math.Min(1, t));

                        Vector closestPoint = new Vector(p1.X + t * edge.X, p1.Y + t * edge.Y);
                        Vector diff = new Vector(corner.X - closestPoint.X, corner.Y - closestPoint.Y);
                        double distance = diff.Length;

                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            bestNormal = GetEdgeNormal(edge);
                            bestContact = closestPoint;
                            collision = true;
                        }
                    }
                }
            }

            // Check edge intersections
            for (int i = 0; i < 3; i++)
            {
                Point p1 = triPoints[i];
                Point p2 = triPoints[(i + 1) % 3];

                if (LineIntersectsRect(p1, p2, boxBounds, out Point intersection))
                {
                    // Find which box edge was hit
                    double left = Math.Abs(intersection.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - intersection.X);
                    double top = Math.Abs(intersection.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - intersection.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min < minDistance)
                    {
                        minDistance = min;

                        if (min == left) bestNormal = new Vector(-1, 0);
                        else if (min == right) bestNormal = new Vector(1, 0);
                        else if (min == top) bestNormal = new Vector(0, -1);
                        else bestNormal = new Vector(0, 1);

                        bestContact = new Vector(intersection.X, intersection.Y);
                        collision = true;
                    }
                }
            }

            if (collision)
            {
                // Ensure normal points from triangle to box
                Vector boxCenter = new Vector(
                    boxBounds.X + boxBounds.Width / 2,
                    boxBounds.Y + boxBounds.Height / 2
                );
                Vector centerToBox = boxCenter - Centroid;

                if (Vector.Multiply(centerToBox, bestNormal) < 0)
                {
                    bestNormal = -bestNormal;
                }

                normal = bestNormal;
                penetration = minDistance;
                contactPoint = bestContact;
                return true;
            }

            return false;
        }

        private bool IntersectsWithTriangle(Triangle other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            // For triangle-triangle, use Separating Axis Theorem (SAT)
            // For now, use a simplified AABB approach
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            Point[] myPoints = GetTransformedPoints();
            Point[] otherPoints = other.GetTransformedPoints();

            // Check if any point of other triangle is inside this triangle
            foreach (Point p in otherPoints)
            {
                if (IsPointInTriangle(new Vector(p.X, p.Y), myPoints))
                {
                    // Find closest edge
                    for (int i = 0; i < 3; i++)
                    {
                        Point p1 = myPoints[i];
                        Point p2 = myPoints[(i + 1) % 3];

                        Vector edge = new Vector(p2.X - p1.X, p2.Y - p1.Y);
                        Vector toPoint = new Vector(p.X - p1.X, p.Y - p1.Y);

                        double t = Vector.Multiply(toPoint, edge) / edge.LengthSquared;
                        t = Math.Max(0, Math.Min(1, t));

                        Vector closestPoint = new Vector(p1.X + t * edge.X, p1.Y + t * edge.Y);
                        Vector diff = new Vector(p.X - closestPoint.X, p.Y - closestPoint.Y);
                        double distance = diff.Length;

                        if (distance < penetration || penetration == 0)
                        {
                            penetration = distance;
                            normal = GetEdgeNormal(edge);
                            contactPoint = closestPoint;
                        }
                    }

                    if (penetration > 0)
                    {
                        return true;
                    }
                }
            }

            // Check edge intersections
            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    Point p1 = myPoints[i];
                    Point p2 = myPoints[(i + 1) % 3];
                    Point p3 = otherPoints[j];
                    Point p4 = otherPoints[(j + 1) % 3];

                    if (LineIntersection(p1, p2, p3, p4, out Point intersection))
                    {
                        penetration = 1; // Small default penetration
                        normal = GetEdgeNormal(new Vector(p2.X - p1.X, p2.Y - p1.Y));
                        contactPoint = new Vector(intersection.X, intersection.Y);
                        return true;
                    }
                }
            }

            return false;
        }

        private Vector GetEdgeNormal(Vector edge)
        {
            // Return normalized perpendicular vector pointing outward
            Vector normal = new Vector(-edge.Y, edge.X);
            double length = normal.Length;
            if (length > 0.0001)
            {
                normal /= length;
            }
            return normal;
        }

        private bool IsPointInTriangle(Vector p, Point[] triangle)
        {
            if (triangle.Length < 3) 
                return false;

            // Barycentric coordinate method
            Vector v0 = new Vector(triangle[2].X - triangle[0].X, triangle[2].Y - triangle[0].Y);
            Vector v1 = new Vector(triangle[1].X - triangle[0].X, triangle[1].Y - triangle[0].Y);
            Vector v2 = new Vector(p.X - triangle[0].X, p.Y - triangle[0].Y);

            double dot00 = Vector.Multiply(v0, v0);
            double dot01 = Vector.Multiply(v0, v1);
            double dot02 = Vector.Multiply(v0, v2);
            double dot11 = Vector.Multiply(v1, v1);
            double dot12 = Vector.Multiply(v1, v2);

            double invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
            double u = (dot11 * dot02 - dot01 * dot12) * invDenom;
            double v = (dot00 * dot12 - dot01 * dot02) * invDenom;

            // Check if point is inside triangle (with small epsilon for floating point)
            double epsilon = 0.0001;
            return (u >= -epsilon) && (v >= -epsilon) && (u + v <= 1 + epsilon);
        }

        private bool LineIntersectsRect(Point p1, Point p2, Rect rect, out Point intersection)
        {
            intersection = new Point(0, 0);

            // Check intersection with each edge of the rectangle
            Point[] rectPoints = new Point[]
            {
                new Point(rect.X, rect.Y),
                new Point(rect.X + rect.Width, rect.Y),
                new Point(rect.X + rect.Width, rect.Y + rect.Height),
                new Point(rect.X, rect.Y + rect.Height)
            };

            for (int i = 0; i < 4; i++)
            {
                Point r1 = rectPoints[i];
                Point r2 = rectPoints[(i + 1) % 4];

                if (LineIntersection(p1, p2, r1, r2, out intersection))
                {
                    return true;
                }
            }

            return false;
        }

        private bool LineIntersection(Point p1, Point p2, Point p3, Point p4, out Point intersection)
        {
            intersection = new Point(0, 0);

            double denominator = (p4.Y - p3.Y) * (p2.X - p1.X) - (p4.X - p3.X) * (p2.Y - p1.Y);

            if (Math.Abs(denominator) < 0.0001)
                return false;

            double ua = ((p4.X - p3.X) * (p1.Y - p3.Y) - (p4.Y - p3.Y) * (p1.X - p3.X)) / denominator;
            double ub = ((p2.X - p1.X) * (p1.Y - p3.Y) - (p2.Y - p1.Y) * (p1.X - p3.X)) / denominator;

            if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
            {
                intersection = new Point(p1.X + ua * (p2.X - p1.X), p1.Y + ua * (p2.Y - p1.Y));
                return true;
            }

            return false;
        }
    }

    public class PolygonObject : PhysicsObject
    {
        public int NumberOfSides { get; set; }
        public double Radius { get; set; }
        private PointCollection originalPoints;
        private PointCollection transformedPoints;

        public override double Width => Radius * 2;
        public override double Height => Radius * 2;

        // Store vertices for collision detection
        private List<Vector> vertices;
        private List<Vector> normals;

        public PolygonObject(int sides, double radius, double x, double y)
        {
            if (sides < 3) 
                sides = 3; // Minimum triangle

            if (sides > 20) 
                sides = 20; // Maximum to prevent performance issues

            NumberOfSides = sides;
            Radius = radius;
            Position = new Vector(x, y);

            // Calculate area for mass (approximate as regular polygon)
            double area = (sides * radius * radius * Math.Sin(2 * Math.PI / sides)) / 2;
            Mass = area;

            // Moment of inertia for regular polygon
            // For a regular polygon, I ≈ (1/6) * Mass * (R^2 + r^2) where r is apothem
            double apothem = radius * Math.Cos(Math.PI / sides);
            MomentOfInertia = Mass * (radius * radius + apothem * apothem) / 6;

            Velocity = new Vector(0, 0);
            Restitution = 0.7;
            Friction = 0.3;

            // Generate polygon points
            GeneratePolygonPoints();

            // Calculate vertices and normals for collision detection
            CalculateVerticesAndNormals();

            // Create visual representation
            Shape = new Polygon
            {
                Points = transformedPoints,
                Fill = new SolidColorBrush(Color.FromRgb((byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255), (byte)new Random().Next(100, 255))),
                Stroke = Brushes.Black,
                StrokeThickness = 1,
                StrokeLineJoin = PenLineJoin.Round
            };
        }

        private void GeneratePolygonPoints()
        {
            originalPoints = new PointCollection();

            double angleStep = 2 * Math.PI / NumberOfSides;

            // Start at top vertex (rotated -90 degrees to make first vertex at top)
            double startAngle = -Math.PI / 2;

            for (int i = 0; i < NumberOfSides; i++)
            {
                double angle = startAngle + i * angleStep;
                double x = Radius + Radius * Math.Cos(angle);
                double y = Radius + Radius * Math.Sin(angle);
                originalPoints.Add(new Point(x, y));
            }

            transformedPoints = new PointCollection();
            UpdateTransformedPoints();
        }

        private void CalculateVerticesAndNormals()
        {
            vertices = new List<Vector>();
            normals = new List<Vector>();

            for (int i = 0; i < NumberOfSides; i++)
            {
                // Calculate vertex in local coordinates (centered at origin)
                double angle = (2 * Math.PI * i / NumberOfSides) - Math.PI / 2;
                Vector vertex = new Vector(Radius * Math.Cos(angle), Radius * Math.Sin(angle));
                vertices.Add(vertex);
            }

            // Calculate edge normals
            for (int i = 0; i < NumberOfSides; i++)
            {
                Vector v1 = vertices[i];
                Vector v2 = vertices[(i + 1) % NumberOfSides];
                Vector edge = v2 - v1;

                // Perpendicular vector (pointing outward)
                Vector normal = new Vector(edge.Y, -edge.X);
                normal.Normalize();
                normals.Add(normal);
            }
        }

        private void UpdateTransformedPoints()
        {
            transformedPoints.Clear();

            foreach (Point p in originalPoints)
            {
                // Translate point to center
                double x = p.X - Radius;
                double y = p.Y - Radius;

                // Rotate
                double rotatedX = x * Math.Cos(Rotation) - y * Math.Sin(Rotation);
                double rotatedY = x * Math.Sin(Rotation) + y * Math.Cos(Rotation);

                // Translate back and add position
                transformedPoints.Add(new Point(Position.X + rotatedX + Radius, Position.Y + rotatedY + Radius));
            }
        }

        protected override void UpdateShapeTransform()
        {
            UpdateTransformedPoints();

            if (Shape is Polygon polygon)
            {
                polygon.Points = transformedPoints;
                polygon.RenderTransform = null;
            }

            Canvas.SetLeft(Shape, 0);
            Canvas.SetTop(Shape, 0);
        }

        public Point[] GetTransformedPoints()
        {
            if (transformedPoints == null || transformedPoints.Count == 0)
            {
                UpdateTransformedPoints();
            }

            Point[] points = new Point[transformedPoints.Count];
            transformedPoints.CopyTo(points, 0);
            return points;
        }

        public List<Vector> GetTransformedVertices()
        {
            List<Vector> transformedVertices = new List<Vector>();
            Point[] points = GetTransformedPoints();

            foreach (Point p in points)
            {
                transformedVertices.Add(new Vector(p.X, p.Y));
            }

            return transformedVertices;
        }

        public override Rect GetBounds()
        {
            Point[] points = GetTransformedPoints();

            if (points.Length == 0)
                return new Rect(Position.X, Position.Y, Width, Height);

            double minX = double.MaxValue;
            double minY = double.MaxValue;
            double maxX = double.MinValue;
            double maxY = double.MinValue;

            foreach (Point p in points)
            {
                minX = Math.Min(minX, p.X);
                minY = Math.Min(minY, p.Y);
                maxX = Math.Max(maxX, p.X);
                maxY = Math.Max(maxY, p.Y);
            }

            return new Rect(minX - 0.1, minY - 0.1, maxX - minX + 0.2, maxY - minY + 0.2);
        }

        public override bool IntersectsWith(PhysicsObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            // Quick bounds check first
            Rect myBounds = GetBounds();
            Rect otherBounds = other.GetBounds();

            if (!myBounds.IntersectsWith(otherBounds))
                return false;

            if (other is Ball otherBall)
            {
                return IntersectsWithBall(otherBall, out normal, out penetration, out contactPoint);
            }
            else if (other is Box otherBox)
            {
                return IntersectsWithBox(otherBox, out normal, out penetration, out contactPoint);
            }
            else if (other is Triangle otherTriangle)
            {
                return IntersectsWithTriangle(otherTriangle, out normal, out penetration, out contactPoint);
            }
            else if (other is PolygonObject otherPolygon)
            {
                return IntersectsWithPolygon(otherPolygon, out normal, out penetration, out contactPoint);
            }

            return false;
        }

        private bool IntersectsWithBall(Ball ball, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            List<Vector> polyPoints = GetTransformedVertices();
            Vector ballCenter = new Vector(ball.Position.X + ball.Radius, ball.Position.Y + ball.Radius);

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check collision with each edge
            for (int i = 0; i < polyPoints.Count; i++)
            {
                Vector p1 = polyPoints[i];
                Vector p2 = polyPoints[(i + 1) % polyPoints.Count];

                Vector edge = p2 - p1;
                Vector toBall = ballCenter - p1;

                // Project ball center onto edge
                double edgeLengthSquared = edge.LengthSquared;
                if (edgeLengthSquared < 0.0001) continue;

                double t = Vector.Multiply(toBall, edge) / edgeLengthSquared;
                t = Math.Max(0, Math.Min(1, t));

                Vector closestPoint = p1 + t * edge;
                Vector diff = ballCenter - closestPoint;
                double distance = diff.Length;

                if (distance < ball.Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        if (distance > 0.0001)
                        {
                            bestNormal = diff / distance;
                        }
                        else
                        {
                            // Ball center is exactly on edge - use edge normal
                            bestNormal = GetEdgeNormal(edge);
                        }
                        bestContact = closestPoint;
                    }
                }
            }

            // Check collision with vertices
            for (int i = 0; i < polyPoints.Count; i++)
            {
                Vector toVertex = ballCenter - polyPoints[i];
                double distance = toVertex.Length;

                if (distance < ball.Radius)
                {
                    collision = true;
                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        if (distance > 0.0001)
                        {
                            bestNormal = toVertex / distance;
                        }
                        else
                        {
                            bestNormal = new Vector(1, 0);
                        }
                        bestContact = polyPoints[i];
                    }
                }
            }

            // Check if ball center is inside polygon
            if (!collision && IsPointInPolygon(ballCenter, polyPoints))
            {
                // Find closest edge to push ball out
                for (int i = 0; i < polyPoints.Count; i++)
                {
                    Vector p1 = polyPoints[i];
                    Vector p2 = polyPoints[(i + 1) % polyPoints.Count];

                    Vector edge = p2 - p1;
                    Vector toBall = ballCenter - p1;

                    double t = Vector.Multiply(toBall, edge) / edge.LengthSquared;
                    t = Math.Max(0, Math.Min(1, t));

                    Vector closestPoint = p1 + t * edge;
                    Vector diff = ballCenter - closestPoint;
                    double distance = diff.Length;

                    if (distance < minDistance)
                    {
                        minDistance = distance;
                        bestNormal = GetEdgeNormal(edge);
                        bestContact = closestPoint;
                    }
                }

                collision = true;
            }

            if (collision)
            {
                // Ensure normal points from polygon to ball
                Vector polyCenter = new Vector(Position.X + Radius, Position.Y + Radius);
                Vector centerToBall = ballCenter - polyCenter;
                if (Vector.Multiply(centerToBall, bestNormal) < 0)
                {
                    bestNormal = -bestNormal;
                }

                normal = bestNormal;
                penetration = ball.Radius - minDistance;
                contactPoint = bestContact;
                return true;
            }

            return false;
        }

        private bool IntersectsWithBox(Box box, out Vector normal, out double penetration, out Vector contactPoint)
        {
            // For box collisions, we can use the polygon-point collision
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            List<Vector> polyPoints = GetTransformedVertices();
            Rect boxBounds = box.GetBounds();

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check polygon vertices against box
            foreach (Vector p in polyPoints)
            {
                if (p.X >= boxBounds.X && p.X <= boxBounds.X + boxBounds.Width && p.Y >= boxBounds.Y && p.Y <= boxBounds.Y + boxBounds.Height)
                {
                    // Point is inside box - find closest edge
                    double left = Math.Abs(p.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - p.X);
                    double top = Math.Abs(p.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - p.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min < minDistance)
                    {
                        minDistance = min;

                        if (min == left)
                        {
                            bestNormal = new Vector(-1, 0);
                            bestContact = new Vector(boxBounds.X, p.Y);
                        }
                        else if (min == right)
                        {
                            bestNormal = new Vector(1, 0);
                            bestContact = new Vector(boxBounds.X + boxBounds.Width, p.Y);
                        }
                        else if (min == top)
                        {
                            bestNormal = new Vector(0, -1);
                            bestContact = new Vector(p.X, boxBounds.Y);
                        }
                        else
                        {
                            bestNormal = new Vector(0, 1);
                            bestContact = new Vector(p.X, boxBounds.Y + boxBounds.Height);
                        }

                        collision = true;
                    }
                }
            }

            // Check box corners against polygon
            Point[] boxCorners = new Point[]
            {
                new Point(boxBounds.X, boxBounds.Y),
                new Point(boxBounds.X + boxBounds.Width, boxBounds.Y),
                new Point(boxBounds.X, boxBounds.Y + boxBounds.Height),
                new Point(boxBounds.X + boxBounds.Width, boxBounds.Y + boxBounds.Height)
            };

            foreach (Point corner in boxCorners)
            {
                if (IsPointInPolygon(new Vector(corner.X, corner.Y), polyPoints))
                {
                    // Find closest polygon edge
                    for (int i = 0; i < polyPoints.Count; i++)
                    {
                        Vector p1 = polyPoints[i];
                        Vector p2 = polyPoints[(i + 1) % polyPoints.Count];

                        Vector edge = p2 - p1;
                        Vector toCorner = new Vector(corner.X - p1.X, corner.Y - p1.Y);

                        double t = Vector.Multiply(toCorner, edge) / edge.LengthSquared;
                        t = Math.Max(0, Math.Min(1, t));

                        Vector closestPoint = p1 + t * edge;
                        Vector diff = new Vector(corner.X - closestPoint.X, corner.Y - closestPoint.Y);
                        double distance = diff.Length;

                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            bestNormal = GetEdgeNormal(edge);
                            bestContact = closestPoint;
                            collision = true;
                        }
                    }
                }
            }

            // Check edge intersections
            for (int i = 0; i < polyPoints.Count; i++)
            {
                Vector p1 = polyPoints[i];
                Vector p2 = polyPoints[(i + 1) % polyPoints.Count];

                if (LineIntersectsRect(new Point(p1.X, p1.Y), new Point(p2.X, p2.Y), boxBounds, out Point intersection))
                {
                    // Find which box edge was hit
                    double left = Math.Abs(intersection.X - boxBounds.X);
                    double right = Math.Abs(boxBounds.X + boxBounds.Width - intersection.X);
                    double top = Math.Abs(intersection.Y - boxBounds.Y);
                    double bottom = Math.Abs(boxBounds.Y + boxBounds.Height - intersection.Y);

                    double min = Math.Min(Math.Min(left, right), Math.Min(top, bottom));

                    if (min < minDistance)
                    {
                        minDistance = min;

                        if (min == left) bestNormal = new Vector(-1, 0);
                        else if (min == right) bestNormal = new Vector(1, 0);
                        else if (min == top) bestNormal = new Vector(0, -1);
                        else bestNormal = new Vector(0, 1);

                        bestContact = new Vector(intersection.X, intersection.Y);
                        collision = true;
                    }
                }
            }

            if (collision)
            {
                normal = bestNormal;
                penetration = minDistance;
                contactPoint = bestContact;
                return true;
            }

            return false;
        }

        private bool IntersectsWithTriangle(Triangle triangle, out Vector normal, out double penetration, out Vector contactPoint)
        {
            // Delegate to polygon-triangle collision
            List<Vector> polyPoints = GetTransformedVertices();
            Point[] triPoints = triangle.GetTransformedPoints();
            List<Vector> triVectors = new List<Vector>();

            foreach (Point p in triPoints)
            {
                triVectors.Add(new Vector(p.X, p.Y));
            }

            return IntersectsWithPolygon(triVectors, out normal, out penetration, out contactPoint);
        }

        private bool IntersectsWithPolygon(PolygonObject other, out Vector normal, out double penetration, out Vector contactPoint)
        {
            List<Vector> myPoints = GetTransformedVertices();
            List<Vector> otherPoints = other.GetTransformedVertices();

            return IntersectsWithPolygon(otherPoints, out normal, out penetration, out contactPoint);
        }

        private bool IntersectsWithPolygon(List<Vector> otherPoints, out Vector normal, out double penetration, out Vector contactPoint)
        {
            normal = new Vector(0, 0);
            penetration = 0;
            contactPoint = new Vector(0, 0);

            List<Vector> myPoints = GetTransformedVertices();

            double minDistance = double.MaxValue;
            Vector bestNormal = new Vector(0, 0);
            Vector bestContact = new Vector(0, 0);
            bool collision = false;

            // Check if any point of other polygon is inside this polygon
            foreach (Vector p in otherPoints)
            {
                if (IsPointInPolygon(p, myPoints))
                {
                    // Find closest edge
                    for (int i = 0; i < myPoints.Count; i++)
                    {
                        Vector p1 = myPoints[i];
                        Vector p2 = myPoints[(i + 1) % myPoints.Count];

                        Vector edge = p2 - p1;
                        Vector toPoint = p - p1;

                        double t = Vector.Multiply(toPoint, edge) / edge.LengthSquared;
                        t = Math.Max(0, Math.Min(1, t));

                        Vector closestPoint = p1 + t * edge;
                        Vector diff = p - closestPoint;
                        double distance = diff.Length;

                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            bestNormal = GetEdgeNormal(edge);
                            bestContact = closestPoint;
                            collision = true;
                        }
                    }
                }
            }

            // Check edge intersections
            for (int i = 0; i < myPoints.Count; i++)
            {
                for (int j = 0; j < otherPoints.Count; j++)
                {
                    Vector p1 = myPoints[i];
                    Vector p2 = myPoints[(i + 1) % myPoints.Count];
                    Vector p3 = otherPoints[j];
                    Vector p4 = otherPoints[(j + 1) % otherPoints.Count];

                    if (LineIntersection(new Point(p1.X, p1.Y), new Point(p2.X, p2.Y), new Point(p3.X, p3.Y), new Point(p4.X, p4.Y), out Point intersection))
                    {
                        double distance = 1; // Small default
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            bestNormal = GetEdgeNormal(p2 - p1);
                            bestContact = new Vector(intersection.X, intersection.Y);
                            collision = true;
                        }
                    }
                }
            }

            if (collision)
            {
                normal = bestNormal;
                penetration = minDistance;
                contactPoint = bestContact;
                return true;
            }

            return false;
        }

        private Vector GetEdgeNormal(Vector edge)
        {
            Vector normal = new Vector(-edge.Y, edge.X);
            double length = normal.Length;
            if (length > 0.0001)
            {
                normal /= length;
            }
            return normal;
        }

        private bool IsPointInPolygon(Vector point, List<Vector> polygon)
        {
            // Ray casting algorithm
            bool inside = false;
            int n = polygon.Count;

            for (int i = 0, j = n - 1; i < n; j = i++)
            {
                if (((polygon[i].Y > point.Y) != (polygon[j].Y > point.Y)) && (point.X < (polygon[j].X - polygon[i].X) * (point.Y - polygon[i].Y) / (polygon[j].Y - polygon[i].Y) + polygon[i].X))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        private bool LineIntersectsRect(Point p1, Point p2, Rect rect, out Point intersection)
        {
            intersection = new Point(0, 0);

            Point[] rectPoints = new Point[]
            {
            new Point(rect.X, rect.Y),
            new Point(rect.X + rect.Width, rect.Y),
            new Point(rect.X + rect.Width, rect.Y + rect.Height),
            new Point(rect.X, rect.Y + rect.Height)
            };

            for (int i = 0; i < 4; i++)
            {
                Point r1 = rectPoints[i];
                Point r2 = rectPoints[(i + 1) % 4];

                if (LineIntersection(p1, p2, r1, r2, out intersection))
                {
                    return true;
                }
            }

            return false;
        }

        private bool LineIntersection(Point p1, Point p2, Point p3, Point p4, out Point intersection)
        {
            intersection = new Point(0, 0);

            double denominator = (p4.Y - p3.Y) * (p2.X - p1.X) - (p4.X - p3.X) * (p2.Y - p1.Y);

            if (Math.Abs(denominator) < 0.0001)
                return false;

            double ua = ((p4.X - p3.X) * (p1.Y - p3.Y) - (p4.Y - p3.Y) * (p1.X - p3.X)) / denominator;
            double ub = ((p2.X - p1.X) * (p1.Y - p3.Y) - (p2.Y - p1.Y) * (p1.X - p3.X)) / denominator;

            if (ua >= 0 && ua <= 1 && ub >= 0 && ub <= 1)
            {
                intersection = new Point(p1.X + ua * (p2.X - p1.X), p1.Y + ua * (p2.Y - p1.Y));
                return true;
            }

            return false;
        }
    }
}
