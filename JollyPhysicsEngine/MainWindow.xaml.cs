using JollyPhysicsEngine.Physics;
using JollyPhysicsEngine.PhysicsWorld;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;

namespace JollyPhysicsEngine
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private class FlashState
        {
            public Brush OriginalBrush { get; set; }
            public Brush OriginalStroke { get; set; }
            public double OriginalThickness { get; set; }
            public int PendingFlashes { get; set; } = 1;
            public DateTime LastFlashTime { get; set; }
        }

        private Dictionary<PhysicsObject, FlashState> flashingObjects = new Dictionary<PhysicsObject, FlashState>();

        private PhysicsEngine physicsEngine;
        private DispatcherTimer timer;
        private DateTime lastUpdate;
        private bool isPaused = false;

        // For dragging objects
        private PhysicsObject draggedObject = null;
        private Vector dragOffset;
        private Vector lastMousePosition;
        private bool isDragging = false;
        Random random = new Random();

        public MainWindow()
        {
            InitializeComponent();

            // Initialize physics engine
            physicsEngine = new PhysicsEngine(physicsCanvas.ActualWidth, physicsCanvas.ActualHeight);
            physicsEngine.CollisionDetected += PhysicsEngine_CollisionDetected;

            // Subscribe to CompositionTarget.Rendering for smooth animation
            CompositionTarget.Rendering += CompositionTarget_Rendering;

            lastUpdate = DateTime.Now;
        }

        private void MainWindow_StateChanged(object? sender, EventArgs e)
        {
            if (this.WindowState == WindowState.Maximized)
            {
                RestoreButton.Visibility = Visibility.Visible;
                MaximizeButton.Visibility = Visibility.Collapsed;
            }
            else
            {
                MaximizeButton.Visibility = Visibility.Visible;
                RestoreButton.Visibility = Visibility.Collapsed;
            }
        }

        private void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // Add some initial objects
            AddBall();
            AddBox();
            AddTriangle();
            AddBall();
            AddBox();
            AddTriangle();
            AddBall();
            AddBox();
            AddTriangle();
        }

        private void CommandBinding_CanExecute(object sender, CanExecuteRoutedEventArgs e)
        {
            e.CanExecute = true;
        }

        private void CommandBinding_Executed_Minimize(object sender, ExecutedRoutedEventArgs e)
        {
            SystemCommands.MinimizeWindow(this);
        }

        private void CommandBinding_Executed_Maximize(object sender, ExecutedRoutedEventArgs e)
        {
            SystemCommands.MaximizeWindow(this);
        }

        private void CommandBinding_Executed_Restore(object sender, ExecutedRoutedEventArgs e)
        {
            SystemCommands.RestoreWindow(this);
        }

        private void CommandBinding_Executed_Close(object sender, ExecutedRoutedEventArgs e)
        {
            SystemCommands.CloseWindow(this);
        }

        private void CompositionTarget_Rendering(object sender, EventArgs e)
        {
            if (isPaused) 
                return;

            DateTime now = DateTime.Now;
            double deltaTime = (now - lastUpdate).TotalSeconds;

            if (deltaTime > 0.1) 
                deltaTime = 0.016;

            lastUpdate = now;

            // Use 2 sub-steps for more stable physics
            physicsEngine.Update(deltaTime, chkGravity.IsChecked == true, chkCollisions.IsChecked == true, 2);

            // Check for expired flashes
            CheckFlashExpiration();

            // Find fastest spinning object for display
            double maxSpin = 0;
            foreach (var obj in physicsEngine.GetObjects())
            {
                if (Math.Abs(obj.AngularVelocity) > maxSpin)
                {
                    maxSpin = Math.Abs(obj.AngularVelocity);
                }
            }

            // Update UI
            txtObjectCount.Text = physicsEngine.GetObjects().Count.ToString();
            txtStatus.Text = $"Objects: {physicsEngine.GetObjects().Count} | FPS: {(1 / deltaTime):F0} | Max Spin: {maxSpin:F1} rad/s";
        }

        private void PhysicsEngine_CollisionDetected(object sender, CollisionEventArgs e)
        {
            // Flash objects on collision
            Dispatcher.BeginInvoke(new Action(() =>
            {
                // Pass size ratio for visual effect
                FlashObjectWithSize(e.Object1, e.Object2, e.SizeRatio);
                FlashObjectWithSize(e.Object2, e.Object1, 1 / e.SizeRatio);

                System.Diagnostics.Debug.WriteLine($"Collision: {e.Object1.GetType().Name} with {e.Object2.GetType().Name} - Size ratio: {e.SizeRatio:F2}");
            }));
        }

        private void FlashObjectWithSize(PhysicsObject obj, PhysicsObject other, double sizeRatio)
        {
            if (flashingObjects.TryGetValue(obj, out FlashState state))
            {
                // Already flashing, increment counter and reset timer
                if (state.PendingFlashes < 10)
                    state.PendingFlashes++;
                state.LastFlashTime = DateTime.Now;
                System.Diagnostics.Debug.WriteLine($"Flash added to {obj.GetType().Name}: {state.PendingFlashes} pending");
            }
            else
            {
                // First flash
                state = new FlashState
                {
                    OriginalBrush = obj.Shape.Fill,
                    OriginalStroke = obj.Shape.Stroke,
                    OriginalThickness = obj.Shape.StrokeThickness,
                    LastFlashTime = DateTime.Now
                };

                // Calculate flash color based on size ratio
                // Smaller objects flash brighter
                byte intensity = (byte)Math.Max(180, 255 - (sizeRatio * 50));
                Color flashColor = Color.FromRgb(255, intensity, 0); // Orange-red for smaller, yellow for larger

                obj.Shape.Fill = new SolidColorBrush(flashColor);

                // Highlight based on size difference
                if (sizeRatio < 0.5)
                {
                    // Much smaller object - add extra visual emphasis
                    obj.Shape.Stroke = Brushes.Red;
                    obj.Shape.StrokeThickness = 4;
                }
                else if (sizeRatio > 2.0)
                {
                    // Much larger object - subtle effect
                    obj.Shape.Stroke = Brushes.Orange;
                    obj.Shape.StrokeThickness = 2;
                }
                else if (Math.Abs(obj.AngularVelocity) > 5)
                {
                    // Spinning fast
                    obj.Shape.Stroke = Brushes.Red;
                    obj.Shape.StrokeThickness = 3;
                }

                flashingObjects[obj] = state;
                System.Diagnostics.Debug.WriteLine($"First flash for {obj.GetType().Name}");                
            }
        }

        private void CheckFlashExpiration()
        {
            var now = DateTime.Now;
            var objectsToRemove = new List<PhysicsObject>();

            foreach (var kvp in flashingObjects)
            {
                var obj = kvp.Key;
                var state = kvp.Value;

                // Check if enough time has passed since last flash
                if ((now - state.LastFlashTime).TotalMilliseconds >= 150)
                {
                    state.PendingFlashes--;

                    if (state.PendingFlashes <= 0)
                    {
                        // No more pending flashes, restore original
                        if (obj.Shape != null)
                        {
                            obj.Shape.Fill = state.OriginalBrush;
                            obj.Shape.Stroke = state.OriginalStroke;
                            obj.Shape.StrokeThickness = state.OriginalThickness;
                        }
                        objectsToRemove.Add(obj);
                        System.Diagnostics.Debug.WriteLine($"Flash completed for {obj.GetType().Name}");
                    }
                    else
                    {
                        // Still have pending flashes, update last flash time
                        state.LastFlashTime = now;
                        System.Diagnostics.Debug.WriteLine($"Flash decremented for {obj.GetType().Name}: {state.PendingFlashes} remaining");
                    }
                }
            }

            // Clean up
            foreach (var obj in objectsToRemove)
            {
                flashingObjects.Remove(obj);
            }
        }

        private void AddBall()
        {
            int size = random.Next(30) + 10;
            var ball = new Ball(size, random.Next(50, (int)(physicsCanvas.ActualWidth - 70)), random.Next(50, (int)(physicsCanvas.ActualHeight - 70)));

            ball.Velocity = new Vector(random.Next(-300, 300), random.Next(-300, 300));

            physicsEngine.AddObject(ball);
            physicsCanvas.Children.Add(ball.Shape);

            Canvas.SetLeft(ball.Shape, ball.Position.X);
            Canvas.SetTop(ball.Shape, ball.Position.Y);
        }

        private void AddBox()
        {
            int size = random.Next(30) + 10;
            var box = new Box(size, size, random.Next(50, (int)(physicsCanvas.ActualWidth - 80)), random.Next(50, (int)(physicsCanvas.ActualHeight - 80)));

            box.Velocity = new Vector(random.Next(-150, 150), random.Next(-150, 150));

            physicsEngine.AddObject(box);
            physicsCanvas.Children.Add(box.Shape);

            Canvas.SetLeft(box.Shape, box.Position.X);
            Canvas.SetTop(box.Shape, box.Position.Y);
        }

        private void AddTriangle()
        {
            double baseWidth = random.Next(30) + 10;
            double height = random.Next(30) + 10;

            // Ensure triangle stays within bounds
            double x = random.Next(20, Math.Max(21, (int)(physicsCanvas.ActualWidth - baseWidth - 20)));
            double y = random.Next(20, Math.Max(21, (int)(physicsCanvas.ActualHeight - height - 20)));

            var triangle = new Triangle(baseWidth, height, x, y);

            triangle.Velocity = new Vector(random.Next(-100, 100), random.Next(-100, 100));
            triangle.AngularVelocity = random.Next(-2, 2); // Give it some initial spin

            physicsEngine.AddObject(triangle);
            physicsCanvas.Children.Add(triangle.Shape);

            // Log for debugging
            System.Diagnostics.Debug.WriteLine($"Triangle added at ({x}, {y}) with size {baseWidth}x{height}");
        }

        private void AddPolygon()
        {
            var random = new Random();

            // Get number of sides from combobox
            int sides = 5; // default
            if (cmbPolygonSides.SelectedItem != null)
            {
                sides = int.Parse((cmbPolygonSides.SelectedItem as ComboBoxItem).Content.ToString());
            }

            double radius = random.Next(20, 35);

            // Ensure polygon stays within bounds
            double x = random.Next(20, Math.Max(21, (int)(physicsCanvas.ActualWidth - radius * 2 - 20)));
            double y = random.Next(20, Math.Max(21, (int)(physicsCanvas.ActualHeight - radius * 2 - 20)));

            var polygon = new PolygonObject(sides, radius, x, y);

            polygon.Velocity = new Vector(random.Next(-100, 100), random.Next(-100, 100));
            polygon.AngularVelocity = random.Next(-3, 3); // Give it some initial spin

            physicsEngine.AddObject(polygon);
            physicsCanvas.Children.Add(polygon.Shape);

            System.Diagnostics.Debug.WriteLine($"{sides}-sided polygon added at ({x}, {y}) with radius {radius}");
        }

        private void btnAddBall_Click(object sender, RoutedEventArgs e)
        {
            AddBall();
        }

        private void btnAddBox_Click(object sender, RoutedEventArgs e)
        {
            AddBox();
        }

        private void btnAddTriangle_Click(object sender, RoutedEventArgs e)
        {
            AddTriangle();
        }

        private void btnAddPolygon_Click(object sender, RoutedEventArgs e)
        {
            AddPolygon();
        }

        private void btnClear_Click(object sender, RoutedEventArgs e)
        {
            physicsCanvas.Children.Clear();
            physicsEngine.ClearObjects();
        }

        private void btnPause_Click(object sender, RoutedEventArgs e)
        {
            isPaused = !isPaused;
            btnPause.Content = isPaused ? "Resume" : "Pause";
        }

        private void physicsCanvas_MouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            var position = e.GetPosition(physicsCanvas);

            // Check if we clicked on an object
            foreach (var obj in physicsEngine.GetObjects())
            {
                Rect bounds = obj.GetBounds();
                if (bounds.Contains(position.X, position.Y))
                {
                    draggedObject = obj;
                    dragOffset = new Vector(position.X - obj.Position.X, position.Y - obj.Position.Y);
                    lastMousePosition = new Vector(position.X, position.Y);
                    isDragging = true;
                    obj.IsStatic = true; // Temporarily make object static while dragging
                    break;
                }
            }
        }

        private void physicsCanvas_MouseMove(object sender, MouseEventArgs e)
        {
            if (isDragging && draggedObject != null)
            {
                var position = e.GetPosition(physicsCanvas);

                // Update object position
                draggedObject.Position = new Vector(
                    position.X - dragOffset.X,
                    position.Y - dragOffset.Y);

                // Calculate velocity based on mouse movement
                Vector currentMousePos = new Vector(position.X, position.Y);
                Vector velocity = (currentMousePos - lastMousePosition) / 0.016; // Approximate delta time

                // Limit velocity to prevent extreme speeds
                if (velocity.Length > 1000)
                {
                    velocity = velocity / velocity.Length * 1000;
                }

                draggedObject.Velocity = velocity;
                lastMousePosition = currentMousePos;
            }
        }

        private void physicsCanvas_MouseLeftButtonUp(object sender, MouseButtonEventArgs e)
        {
            if (draggedObject != null)
            {
                draggedObject.IsStatic = false;
                draggedObject = null;
                isDragging = false;
            }
        }

        protected override void OnRenderSizeChanged(SizeChangedInfo sizeInfo)
        {
            base.OnRenderSizeChanged(sizeInfo);

            // Update physics engine boundaries
            if (physicsEngine != null)
            {
                // Recreate physics engine with new dimensions
                physicsEngine = new PhysicsEngine(physicsCanvas.ActualWidth, physicsCanvas.ActualHeight);
                physicsEngine.CollisionDetected += PhysicsEngine_CollisionDetected;

                // Re-add objects to new engine
                var objects = physicsEngine.GetObjects();
                physicsEngine.ClearObjects();
                foreach (var obj in objects)
                {
                    physicsEngine.AddObject(obj);
                }
            }
        }
    }
}