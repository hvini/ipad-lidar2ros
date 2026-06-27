# iPhone LiDAR to ROS

An iOS application that streams high-density point clouds, depth maps, camera frames, and ARKit transforms directly to a ROS environment using WebSockets.

---

> This project is a heavily modified fork of the original [ipad-lidar2ros](https://github.com/christophebedard/ipad-lidar2ros) by Christophe Bedard.

### Key Modifications
- **Dense Point Cloud**: Replaced the sparse ARKit tracking feature points with a high-density point cloud natively unprojected from the LiDAR `sceneDepth` depth map.
- **Portrait Optimization**: Recalculated TF transformations and axis math specifically for iPhone Portrait orientation.
- **Network Optimization**: Implemented a 2x decimation algorithm to reduce the dense point payload by 75%, allowing stutter-free real-time streaming over `rosbridge`.
- **UI & Lifecycle**: Stripped out the 3D preview, introduced a UI redesign, added automatic WebSocket connections, and improved iOS background state handling to prevent zombie ROS nodes.

## Xcode Project Setup

To compile the application, you need to set up the Xcode project using the provided source files.

1. **Create Project**: Open Xcode and select **Create a new Xcode project**.
2. **Choose Template**: Select **iOS** > **App** and click Next.
3. **Project Details**: 
   - Product Name: `ipad-lidar2ros`
   - Interface: **Storyboard**
   - Language: **Swift**
4. **Import Source Files**: Delete the default `ViewController.swift`, `AppDelegate.swift`, and `SceneDelegate.swift` files created by Xcode. Drag and drop the entire `src/` directory from this repository into your Xcode project navigator.
5. **Set Info.plist**: The app requires specific permissions (like Local Network and Camera usage). Go to your project settings, select your Target, navigate to the **Build Settings** tab, search for `Info.plist File`, and set its path to `src/Info.plist`.
6. **Signing Capabilities**: Go to the **Signing & Capabilities** tab and select your Apple developer team to allow deploying to a physical device.

## App Configuration

Before building the app in Xcode, you must configure the IP address of your ROS machine. 
Open `src/RosControllerViewProvider.swift` and update the `hardcodedUrl` constant to match the IP address of the machine running the rosbridge server.

```swift
private let hardcodedUrl = "192.168.X.X:9090"
```

Build and run the application on a physical iOS device equipped with a LiDAR scanner (iPhone 12 Pro or newer, iPad Pro 2020 or newer).

## Getting Started with Docker

You can easily set up the entire receiving environment using Docker. This will run both the `rosbridge_server` (to receive data from the iOS app) and `foxglove-bridge` (to visualize the data) in the same container instance.

### 1. Start the ROS Docker Container
Open a terminal and start an interactive ROS container with the necessary ports exposed. We will use the standard ROS Humble image (or any other ROS version you prefer).

```bash
docker run -it --rm -p 9090:9090 -p 8765:8765 --name ros_lidar_env ros:humble /bin/bash
```

### 2. Install Dependencies
Inside the container, update `apt` and install the required bridges:

```bash
apt update
apt install -y ros-humble-rosbridge-suite
```

### 3. Run Rosbridge (Terminal 1)
In the container, launch the `rosbridge_websocket`:

```bash
source /opt/ros/humble/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml address:=0.0.0.0
```
The server will start listening on port `9090`. Your iOS app is configured to connect to this port by default.

### 4. Run Foxglove Bridge (Terminal 2)
Open a new terminal on your host machine and attach it to the running container:

```bash
docker exec -it ros_lidar_env /bin/bash
apt update
apt install -y ros-humble-foxglove-bridge
source /opt/ros/humble/setup.bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml address:=0.0.0.0 port:=8765
```

### 5. Visualize the Sensors
To visualize the data, open [Foxglove Studio](https://foxglove.dev/studio), open a new connection, select **Foxglove WebSocket**, and enter your computer's IP address (e.g., `ws://192.168.x.x:8765`).

The following topics are available to subscribe to:

- `/ipad/pointcloud`: Dense point cloud from the LiDAR.
- `/ipad/camera`: Camera frames.
- `/ipad/depth`: Depth maps.
- `/tf`: Coordinate transformations from ARKit.

Configure your Foxglove workspace by adding the following panels:
- **3D Panel**: Set the Global Frame to `ipad`. Click the settings gear and add the `/ipad/pointcloud` topic to visualize the dense LiDAR stream.
- **Image Panels**: Add Image panels and subscribe to `/ipad/camera` and `/ipad/depth`.
- **Raw Messages**: Add a raw messages panel for `/tf` to observe the coordinate transformations.
