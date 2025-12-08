# Sensor Streaming

Focusing on a concrete demonstration program will solidify the core infrastructure—the Zenoh data fabric and the CMake setup, before tackling complex autonomy.

**Phase 1 Demonstration: Sensor Data Streaming**, establishes the fundamental communication link between the simulator (AirSim or Isaac) and a basic data viewer/consumer using Zenoh.

-----

## Demonstration: Sensor Data Streaming

This demonstration involves two primary executables that communicate via a Zenoh network:

1.  **The Publisher (Simulator Client):** Connects to the simulation API, captures sensor data (e.g., a camera image), serializes it, and publishes it to a Zenoh key.

2.  **The Subscriber (Data Viewer):** Connects to the same Zenoh network, subscribes to the key, deserializes the data, and displays it.

### 1. The Zenoh Key Structure

We will use a structured Zenoh key space to organize different sensor types and robots.

| Component | Zenoh Key Example | Data Type | Purpose |
| :--- | :--- | :--- | :--- |
| **RGB Camera** | `robot/drone/sensor/camera/rgb` | Raw Image Bytes | High-bandwidth visual feedback. |
| **IMU/Odometry** | `robot/drone/sensor/state/odom` | Custom Data Structure (Pose \& Velocity) | Low-bandwidth state information. |
| **Control Command** | `robot/drone/cmd/velocity` | Custom Data Structure (Vec3) | Future demonstration link (teleop input). |

### 2. Implementation Details

#### A. Data Serialization and Types (`libs/data_types/`)

To efficiently send complex data like images and custom structures over Zenoh, you'll need serialization utilities.

 * **For Images:** Serializing an image (e.g., from OpenCV or the simulator API) into a raw byte array (`std::vector<uint8_t>`) is essential. We will define a standard header for image metadata (width, height, encoding) followed by the raw pixel data.

 * **For Custom Structures (Odometry):** Define a standard C++ structure and implement simple binary serialization/deserialization functions.

```cpp
// libs/data_types/Odometry.h
struct Odometry {
   float x, y, z;      // Position
   float roll, pitch, yaw; // Orientation
   // ... other fields
   std::vector<uint8_t> serialize() const;
   static Odometry deserialize(const std::vector<uint8_t>\& data);
};
```

#### B. The Simulator Client (`sim_interfaces/airsim_client/`)

This library will contain the main execution logic for publishing data.

1.  **Initialize Zenoh:** Establish a Zenoh session.

2.  **Initialize Simulator:** Connect to the AirSim/Isaac API.

3.  **Main Loop:**

     * Poll the simulator for the latest sensor data (e.g., `simGetImage()` in AirSim).
     * Use the serialization utility to convert the image into a byte vector.
     * Use the **Zenoh Interface Library** to publish the data.

```cpp
// Pseudo-Code for the Publisher Executable
int main() {
   auto zenoh_session = zenoh_interface::init_session();
   auto airsim_api = airsim_client::connect();

   while (true) {
       // 1. Get data
       auto raw_image = airsim_api.capture_rgb();

       // 2. Serialize (raw bytes + metadata)
       auto payload = data_types::serialize_image(raw_image);

       // 3. Publish via Zenoh interface
       zenoh_interface::publish(zenoh_session, "robot/drone/sensor/camera/rgb", payload);

       std::this_thread::sleep_for(10ms);
   }
}

```
#### C. The Data Viewer (`demonstrations/01_sensor_streaming/`)

This executable connects to Zenoh and processes the incoming data.

1.  **Initialize Zenoh:** Establish a Zenoh session.

2.  **Subscriber Callback:** Define a callback function that is executed every time new data arrives on the subscribed key.

3.  **Visualization:** Inside the callback, deserialize the image data and display it (e.g., using OpenCV's `imshow`).

```cpp
// Pseudo-Code for the Subscriber Executable
void image_callback(const zenohc_sample_t* sample) {
   // 1. Deserialize
   auto image_data = data_types::deserialize_image(sample->payload);

   // 2. Display
   cv::imshow("Drone Camera Feed", image_data);

   cv::waitKey(1);
}

int main() {
   auto zenoh_session = zenoh_interface::init_session();

   // 3. Subscribe to the camera feed key
   zenoh_interface::subscribe(zenoh_session, "robot/drone/sensor/camera/rgb", image_callback);

   // Keep the main thread alive
   std::cout << "Zenoh Subscriber running. Press CTRL+C to exit." << std::endl;

   // ... keep running logic
}
```

### 3. CMake Integration Example

The core of the strategy is using CMake to manage these dependencies neatly.

#### `libs/zenoh_interface/CMakeLists.txt`

```cmake
# Create the zenoh interface library
add_library(zenoh_interface STATIC 
   ZenohInterface.cpp 
   ZenohInterface.h
)

# Link the Zenoh C/C++ library found earlier (e.g., zenohc::zenoh)
target_link_libraries(zenoh_interface PUBLIC 
   zenohc::zenoh
)

```

#### `demonstrations/01_sensor_streaming/CMakeLists.txt`

```cmake
# Create the executable for the subscriber/viewer
add_executable(viewer_node 
   ViewerNode.cpp
)

# Link the executable to the necessary custom and third-party libraries
target_link_libraries(viewer_node PRIVATE
   zenoh_interface        # For Zenoh communication
   data_types             # For serialization/deserialization
   opencv                 # For image display (assuming OpenCV is found)
)

```

This structure creates a clean, decoupled system: `zenoh_interface` handles the communication plumbing, `data_types` handles the data format, and the nodes (`viewer_node`, `airsim_client`) handle the application logic.