\# Sensor Streaming



Focusing on a concrete demonstration program will solidify the core infrastructure—the Zenoh data fabric and the CMake setup, before tackling complex autonomy.



\*\*Phase 1 Demonstration: Sensor Data Streaming\*\*, establishes the fundamental communication link between the simulator (AirSim or Isaac) and a basic data viewer/consumer using Zenoh.



-----



\## Demonstration: Sensor Data Streaming



This demonstration involves two primary executables that communicate via a Zenoh network:



1\.  \*\*The Publisher (Simulator Client):\*\* Connects to the simulation API, captures sensor data (e.g., a camera image), serializes it, and publishes it to a Zenoh key.

2\.  \*\*The Subscriber (Data Viewer):\*\* Connects to the same Zenoh network, subscribes to the key, deserializes the data, and displays it.



\### 1. The Zenoh Key Structure



We will use a structured Zenoh key space to organize different sensor types and robots.



| Component | Zenoh Key Example | Data Type | Purpose |

| :--- | :--- | :--- | :--- |

| \*\*RGB Camera\*\* | `robot/drone/sensor/camera/rgb` | Raw Image Bytes | High-bandwidth visual feedback. |

| \*\*IMU/Odometry\*\* | `robot/drone/sensor/state/odom` | Custom Data Structure (Pose \& Velocity) | Low-bandwidth state information. |

| \*\*Control Command\*\* | `robot/drone/cmd/velocity` | Custom Data Structure (Vec3) | Future demonstration link (teleop input). |



\### 2. Implementation Details



\#### A. Data Serialization and Types (`libs/data\_types/`)



To efficiently send complex data like images and custom structures over Zenoh, you'll need serialization utilities.



&nbsp; \* \*\*For Images:\*\* Serializing an image (e.g., from OpenCV or the simulator API) into a raw byte array (`std::vector<uint8\_t>`) is essential. We will define a standard header for image metadata (width, height, encoding) followed by the raw pixel data.

&nbsp; \* \*\*For Custom Structures (Odometry):\*\* Define a standard C++ structure and implement simple binary serialization/deserialization functions.



<!-- end list -->



```cpp

// libs/data\_types/Odometry.h

struct Odometry {

&nbsp;   float x, y, z;      // Position

&nbsp;   float roll, pitch, yaw; // Orientation

&nbsp;   // ... other fields

&nbsp;   std::vector<uint8\_t> serialize() const;

&nbsp;   static Odometry deserialize(const std::vector<uint8\_t>\& data);

};

```



\#### B. The Simulator Client (`sim\_interfaces/airsim\_client/`)



This library will contain the main execution logic for publishing data.



1\.  \*\*Initialize Zenoh:\*\* Establish a Zenoh session.

2\.  \*\*Initialize Simulator:\*\* Connect to the AirSim/Isaac API.

3\.  \*\*Main Loop:\*\*

&nbsp;     \* Poll the simulator for the latest sensor data (e.g., `simGetImage()` in AirSim).

&nbsp;     \* Use the serialization utility to convert the image into a byte vector.

&nbsp;     \* Use the \*\*Zenoh Interface Library\*\* to publish the data.



<!-- end list -->



```cpp

// Pseudo-Code for the Publisher Executable

int main() {

&nbsp;   auto zenoh\_session = zenoh\_interface::init\_session();

&nbsp;   auto airsim\_api = airsim\_client::connect();

&nbsp;   

&nbsp;   while (true) {

&nbsp;       // 1. Get data

&nbsp;       auto raw\_image = airsim\_api.capture\_rgb();

&nbsp;       

&nbsp;       // 2. Serialize (raw bytes + metadata)

&nbsp;       auto payload = data\_types::serialize\_image(raw\_image);

&nbsp;       

&nbsp;       // 3. Publish via Zenoh interface

&nbsp;       zenoh\_interface::publish(zenoh\_session, "robot/drone/sensor/camera/rgb", payload);



&nbsp;       std::this\_thread::sleep\_for(10ms);

&nbsp;   }

}

```



\#### C. The Data Viewer (`demonstrations/01\_sensor\_streaming/`)



This executable connects to Zenoh and processes the incoming data.



1\.  \*\*Initialize Zenoh:\*\* Establish a Zenoh session.

2\.  \*\*Subscriber Callback:\*\* Define a callback function that is executed every time new data arrives on the subscribed key.

3\.  \*\*Visualization:\*\* Inside the callback, deserialize the image data and display it (e.g., using OpenCV's `imshow`).



<!-- end list -->



```cpp

// Pseudo-Code for the Subscriber Executable

void image\_callback(const zenohc\_sample\_t\* sample) {

&nbsp;   // 1. Deserialize

&nbsp;   auto image\_data = data\_types::deserialize\_image(sample->payload);



&nbsp;   // 2. Display

&nbsp;   cv::imshow("Drone Camera Feed", image\_data);

&nbsp;   cv::waitKey(1);

}



int main() {

&nbsp;   auto zenoh\_session = zenoh\_interface::init\_session();

&nbsp;   

&nbsp;   // 3. Subscribe to the camera feed key

&nbsp;   zenoh\_interface::subscribe(zenoh\_session, "robot/drone/sensor/camera/rgb", image\_callback);

&nbsp;   

&nbsp;   // Keep the main thread alive

&nbsp;   std::cout << "Zenoh Subscriber running. Press CTRL+C to exit." << std::endl;

&nbsp;   // ... keep running logic

}

```



\### 3. CMake Integration Example



The core of the strategy is using CMake to manage these dependencies neatly.



\#### `libs/zenoh\_interface/CMakeLists.txt`



```cmake

\# Create the zenoh interface library

add\_library(zenoh\_interface STATIC 

&nbsp;   ZenohInterface.cpp 

&nbsp;   ZenohInterface.h

)



\# Link the Zenoh C/C++ library found earlier (e.g., zenohc::zenoh)

target\_link\_libraries(zenoh\_interface PUBLIC 

&nbsp;   zenohc::zenoh

)

```



\#### `demonstrations/01\_sensor\_streaming/CMakeLists.txt`



```cmake

\# Create the executable for the subscriber/viewer

add\_executable(viewer\_node 

&nbsp;   ViewerNode.cpp

)



\# Link the executable to the necessary custom and third-party libraries

target\_link\_libraries(viewer\_node PRIVATE

&nbsp;   zenoh\_interface        # For Zenoh communication

&nbsp;   data\_types             # For serialization/deserialization

&nbsp;   opencv                 # For image display (assuming OpenCV is found)

)

```



This structure creates a clean, decoupled system: `zenoh\_interface` handles the communication plumbing, `data\_types` handles the data format, and the nodes (`viewer\_node`, `airsim\_client`) handle the application logic.





