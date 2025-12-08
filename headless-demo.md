Headless Demo

Created viewer_node_headless which works without a display:

# Terminal 1: Publisher
  ./build/linux-release/demos/01_sensor_streaming/mock_publisher

# Terminal 2: Headless viewer (console output only)
  ./build/linux-release/demos/01_sensor_streaming/viewer_node_headless

# With periodic image saving
  ./build/linux-release/demos/01_sensor_streaming/viewer_node_headless --save-images --output-dir ./frames --save-interval 30

Options:
  - --save-images - Save received frames to disk
  - --output-dir <path> - Directory for saved images (default: ./captured_frames)
  - --save-interval <n> - Save every N frames (default: 30)

