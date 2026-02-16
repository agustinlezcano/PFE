# Vision Socket Client Integration

## Overview

This implementation integrates a socket-based vision server client into the ROS2 robot control system. The vision client requests object coordinates from an external vision server and feeds them into the trajectory planning pipeline.

## Architecture

### Components

1. **VisionClient** ([image_manager.py](../robot_control/image_manager.py))
   - Low-level socket communication
   - Persistent connection with reconnection logic
   - Retry mechanism for failed requests
   - Thread-safe with locking

2. **VisionClientNode** ([image_manager.py](../robot_control/image_manager.py))
   - Threaded wrapper for non-blocking operation
   - Queue-based async request/response pattern
   - Daemon thread (compatible with ROS2 rclpy.spin())

3. **Publisher Integration** ([publisher.py](../robot_control/publisher.py))
   - Vision client initialized on node startup
   - 10 Hz timer polling for vision responses
   - Automatic object list processing
   - State machine integration (IDLE → BLOCKED → RUNNING)

### Communication Protocol

**Request Format:** Plain text object name
```
tornillo
```

**Response Format:** Pattern `X###Y###Z###`
```
X150.50Y200.30Z50.00
```

### Workflow

1. Load objects list from CSV: `tuerca,tornillo,llave`
2. Auto-trigger: `request_next_object()` called
3. Send object name to vision server via socket
4. State set to `BLOCKED` waiting for response
5. Timer callback polls for response (10 Hz)
6. Validate coordinates with `LimitsValidator`
7. Publish to trajectory planner (`/ROS/trajectory_planning`)
8. State → `RUNNING` during trajectory execution
9. On completion, request next object (loop to step 2)

## Configuration

### Vision Server Connection

Defaults in [publisher.py](../robot_control/publisher.py#L46-L47):
```python
self.vision_client = VisionClientNode("127.0.0.1", 65432, timeout=5.0, max_retries=3)
```

**Parameters:**
- `host`: Vision server IP (default: `127.0.0.1`)
- `port`: Vision server port (default: `65432`)
- `timeout`: Socket timeout in seconds (default: `5.0`)
- `max_retries`: Retry attempts for failed requests (default: `3`)

### Error Handling

| Error Scenario | Behavior |
|---------------|----------|
| Connection failure on startup | Logs warning, continues without vision (degraded mode) |
| Connection lost during operation | Automatic reconnection attempt |
| Coordinate validation fails | Retry same object |
| Max retries exceeded | Log error, skip to next object |
| Object not found by vision | Skip to next object |
| Objects list exhausted | State → IDLE |

## Usage

### 1. Setup Mock Vision Server (Testing)

Start the mock server in one terminal:
```bash
cd test/
python3 mock_vision_server.py
```

Expected output:
```
[Mock Vision Server] Starting on 127.0.0.1:65432
[Mock Vision Server] Known objects: tornillo, tuerca, llave
[Mock Vision Server] Listening for connections...
```

### 2. Test Socket Communication (Optional)

Verify socket communication before ROS:
```bash
cd test/
python3 test_vision_client.py
```

This tests:
- VisionClient synchronous communication
- VisionClientNode async/threaded operation
- Request/response parsing

### 3. Prepare Objects CSV

Edit [items.csv](../robot_control/items.csv):
```csv
tornillo,tuerca,llave,tornillo
```

Format: comma-separated object names (no header)

### 4. Build and Run ROS Node

Build the package:
```bash
cd /home/agustin/UNCUYO/PFE/ROS/orchestrator-project
colcon build --packages-select robot_control
source install/setup.bash
```

Launch publisher node:
```bash
ros2 run robot_control talker
```

### 5. Load Objects List

From robot UI or via ROS topic:
```bash
ros2 topic pub /robot_ui/load_csv std_msgs/msg/String "{data: '/path/to/items.csv'}" --once
```

Or use the robot_ui_node interface.

### 6. Monitor Execution

Watch logs:
```bash
ros2 run robot_control talker
```

Expected log sequence:
```
[INFO] Vision client: Connected to vision server at 127.0.0.1:65432
[INFO] Lista de objetos cargada: 4 elementos
[INFO] Solicitando objeto: tornillo (ID: 0)
[INFO] Vision response: Object 'tornillo' located at X=150.5, Y=200.3, Z=50.0
[INFO] Respuesta de visión validada: x=150.5000, y=200.3000, z=50.0000
[INFO] Objetivo enviado al trajectory planner para tornillo
```

Monitor trajectory planning topic:
```bash
ros2 topic echo /ROS/trajectory_planning
```

## Testing

### Unit Tests

Test socket communication independently:
```bash
# Terminal 1: Start mock server
python3 test/mock_vision_server.py

# Terminal 2: Run tests
python3 test/test_vision_client.py
```

### Integration Tests

Test with ROS2:
```bash
# Terminal 1: Mock vision server
python3 test/mock_vision_server.py

# Terminal 2: Launch nodes
ros2 run robot_control talker

# Terminal 3: Load CSV
ros2 topic pub /robot_ui/load_csv std_msgs/msg/String "{data: '/absolute/path/to/items.csv'}" --once

# Terminal 4: Monitor topics
ros2 topic echo /ROS/trajectory_planning
```

### Debug Mode

Enable debug logging in publisher node:
```python
# In publisher.py __init__
self.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
```

## Troubleshooting

### "Connection refused" on startup

**Problem:** Vision server not running  
**Solution:** Start `mock_vision_server.py` before launching ROS node  
**Note:** Node continues in degraded mode (vision features disabled)

### "Timeout waiting for response"

**Problem:** Vision server slow or unresponsive  
**Solution:** Increase timeout in vision client initialization:
```python
self.vision_client = VisionClientNode("127.0.0.1", 65432, timeout=10.0)
```

### "Coordenadas fuera de límites"

**Problem:** Vision returns coordinates outside robot workspace  
**Solution:** Check `LimitsValidator` configuration or adjust mock server coordinates

### State stuck in BLOCKED

**Problem:** Vision response not processed  
**Solution:** 
- Check vision server logs for errors
- Verify response format matches `X###Y###Z###` pattern
- Check `check_vision_response_callback()` is running (10 Hz timer)

### Objects list not triggering

**Problem:** `request_next_object()` not called  
**Solution:** 
- Verify CSV loaded successfully: check logs for "Lista de objetos cargada"
- Ensure robot state is IDLE when loading CSV
- Check `objects_reader` is initialized (bug fix applied)

## Files Modified/Created

### Modified
- [robot_control/image_manager.py](../robot_control/image_manager.py) - Added VisionClient and VisionClientNode
- [robot_control/publisher.py](../robot_control/publisher.py) - Integrated vision client, implemented workflow

### Created
- [test/mock_vision_server.py](mock_vision_server.py) - Mock server for testing
- [test/test_vision_client.py](test_vision_client.py) - Unit/integration tests
- [test/VISION_INTEGRATION.md](VISION_INTEGRATION.md) - This documentation

## Future Enhancements

- [ ] Support JSON response format for richer data
- [ ] Add camera calibration data exchange
- [ ] Implement coordinate frame transformations (camera → robot)
- [ ] Add vision feedback (object grabbed confirmation)
- [ ] SSL/TLS encryption for production
- [ ] Heartbeat mechanism for connection monitoring
- [ ] Multiple vision server failover
- [ ] Object priority queue instead of sequential processing

## Contact

For issues or questions about this implementation, refer to:
- Vision server specification: [TODO: add link]
- ROS2 topics documentation: `ROS_TOPICS_DOCUMENTATION.csv`
- Limits validation: [limits_validator.py](../robot_control/limits_validator.py)
