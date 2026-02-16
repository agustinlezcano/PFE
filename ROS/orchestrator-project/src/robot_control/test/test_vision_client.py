#!/usr/bin/env python3
"""
Test client for vision server socket communication.
Tests the connection and data flow independently of ROS.

Usage:
    1. Start mock_vision_server.py in one terminal
    2. Run this test in another terminal: python3 test_vision_client.py
"""

import sys
import os
# Add parent directory to path to import image_manager
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from robot_control.image_manager import VisionClient, VisionClientNode
import time


def test_vision_client():
    """Test VisionClient direct synchronous calls."""
    print("=" * 60)
    print("TEST 1: VisionClient (Synchronous)")
    print("=" * 60)
    
    client = VisionClient("127.0.0.1", 65432, timeout=5.0, max_retries=3)
    
    # Test connection
    print("\n[Test] Connecting to vision server...")
    success, msg = client.connect()
    print(f"[Result] {msg}")
    
    if not success:
        print("[Error] Could not connect. Make sure mock_vision_server.py is running.")
        return False
    
    # Test object requests
    test_objects = ["tornillo", "tuerca", "llave", "unknown_object"]
    
    for obj_name in test_objects:
        print(f"\n[Test] Requesting object: {obj_name}")
        coordinates, msg = client.request_object(obj_name)
        
        if coordinates:
            x, y, z = coordinates
            print(f"[Success] {msg}")
            print(f"[Coordinates] X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
        else:
            print(f"[Failed] {msg}")
    
    # Close connection
    print("\n[Test] Closing connection...")
    client.close()
    print("[Result] Connection closed")
    
    return True


def test_vision_client_node():
    """Test VisionClientNode threaded asynchronous calls."""
    print("\n\n" + "=" * 60)
    print("TEST 2: VisionClientNode (Asynchronous/Threaded)")
    print("=" * 60)
    
    node = VisionClientNode("127.0.0.1", 65432, timeout=5.0, max_retries=3)
    
    # Start node
    print("\n[Test] Starting vision client node...")
    success, msg = node.start()
    print(f"[Result] {msg}")
    
    if not success:
        print("[Error] Could not start node. Make sure mock_vision_server.py is running.")
        return False
    
    # Send async requests
    test_objects = ["tornillo", "tuerca", "llave"]
    
    print("\n[Test] Sending async requests for objects...")
    for obj_name in test_objects:
        print(f"  - Queuing request: {obj_name}")
        node.request_object_async(obj_name)
    
    # Poll for responses
    print("\n[Test] Polling for responses...")
    received = 0
    max_polls = 50  # 5 seconds at 0.1s intervals
    
    for i in range(max_polls):
        response = node.get_response()
        
        if response:
            received += 1
            print(f"\n[Response {received}] Object: {response['name']}")
            print(f"  Success: {response['success']}")
            print(f"  Message: {response['message']}")
            
            if response['coordinates']:
                x, y, z = response['coordinates']
                print(f"  Coordinates: X={x:.2f}, Y={y:.2f}, Z={z:.2f}")
            
            if received >= len(test_objects):
                break
        
        time.sleep(0.1)
    
    if received < len(test_objects):
        print(f"\n[Warning] Only received {received}/{len(test_objects)} responses")
    
    # Stop node
    print("\n[Test] Stopping vision client node...")
    node.stop()
    print("[Result] Node stopped")
    
    return True


def main():
    """Run all tests."""
    print("\n" + "=" * 60)
    print("VISION CLIENT TEST SUITE")
    print("=" * 60)
    print("\nPrerequisite: mock_vision_server.py must be running")
    print("If not running, start it in another terminal:\n")
    print("  cd test/")
    print("  python3 mock_vision_server.py\n")
    
    input("Press Enter when server is ready...")
    
    # Run tests
    test1_passed = test_vision_client()
    
    if test1_passed:
        time.sleep(1)  # Brief pause between tests
        test2_passed = test_vision_client_node()
    else:
        test2_passed = False
    
    # Summary
    print("\n\n" + "=" * 60)
    print("TEST SUMMARY")
    print("=" * 60)
    print(f"Test 1 (VisionClient):     {'✓ PASSED' if test1_passed else '✗ FAILED'}")
    print(f"Test 2 (VisionClientNode): {'✓ PASSED' if test2_passed else '✗ FAILED'}")
    print("=" * 60)
    
    return test1_passed and test2_passed


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
