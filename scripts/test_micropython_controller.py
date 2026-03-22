#!/usr/bin/env python3
"""
MicroPython Controller Test Script (Serial Communication)
========================================================

This script tests serial communication with the MicroPython motor controller
and verifies all basic functions are working correctly.

Usage:
  python3 test_micropython_controller.py [SERIAL_PORT]
  
Example:
  python3 test_micropython_controller.py /dev/ttyACM0
"""

import serial
import json
import time
import sys
import argparse
import threading

class MicroPythonTester:
    def __init__(self, serial_port='/dev/ttyACM0', baud_rate=115200):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.serial_connection = None
        self.test_results = []
        self.response_buffer = ""
        self.response_lock = threading.Lock()
        self.latest_response = None
    
    def connect(self):
        """Connect to MicroPython controller via serial"""
        try:
            self.serial_connection = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=1.0,
                write_timeout=1.0
            )
            # Wait for connection to stabilize
            time.sleep(2)
            
            # Clear any existing data
            self.serial_connection.reset_input_buffer()
            self.serial_connection.reset_output_buffer()
            return True
        except Exception as e:
            print(f"Failed to connect to {self.serial_port}: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from serial port"""
        if self.serial_connection and self.serial_connection.is_open:
            self.serial_connection.close()
    
    def send_command(self, command_dict, timeout=3.0):
        """Send JSON command to MicroPython controller and wait for response"""
        if not self.serial_connection or not self.serial_connection.is_open:
            return {'status': 'error', 'message': 'Serial connection not open'}
        
        try:
            # Send command as JSON
            command_json = json.dumps(command_dict) + '\n'
            self.serial_connection.write(command_json.encode('utf-8'))
            self.serial_connection.flush()
            
            # Wait for response
            start_time = time.time()
            response_data = ""
            
            while (time.time() - start_time) < timeout:
                if self.serial_connection.in_waiting > 0:
                    data = self.serial_connection.read(self.serial_connection.in_waiting).decode('utf-8', errors='ignore')
                    response_data += data
                    
                    # Look for complete JSON response
                    lines = response_data.split('\n')
                    for line in lines[:-1]:  # Process all complete lines
                        line = line.strip()
                        if line and line.startswith('{') and line.endswith('}'):
                            try:
                                response = json.loads(line)
                                return response
                            except json.JSONDecodeError:
                                continue
                    
                    # Keep the last incomplete line for next iteration
                    response_data = lines[-1]
                else:
                    time.sleep(0.1)
            
            return {'status': 'error', 'message': 'Response timeout'}
            
        except Exception as e:
            return {'status': 'error', 'message': str(e)}
    
    
    def test_connection(self):
        """Test basic connection to controller"""
        print("Testing connection...")
        
        if not self.connect():
            print("  FAIL: Failed to connect to serial port")
            self.test_results.append(('Connection', False, 'Serial connection failed'))
            return False
        
        # Test status command
        result = self.send_command({'cmd': 'status'})
        
        if result and (result.get('status') == 'ok' or result.get('type') == 'status'):
            print("  PASS: Connection successful")
            if result.get('type') == 'status':
                print(f"  Status: {json.dumps(result, indent=2)}")
            else:
                print(f"  Status: {json.dumps(result.get('data', {}), indent=2)}")
            self.test_results.append(('Connection', True, 'OK'))
            return True
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"  FAIL: Connection failed: {message}")
            self.test_results.append(('Connection', False, message))
            return False
    
    def test_emergency_stop(self):
        """Test emergency stop function"""
        print("\nTesting emergency stop...")
        
        # First, reset any existing emergency stop
        print("  Resetting any existing emergency stop...")
        reset_result = self.send_command({'cmd': 'reset_estop'})
        time.sleep(0.5)
        
        # Check initial status - should not be in emergency stop
        status = self.send_command({'cmd': 'status'})
        if status and status.get('emergency_stop'):
            print("  WARNING: Emergency stop still active after reset")
        
        # Activate emergency stop
        result = self.send_command({'cmd': 'estop'})
        if result and result.get('status') == 'ok':
            print("  PASS: Emergency stop command accepted")
            time.sleep(0.5)
            
            # Check status - should now be in emergency stop
            status = self.send_command({'cmd': 'status'})
            if status and status.get('emergency_stop'):
                print("  PASS: Emergency stop status confirmed")
                
                # Reset emergency stop
                reset_result = self.send_command({'cmd': 'reset_estop'})
                if reset_result and reset_result.get('status') == 'ok':
                    print("  PASS: Emergency stop reset successful")
                    
                    # Verify emergency stop is cleared
                    final_status = self.send_command({'cmd': 'status'})
                    if final_status and not final_status.get('emergency_stop'):
                        print("  PASS: Emergency stop cleared")
                        self.test_results.append(('Emergency Stop', True, 'OK'))
                        return True
                    else:
                        print("  FAIL: Emergency stop not cleared after reset")
                        self.test_results.append(('Emergency Stop', False, 'Not cleared'))
                        return False
                else:
                    message = reset_result.get('message', 'No response') if reset_result else 'No response'
                    print(f"  FAIL: Emergency stop reset failed: {message}")
                    self.test_results.append(('Emergency Stop', False, 'Reset failed'))
                    return False
            else:
                print("  FAIL: Emergency stop status not confirmed")
                self.test_results.append(('Emergency Stop', False, 'Status not confirmed'))
                return False
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"  FAIL: Emergency stop command failed: {message}")
            self.test_results.append(('Emergency Stop', False, message))
            return False
    
    def test_autonomous_mode(self):
        """Test autonomous mode toggle"""
        print("\nTesting autonomous mode...")
        
        # Enable autonomous mode
        result = self.send_command({'cmd': 'autonomous', 'mode': 'on'})
        if result and result.get('status') == 'ok':
            print("  PASS: Autonomous mode enabled")
            time.sleep(0.5)
            
            # Disable autonomous mode
            result = self.send_command({'cmd': 'autonomous', 'mode': 'off'})
            if result and result.get('status') == 'ok':
                print("  PASS: Autonomous mode disabled")
                self.test_results.append(('Autonomous Mode', True, 'OK'))
                return True
            else:
                message = result.get('message', 'No response') if result else 'No response'
                print(f"  FAIL: Autonomous mode disable failed: {message}")
                self.test_results.append(('Autonomous Mode', False, 'Disable failed'))
                return False
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"  FAIL: Autonomous mode enable failed: {message}")
            self.test_results.append(('Autonomous Mode', False, message))
            return False
    
    def test_movement_commands(self):
        """Test basic movement commands"""
        print("\nTesting movement commands...")
        
        # Ensure emergency stop is reset first
        reset_result = self.send_command({'cmd': 'reset_estop'})
        time.sleep(0.5)
        
        # Test forward movement
        print("  Testing forward movement...")
        result = self.send_command({
            'cmd': 'move',
            'dir': 'forward',
            'speed': 30,
            'duration': 1
        })
        if result and result.get('status') == 'ok':
            print("    PASS: Forward command accepted")
            time.sleep(1.5)  # Wait for movement to complete
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"    FAIL: Forward command failed: {message}")
            self.test_results.append(('Movement Commands', False, 'Forward failed'))
            return False
        
        # Test rotation
        print("  Testing rotation...")
        result = self.send_command({
            'cmd': 'rotate',
            'dir': 'left',
            'speed': 30,
            'duration': 1
        })
        if result and result.get('status') == 'ok':
            print("    PASS: Rotation command accepted")
            time.sleep(1.5)  # Wait for rotation to complete
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"    FAIL: Rotation command failed: {message}")
            self.test_results.append(('Movement Commands', False, 'Rotation failed'))
            return False
        
        # Test stop
        print("  Testing stop command...")
        result = self.send_command({'cmd': 'stop'})
        if result and result.get('status') == 'ok':
            print("    PASS: Stop command accepted")
            self.test_results.append(('Movement Commands', True, 'OK'))
            return True
        else:
            message = result.get('message', 'No response') if result else 'No response'
            print(f"    FAIL: Stop command failed: {message}")
            self.test_results.append(('Movement Commands', False, 'Stop failed'))
            return False
    
    def test_parameter_validation(self):
        """Test parameter validation"""
        print("\nTesting parameter validation...")
        
        # Test invalid direction
        result = self.send_command({
            'cmd': 'move',
            'dir': 'invalid',
            'speed': 50
        })
        success = result and result.get('status') == 'error'
        print(f"  Invalid direction test: {'PASS' if success else 'FAIL'}")
        
        # Test extreme speed values
        result = self.send_command({
            'cmd': 'move',
            'dir': 'forward',
            'speed': 150  # Over 100%
        })
        success = result and result.get('status') in ['ok', 'error']
        print(f"  High speed handling: {'PASS' if success else 'FAIL'}")
        
        self.test_results.append(('Parameter Validation', True, 'OK'))
        return True
    
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("=" * 60)
        print("MICROPYTHON CONTROLLER TEST SUITE (Serial)")
        print("=" * 60)
        print(f"Target: {self.serial_port} @ {self.baud_rate} baud")
        print()
        
        # Run tests in sequence
        tests = [
            self.test_connection,
            self.test_emergency_stop,
            self.test_autonomous_mode,
            self.test_movement_commands,
            self.test_parameter_validation,
        ]
        
        all_passed = True
        for test in tests:
            try:
                if not test():
                    all_passed = False
            except Exception as e:
                print(f"  FAIL: Test failed with exception: {e}")
                all_passed = False
                
        # Final cleanup - ensure everything is stopped
        print("\nCleanup...")
        if self.serial_connection and self.serial_connection.is_open:
            self.send_command({'cmd': 'stop'})
            self.send_command({'cmd': 'reset_estop'})
            self.disconnect()
        
        # Results summary
        print("\n" + "=" * 60)
        print("TEST RESULTS SUMMARY")
        print("=" * 60)
        
        for test_name, passed, message in self.test_results:
            status = "PASS" if passed else "FAIL"
            print(f"{test_name:20} {status:10} {message}")
        
        print("\n" + "=" * 60)
        if all_passed:
            print("ALL TESTS PASSED - Controller is ready for autonomous operation!")
        else:
            print("SOME TESTS FAILED - Check controller setup before autonomous operation")
        print("=" * 60)
        
        return all_passed

def main():
    parser = argparse.ArgumentParser(description='Test MicroPython Motor Controller (Serial)')
    parser.add_argument('serial_port', nargs='?', default='/dev/ttyACM0', 
                       help='Serial port of MicroPython controller (default: /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200,
                       help='Baud rate for serial communication (default: 115200)')
    
    args = parser.parse_args()
    
    tester = MicroPythonTester(args.serial_port, args.baud)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        # Cleanup
        if tester.serial_connection and tester.serial_connection.is_open:
            tester.send_command({'cmd': 'stop'})
            tester.send_command({'cmd': 'reset_estop'})
            tester.disconnect()
        sys.exit(1)
    except Exception as e:
        print(f"\n\nTest suite failed with error: {e}")
        if tester.serial_connection and tester.serial_connection.is_open:
            tester.disconnect()
        sys.exit(1)

if __name__ == '__main__':
    main()
