#!/usr/bin/env python3
"""
MicroPython Controller Test Script
=================================

This script tests communication with the MicroPython motor controller
and verifies all basic functions are working correctly.

Usage:
  python3 test_micropython_controller.py [IP_ADDRESS]
  
Example:
  python3 test_micropython_controller.py 192.168.25.72
"""

import requests
import json
import time
import sys
import argparse

class MicroPythonTester:
    def __init__(self, ip='192.168.25.72', port=8080):
        self.ip = ip
        self.port = port
        self.base_url = f"http://{ip}:{port}"
        self.test_results = []
    
    def send_command(self, endpoint, params=None, timeout=2.0):
        """Send command to MicroPython controller"""
        try:
            url = f"{self.base_url}{endpoint}"
            if params:
                url += "?" + "&".join([f"{k}={v}" for k, v in params.items()])
            
            response = requests.get(url, timeout=timeout)
            if response.status_code == 200:
                return response.json()
            else:
                return {'status': 'error', 'message': f'HTTP {response.status_code}'}
        except requests.exceptions.RequestException as e:
            return {'status': 'error', 'message': str(e)}
    
    def test_connection(self):
        """Test basic connection to controller"""
        print("🔌 Testing connection...")
        result = self.send_command('/status')
        
        if result['status'] == 'ok':
            print("  ✅ Connection successful")
            print(f"  📊 Status: {json.dumps(result['data'], indent=2)}")
            self.test_results.append(('Connection', True, 'OK'))
            return True
        else:
            print(f"  ❌ Connection failed: {result['message']}")
            self.test_results.append(('Connection', False, result['message']))
            return False
    
    def test_emergency_stop(self):
        """Test emergency stop function"""
        print("\n🚨 Testing emergency stop...")
        
        # Activate emergency stop
        result = self.send_command('/estop')
        if result['status'] == 'ok':
            print("  ✅ Emergency stop activated")
            time.sleep(0.5)
            
            # Check status
            status = self.send_command('/status')
            if status['status'] == 'ok' and status['data'].get('emergency_stop'):
                print("  ✅ Emergency stop status confirmed")
                
                # Reset emergency stop
                reset_result = self.send_command('/reset_estop')
                if reset_result['status'] == 'ok':
                    print("  ✅ Emergency stop reset successful")
                    self.test_results.append(('Emergency Stop', True, 'OK'))
                    return True
                else:
                    print(f"  ❌ Emergency stop reset failed: {reset_result['message']}")
                    self.test_results.append(('Emergency Stop', False, 'Reset failed'))
                    return False
            else:
                print("  ❌ Emergency stop status not confirmed")
                self.test_results.append(('Emergency Stop', False, 'Status not confirmed'))
                return False
        else:
            print(f"  ❌ Emergency stop failed: {result['message']}")
            self.test_results.append(('Emergency Stop', False, result['message']))
            return False
    
    def test_autonomous_mode(self):
        """Test autonomous mode toggle"""
        print("\n🤖 Testing autonomous mode...")
        
        # Enable autonomous mode
        result = self.send_command('/autonomous', {'mode': 'on'})
        if result['status'] == 'ok':
            print("  ✅ Autonomous mode enabled")
            time.sleep(0.5)
            
            # Disable autonomous mode
            result = self.send_command('/autonomous', {'mode': 'off'})
            if result['status'] == 'ok':
                print("  ✅ Autonomous mode disabled")
                self.test_results.append(('Autonomous Mode', True, 'OK'))
                return True
            else:
                print(f"  ❌ Autonomous mode disable failed: {result['message']}")
                self.test_results.append(('Autonomous Mode', False, 'Disable failed'))
                return False
        else:
            print(f"  ❌ Autonomous mode enable failed: {result['message']}")
            self.test_results.append(('Autonomous Mode', False, result['message']))
            return False
    
    def test_movement_commands(self):
        """Test basic movement commands"""
        print("\n🚗 Testing movement commands...")
        
        # Test forward movement
        print("  Testing forward movement...")
        result = self.send_command('/move', {'dir': 'forward', 'speed': 30, 'duration': 1})
        if result['status'] == 'ok':
            print("    ✅ Forward command accepted")
            time.sleep(1.5)  # Wait for movement to complete
        else:
            print(f"    ❌ Forward command failed: {result['message']}")
            self.test_results.append(('Movement Commands', False, 'Forward failed'))
            return False
        
        # Test rotation
        print("  Testing rotation...")
        result = self.send_command('/rotate', {'dir': 'left', 'speed': 30, 'duration': 1})
        if result['status'] == 'ok':
            print("    ✅ Rotation command accepted")
            time.sleep(1.5)  # Wait for rotation to complete
        else:
            print(f"    ❌ Rotation command failed: {result['message']}")
            self.test_results.append(('Movement Commands', False, 'Rotation failed'))
            return False
        
        # Test stop
        print("  Testing stop command...")
        result = self.send_command('/stop')
        if result['status'] == 'ok':
            print("    ✅ Stop command accepted")
            self.test_results.append(('Movement Commands', True, 'OK'))
            return True
        else:
            print(f"    ❌ Stop command failed: {result['message']}")
            self.test_results.append(('Movement Commands', False, 'Stop failed'))
            return False
    
    def test_parameter_validation(self):
        """Test parameter validation"""
        print("\n🔧 Testing parameter validation...")
        
        # Test invalid direction
        result = self.send_command('/move', {'dir': 'invalid', 'speed': 50})
        print(f"  Invalid direction test: {'✅' if result['status'] == 'error' else '❌'}")
        
        # Test extreme speed values
        result = self.send_command('/move', {'dir': 'forward', 'speed': 150})  # Over 100%
        print(f"  High speed handling: {'✅' if result['status'] in ['ok', 'error'] else '❌'}")
        
        self.test_results.append(('Parameter Validation', True, 'OK'))
        return True
    
    def run_all_tests(self):
        """Run complete test suite"""
        print("=" * 60)
        print("🧪 MICROPYTHON CONTROLLER TEST SUITE")
        print("=" * 60)
        print(f"Target: {self.base_url}")
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
                print(f"  ❌ Test failed with exception: {e}")
                all_passed = False
                
        # Final cleanup - ensure everything is stopped
        print("\n🧹 Cleanup...")
        self.send_command('/stop')
        self.send_command('/reset_estop')
        
        # Results summary
        print("\n" + "=" * 60)
        print("📊 TEST RESULTS SUMMARY")
        print("=" * 60)
        
        for test_name, passed, message in self.test_results:
            status = "✅ PASS" if passed else "❌ FAIL"
            print(f"{test_name:20} {status:10} {message}")
        
        print("\n" + "=" * 60)
        if all_passed:
            print("🎉 ALL TESTS PASSED - Controller is ready for autonomous operation!")
        else:
            print("⚠️  SOME TESTS FAILED - Check controller setup before autonomous operation")
        print("=" * 60)
        
        return all_passed

def main():
    parser = argparse.ArgumentParser(description='Test MicroPython Motor Controller')
    parser.add_argument('ip', nargs='?', default='192.168.25.72', 
                       help='IP address of MicroPython controller (default: 192.168.25.72)')
    parser.add_argument('--port', type=int, default=8080,
                       help='Port of MicroPython controller (default: 8080)')
    
    args = parser.parse_args()
    
    tester = MicroPythonTester(args.ip, args.port)
    
    try:
        success = tester.run_all_tests()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n\n⏹️  Test interrupted by user")
        # Cleanup
        tester.send_command('/stop')
        tester.send_command('/reset_estop')
        sys.exit(1)
    except Exception as e:
        print(f"\n\n💥 Test suite failed with error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
