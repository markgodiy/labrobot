#!/bin/bash
"""
Robot Monitoring Dashboard - Terminal Based
==========================================

Simple terminal-based monitoring for robot perception and decisions.
Run these commands in separate terminals to monitor different aspects.
"""

echo "=== Robot Monitoring Commands ==="
echo ""
echo "🤖 Navigation State & Decisions:"
echo "  ros2 topic echo /navigation/state"
echo ""
echo "🧠 Debug Info (Obstacles, Distances, Actions):"
echo "  ros2 topic echo /navigation/debug"
echo ""
echo "📊 Motor Controller Status:"
echo "  ros2 topic echo /motor_controller/status"
echo ""
echo "👀 LIDAR Data Summary (distances):"
echo "  ros2 topic echo /scan --field ranges"
echo ""
echo "📷 Camera Topics Available:"
echo "  ros2 topic list | grep oak"
echo ""
echo "🔧 Quick Status Check:"
echo "  ros2 topic hz /scan"
echo "  ros2 topic hz /oak/rgb/image_raw"
echo "  ros2 topic hz /oak/depth/image_raw"
echo ""
echo "🛑 Emergency Controls:"
echo "  ros2 service call /emergency_stop std_srvs/srv/Trigger"
echo "  ros2 service call /reset_emergency_stop std_srvs/srv/Trigger"
echo ""
echo "Run any of these commands in separate terminals to monitor the robot!"
