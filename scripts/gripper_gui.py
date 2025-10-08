#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import tkinter as tk
from tkinter import ttk
import threading

class GripperControlGUI(Node):
    def __init__(self):
        super().__init__('gripper_control_gui')
        
        # Publisher for angle control
        self.angle_pub = self.create_publisher(Int32, 'gripper_angle', 10)
        
        # Create GUI
        self.create_gui()
        
    def create_gui(self):
        self.root = tk.Tk()
        self.root.title("Gripper Controller - Tuning Mode")
        self.root.geometry("500x300")
        
        # Main frame
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Gripper Angle Tuning", font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Angle control section
        angle_frame = ttk.LabelFrame(main_frame, text="Angle Control (0-180°)", padding="10")
        angle_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        self.angle_var = tk.IntVar(value=90)
        self.angle_scale = ttk.Scale(angle_frame, from_=0, to=180, orient=tk.HORIZONTAL, 
                                    variable=self.angle_var, length=400,
                                    command=self.on_angle_change)
        self.angle_scale.grid(row=0, column=0, columnspan=5, pady=5)
        
        self.angle_label = ttk.Label(angle_frame, text="Angle: 90°", font=("Arial", 12))
        self.angle_label.grid(row=1, column=2, pady=5)
        
        # Quick angle buttons
        ttk.Button(angle_frame, text="0°", command=lambda: self.set_angle(0)).grid(row=2, column=0, padx=5)
        ttk.Button(angle_frame, text="45°", command=lambda: self.set_angle(45)).grid(row=2, column=1, padx=5)
        ttk.Button(angle_frame, text="90°", command=lambda: self.set_angle(90)).grid(row=2, column=2, padx=5)
        ttk.Button(angle_frame, text="135°", command=lambda: self.set_angle(135)).grid(row=2, column=3, padx=5)
        ttk.Button(angle_frame, text="180°", command=lambda: self.set_angle(180)).grid(row=2, column=4, padx=5)
        
        # Gripper control section
        gripper_frame = ttk.LabelFrame(main_frame, text="Gripper Positions", padding="10")
        gripper_frame.grid(row=2, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        ttk.Button(gripper_frame, text="Close Position (3°)", 
                  command=lambda: self.set_angle(3)).grid(row=0, column=0, padx=10)
        ttk.Button(gripper_frame, text="Open Position (38°)", 
                  command=lambda: self.set_angle(38)).grid(row=0, column=1, padx=10)
        
        # Keyboard control info
        info_frame = ttk.LabelFrame(main_frame, text="Keyboard Controls", padding="10")
        info_frame.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=10)
        
        info_text = """Arrow Keys: Fine adjustment (±1°)
Page Up/Down: Coarse adjustment (±10°)
Space: Reset to center (90°)"""
        ttk.Label(info_frame, text=info_text, justify=tk.LEFT).grid(row=0, column=0)
        
        # Bind keyboard events
        self.root.bind('<Key>', self.on_key_press)
        self.root.focus_set()
        
        # Configure grid weights
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        
    def on_angle_change(self, value):
        angle = int(float(value))
        self.angle_label.config(text=f"Angle: {angle}°")
        self.publish_angle(angle)
        
    def set_angle(self, angle):
        self.angle_var.set(angle)
        self.angle_label.config(text=f"Angle: {angle}°")
        self.publish_angle(angle)
        
    def publish_angle(self, angle):
        msg = Int32()
        msg.data = angle
        self.angle_pub.publish(msg)
        self.get_logger().info(f'Published angle: {angle}°')
        
    def on_key_press(self, event):
        if event.keysym == 'Up':
            self.set_angle(min(180, self.angle_var.get() + 1))
        elif event.keysym == 'Down':
            self.set_angle(max(0, self.angle_var.get() - 1))
        elif event.keysym == 'Prior':  # Page Up
            self.set_angle(min(180, self.angle_var.get() + 10))
        elif event.keysym == 'Next':   # Page Down
            self.set_angle(max(0, self.angle_var.get() - 10))
        elif event.keysym == 'space':
            self.set_angle(90)
            
    def run(self):
        # Start ROS spinning in a separate thread
        self.ros_thread = threading.Thread(target=rclpy.spin, args=(self,))
        self.ros_thread.daemon = True
        self.ros_thread.start()
        
        # Start GUI main loop
        self.root.mainloop()

def main():
    rclpy.init()
    
    try:
        gui = GripperControlGUI()
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()