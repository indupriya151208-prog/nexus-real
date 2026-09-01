import streamlit as st
import math
import random

# Set up clean web layout page configurations
st.set_page_config(page_title="NEXUS Autonomous Simulation", layout="wide")

st.title("🚗 NEXUS: Real-Time Unstructured Road Navigation Demo")
st.markdown("Move the sliders in the sidebar to simulate chaotic traffic and watch the safety architecture adjust instantly.")

# ==========================================
# Core Autonomous AI Navigation Logic Kernel
# ==========================================
class NativeAutonomousAV:
    def __init__(self, cbf_gamma):
        self.time_step = 0.1
        self.max_steer = 0.5
        self.max_speed = 15.0
        self.cbf_gamma = cbf_gamma

    def process_dynamic_risk_field(self, x, y, speed, heading, obstacle):
        vx = speed * math.cos(heading)
        vy = speed * math.sin(heading)
        dx = obstacle["x"] - x
        dy = obstacle["y"] - y
        dvx = obstacle["vx"] - vx
        dvy = obstacle["vy"] - vy
        
        dot_product = dx * dvx + dy * dvy
        v_squared = dvx**2 + dvy**2
        
        if v_squared > 0.01 and dot_product < 0:
            time_to_collision = -dot_product / v_squared
            risk_modifier = 1.0 + (1.0 / (time_to_collision + 0.05))
        else:
            risk_modifier = 1.0
            
        return obstacle["radius"] * min(2.5, risk_modifier)

    def calculate_safety_actuation(self, vehicle_state, destination, obstacles):
        x, y, speed, heading = vehicle_state
        
        # FIX: Access destination elements cleanly as array indexes [0] and [1]
        target_angle = math.arctan2(destination[1] - y, destination[0] - x)
        nominal_steer = max(-self.max_steer, min(self.max_steer, target_angle - heading))
        
        best_speed, best_steer = self.max_speed, nominal_steer
        max_cbf_margin = -float('inf')
        
        candidate_steers = [nominal_steer, nominal_steer - 0.2, nominal_steer + 0.2, 0.0, -0.4, 0.4]
        candidate_speeds = [self.max_speed, self.max_speed * 0.5, 0.0]
        
        for v_cmd in candidate_speeds:
            for delta_cmd in candidate_steers:
                next_x = x + speed * math.cos(heading) * self.time_step
                next_y = y + speed * math.sin(heading) * self.time_step
                
                min_barrier_value = float('inf')
                for obs in obstacles:
                    dynamic_radius = self.process_dynamic_risk_field(next_x, next_y, v_cmd, heading, obs)
                    h = math.hypot(next_x - obs["x"], next_y - obs["y"])**2 - dynamic_radius**2
                    dh_dt = 2 * (next_x - obs["x"]) * (v_cmd * math.cos(heading) - obs["vx"]) + \
                            2 * (next_y - obs["y"]) * (v_cmd * math.sin(heading) - obs["vy"])
                            
                    cbf_condition = dh_dt + self.cbf_gamma * h
                    if cbf_condition < min_barrier_value:
                        min_barrier_value = cbf_condition
                        
                if min_barrier_value > max_cbf_margin:
                    max_cbf_margin = min_barrier_value
                    best_speed = v_cmd
                    best_steer = delta_cmd
                    
        if max_cbf_margin < -2.0:
            return 0.0, 0.0, "🔴 EMERGENCY OVERRIDE TRIGGERED"
        return best_speed, best_steer, "🟢 NOMINAL PASSIVE TRACKING"

# ==========================================
# Interactive Sidebar User Controls Matrix
# ==========================================
st.sidebar.header("🛠️ Environment Parameters")
gamma_slider = st.sidebar.slider("Control Barrier Aggressiveness (Gamma)", 0.1, 1.5, 0.6)

st.sidebar.subheader("🐖 Obstacle 1: Stray Livestock")
obs1_x = st.sidebar.slider("Livestock X Coordinate", 5.0, 30.0, 12.0)
obs1_y = st.sidebar.slider("Livestock Y Coordinate", -4.0, 4.0, 0.5)

st.sidebar.subheader("🛵 Obstacle 2: Wrong-Way Commuter")
obs2_x = st.sidebar.slider("Scooter X Coordinate", 5.0, 30.0, 24.0)
obs2_vx = st.sidebar.slider("Scooter Oncoming Speed (m/s)", -10.0, 0.0, -4.5)

# Initialize Simulation State Data
av_engine = NativeAutonomousAV(cbf_gamma=gamma_slider)
car_pose = [0.0, 0.0, 8.0, 0.0] 
target_goal = [35.0, 0.0] # Keeping layout as coordinate pair lists

active_obstacles = [
    {"x": obs1_x, "y": obs1_y, "radius": 1.2, "vx": -0.5, "vy": 0.0},
    {"x": obs2_x, "y": -1.0, "radius": 1.0, "vx": obs2_vx, "vy": 0.0},
    {"x": 18.0, "y": -2.5, "radius": 1.4, "vx": 0.0, "vy": 0.0} 
]

# Run Single Execution Tracking Frame Loop
final_speed, final_steer, system_status = av_engine.calculate_safety_actuation(car_pose, target_goal, active_obstacles)

# ==========================================
# Layout Display Presentation Windows
# ==========================================
col1, col2, col3 = st.columns(3)
col1.metric("Calculated Target Speed", f"{final_speed:.1f} m/s")
col2.metric("Steering Turn Angle", f"{final_steer:.3f} rad")
col3.metric("Safety Core Status", system_status)

st.subheader("🗺️ Live Virtual Drivable Corridor Vector Tracking Map")

chart_data = []
# Add vehicle position coordinates cleanly
chart_data.append({"X": car_pose[0], "Y": car_pose[1], "Type": "Autonomous Vehicle"})
# Add destination target goal coordinates cleanly
chart_data.append({"X": target_goal[0], "Y": target_goal[1], "Type": "Target Destination"})
# Add live obstacles positions
for i, obs in enumerate(active_obstacles):
    chart_data.append({"X": obs["x"], "Y": obs["y"], "Type": f"Obstacle {i+1}"})

st.scatter_chart(chart_data, x="X", y="Y", color="Type", use_container_width=True)
