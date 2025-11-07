#!/usr/bin/env python3
"""
dashboard_ui.py
-------------------------------------------------
MAYURI Command & Visualization Dashboard

Provides a web-based control center for:
- Drone telemetry (mode, altitude, speed, battery)
- AI threat detections
- Mission map visualization
- Command buttons (arm, takeoff, land, RTL, emergency hold)
- Event log and status panel

==> Live updates from ROS2 topics
==> Control via web UI
==> Works with simulation or real hardware
==> Modular layout (easy to extend)

Author: Shree Praveen(@p_rav_ee_n1082)
"""

import json
import threading
import time
import os

import dash
from dash import html, dcc, Input, Output, State
import plotly.graph_objects as go
import dash_bootstrap_components as dbc

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# -------------------------------------------------------------
# ROS2 Node for UI communication
# -------------------------------------------------------------
class DashboardNode(Node):
    def __init__(self):
        super().__init__("dashboard_ui_node")
        self.flight_status = {}
        self.ai_detections = []
        self.last_event = ""
        self.cmd_pub = self.create_publisher(String, "/mayuri/flight/cmd", 10)

        self.create_subscription(String, "/mayuri/flight/status", self._on_status, 10)
        self.create_subscription(String, "/mayuri/ai/detections", self._on_ai, 10)
        self.create_subscription(String, "/mayuri/failsafe/event", self._on_event, 10)

        self.get_logger().info("<●> DashboardNode connected to ROS2 topics.")

    def _on_status(self, msg: String):
        try:
            self.flight_status = json.loads(msg.data)
        except Exception:
            pass

    def _on_ai(self, msg: String):
        try:
            data = json.loads(msg.data)
            self.ai_detections = data.get("detections", [])
        except Exception:
            pass

    def _on_event(self, msg: String):
        try:
            event = json.loads(msg.data)
            self.last_event = f"{event.get('reason', 'Unknown')}: {event.get('action', '')}"
        except Exception:
            pass

    def send_command(self, command: dict):
        msg = String()
        msg.data = json.dumps(command)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"📤 Sent command: {command}")


# -------------------------------------------------------------
# Initialize ROS2 in background thread
# -------------------------------------------------------------
node: DashboardNode = None


def start_ros():
    global node
    rclpy.init()
    node = DashboardNode()
    rclpy.spin(node)


ros_thread = threading.Thread(target=start_ros, daemon=True)
ros_thread.start()


# -------------------------------------------------------------
# DASH UI SETUP
# -------------------------------------------------------------
app = dash.Dash(__name__, external_stylesheets=[dbc.themes.SOLAR])

app.title = "MAYURI Control Dashboard"

# --------------------------- Layout ---------------------------
app.layout = dbc.Container(
    [
        html.H2("🧠 MAYURI – AI-Driven Surveillance & Defense Drone", className="text-center mt-3 mb-4"),

        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardHeader("🛰️ Drone Status"),
                    dbc.CardBody([
                        html.Div(id="status-text"),
                        html.Br(),
                        html.Div(id="event-text", className="text-warning"),
                    ])
                ])
            ], width=4),

            dbc.Col([
                dbc.Card([
                    dbc.CardHeader("🗺️ Map View"),
                    dbc.CardBody([
                        html.Iframe(
                            id="map-frame",
                            src="/assets/live_map.html",
                            style={"width": "100%", "height": "400px", "border": "2px solid #444"}
                        )
                    ])
                ])
            ], width=8),
        ]),

        html.Br(),

        dbc.Row([
            dbc.Col([
                dbc.Card([
                    dbc.CardHeader("🎯 AI Detections"),
                    dbc.CardBody([
                        dcc.Graph(id="ai-detection-graph", config={"displayModeBar": False}),
                    ])
                ])
            ], width=6),

            dbc.Col([
                dbc.Card([
                    dbc.CardHeader("🎮 Controls"),
                    dbc.CardBody([
                        dbc.ButtonGroup([
                            dbc.Button("Arm", id="btn-arm", color="success", className="me-1"),
                            dbc.Button("Disarm", id="btn-disarm", color="danger", className="me-1"),
                            dbc.Button("Takeoff", id="btn-takeoff", color="primary", className="me-1"),
                            dbc.Button("Land", id="btn-land", color="secondary", className="me-1"),
                            dbc.Button("Hold", id="btn-hold", color="warning", className="me-1"),
                        ]),
                    ])
                ])
            ], width=6),
        ]),

        dcc.Interval(id="update-interval", interval=1000, n_intervals=0),
    ],
    fluid=True,
)


# -------------------------------------------------------------
# DASH CALLBACKS
# -------------------------------------------------------------
@app.callback(
    Output("status-text", "children"),
    Output("event-text", "children"),
    Input("update-interval", "n_intervals"),
)
def update_status(_):
    """Live update drone telemetry and events."""
    if not node or not node.flight_status:
        return "⏳ Waiting for telemetry...", ""

    pos = node.flight_status.get("position", {})
    armed = node.flight_status.get("armed", False)
    mode = node.flight_status.get("mode", "UNKNOWN")
    alt = pos.get("alt", 0)
    lat, lon = pos.get("lat", 0), pos.get("lon", 0)

    status = [
        html.P(f"Mode: {mode}"),
        html.P(f"Armed: {'✅' if armed else '❌'}"),
        html.P(f"Altitude: {alt:.1f} m"),
        html.P(f"Lat: {lat:.5f}, Lon: {lon:.5f}"),
        html.P(f"Battery: {node.flight_status.get('battery', {}).get('battery_remaining', '--')}%"),
    ]
    return status, f"⚠️ Last Event: {node.last_event}"


@app.callback(
    Output("ai-detection-graph", "figure"),
    Input("update-interval", "n_intervals"),
)
def update_ai_detections(_):
    """Visualize detections from AI core."""
    detections = node.ai_detections if node else []
    if not detections:
        fig = go.Figure()
        fig.update_layout(title="No detections yet", template="plotly_dark")
        return fig

    labels = [d["label"] for d in detections]
    conf = [d["confidence"] for d in detections]
    fig = go.Figure(go.Bar(x=labels, y=conf, marker_color="red"))
    fig.update_layout(
        title="Active AI Detections",
        xaxis_title="Object Type",
        yaxis_title="Confidence",
        template="plotly_dark"
    )
    return fig


# -------------------------- Command Buttons ---------------------------
@app.callback(
    Output("btn-arm", "n_clicks"),
    Input("btn-arm", "n_clicks"),
    prevent_initial_call=True,
)
def arm(_):
    node.send_command({"cmd": "arm", "confirm": True})
    return None


@app.callback(
    Output("btn-disarm", "n_clicks"),
    Input("btn-disarm", "n_clicks"),
    prevent_initial_call=True,
)
def disarm(_):
    node.send_command({"cmd": "disarm", "confirm": True})
    return None


@app.callback(
    Output("btn-takeoff", "n_clicks"),
    Input("btn-takeoff", "n_clicks"),
    prevent_initial_call=True,
)
def takeoff(_):
    node.send_command({"cmd": "takeoff", "alt": 10, "confirm": True})
    return None


@app.callback(
    Output("btn-land", "n_clicks"),
    Input("btn-land", "n_clicks"),
    prevent_initial_call=True,
)
def land(_):
    node.send_command({"cmd": "land", "confirm": True})
    return None


@app.callback(
    Output("btn-hold", "n_clicks"),
    Input("btn-hold", "n_clicks"),
    prevent_initial_call=True,
)
def hold(_):
    node.send_command({"cmd": "set_mode", "mode": "HOLD", "confirm": True})
    return None


# -------------------------------------------------------------
# MAIN ENTRY
# -------------------------------------------------------------
if __name__ == "__main__":
    os.makedirs("mayuri/src/ui_control/assets", exist_ok=True)
    app.run_server(host="0.0.0.0", port=8050, debug=True)
