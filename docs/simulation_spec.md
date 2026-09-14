# Intersection Simulation & Scenario Specification
**Project:** Smart Traffic Flow Control Agent (OpenCV 2026 Competition)  
**Author:** Dominex (Simulation / Hardware Engineer)  
**Status:** Sprint 0 — Design Complete  

---

## 1. Intersection Layout & Virtual Sensors
- **Map:** CARLA `Town01` / `Town03` (4-Way Signalized Intersection)
- **Lanes Structure:**
  - **Inbound (ขาเข้า):** 3 Lanes per direction (Left/Thru, Thru, Right/U-Turn) — Detection zone 100 meters.
  - **Outbound (ขาออก):** 2 Lanes per direction.
- **Virtual CCTV Cameras:**
  - **4x Directional CCTV:** Height 7m, Pitch -30°, mounted above traffic signal poles on each arm (North, South, East, West) for per-lane queue & density tracking.
  - **1x Context Overhead Camera:** Height 25m, Pitch -90° (Bird's-Eye View) for global monitoring and demo video recording.

---

## 2. CARLA Environment Test Matrix
| Environment ID | Preset Name | CARLA Parameters | Target Vision Test |
| :--- | :--- | :--- | :--- |
| **ENV-01** | Clear Daylight | `SunAltitude=75`, `Cloudiness=10`, `Precipitation=0` | Baseline vehicle detection, speed & classification accuracy |
| **ENV-02** | Night Lights | `SunAltitude=-30`, `Streetlights=ON`, `VehicleLights=ON` | Low-light vision, glare reflection, headlight occlusion |
| **ENV-03** | Heavy Rain | `Cloudiness=90`, `Precipitation=80`, `Wetness=100` | Wet road surface reflections, rain lens artifacts |
| **ENV-04** | Dense Fog | `FogDensity=75`, `FogDistance=10` | Reduced visibility, long-range object occlusion |

---

## 3. Test Cases & Agent Scenarios
### TC-01: Off-Peak Flow (Low Volume)
- **Traffic Load:** 5–10 vehicles/min evenly distributed.
- **Goal:** Verify Agent minimizes idle green time and reduces overall average delay.

### TC-02: Peak Hour Asymmetric Congestion (High Volume)
- **Traffic Load:** 
  - North-South: 45–50 vehicles/min (Heavy queue)
  - East-West: 5 vehicles/min (Light traffic)
- **Goal:** Verify Agent detects >80% lane density on North-South and dynamically extends green window duration without exceeding East-West max wait threshold.

### TC-03: Emergency Vehicle Preemption
- **Traffic Load:** 25 vehicles/min.
- **Special Event:** Spawn `vehicle.ford.ambulance` (or sirens enabled) on East Inbound Lane 2.
- **Goal:** Verify Vision detects emergency vehicle type -> Agent triggers Immediate Green Wave -> ESP32 flashes emergency warning.

### TC-04: Incident & Obstruction Detection
- **Traffic Load:** 30 vehicles/min.
- **Special Event:** Stalled vehicle stopped in South Inbound Lane 2 during Green Phase for >20 seconds.
- **Goal:** Verify Anomaly Detection (stopped object in active lane) -> Agent redistributes signal timing -> Triggers AWS SNS Alert notification.

---

## 4. Edge-to-Cloud Data Payload Schema
Data format emitted from simulation stream / local processing to AWS IoT Core & Hardware Prototype:

```json
{
  "timestamp": 1773532800,
  "intersection_id": "INT-CARLA-TOWN01",
  "environment": {
    "weather": "CLEAR_NOON",
    "visibility_score": 0.98
  },
  "lanes_status": [
    {
      "lane_id": "NORTH_INBOUND_L2",
      "vehicle_count": 14,
      "occupancy_ratio": 0.82,
      "avg_speed_kmh": 4.2
    }
  ],
  "emergency_detected": false,
  "incident_detected": {
    "status": true,
    "lane_id": "SOUTH_INBOUND_L2",
    "stopped_duration_sec": 24
  }
}