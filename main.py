"""
main.py
Drone_Onboard_AV entrypoint.
Initializes communication, sensors, and autonomy loop.
"""

import asyncio
import logging
from core import comms, sensors, control, state_manager, model_manager, utils
from modules.perception.object_detection import ObjectDetector
from modules.navigation.local_planner import LocalPlanner
from modules.navigation.collision_avoidance import CollisionAvoidance
from modules.safety.failsafe import FailSafe
from modules.mission.mission_executor import MissionExecutor
import config

async def main():
    # --- Setup Logging ---
    logging.basicConfig(level=config.LOGGING["level"])
    logger = logging.getLogger("Drone_Onboard_AV")
    logger.info("Initializing Drone_Onboard_AV System...")

    # --- Initialize Core Managers ---
    comms_mgr = comms.CommunicationManager(config.COMMS)
    sensor_mgr = sensors.SensorManager(config.SENSORS)
    control_mgr = control.ControlManager(comms_mgr)
    state_mgr = state_manager.StateManager()
    model_mgr = model_manager.ModelManager(config.MODELS)

    # --- Initialize Modules ---
    detector = ObjectDetector(model_mgr)
    planner = LocalPlanner()
    avoidance = CollisionAvoidance()
    failsafe = FailSafe(control_mgr, state_mgr)
    mission_exec = MissionExecutor(control_mgr, planner)

    # --- Start Communication ---
    await comms_mgr.connect()
    logger.info("Communication established.")

    # --- Main Control Loop ---
    while True:
        # 1. Read sensor data
        sensor_data = await sensor_mgr.read_all()
        gps_data, imu_data, cam_frame = sensor_data.get("gps"), sensor_data.get("imu"), sensor_data.get("camera")

        # 2. Update state
        state_mgr.update(gps_data, imu_data)

        # 3. Run perception (object detection)
        if cam_frame is not None:
            detections = detector.infer(cam_frame)
            state_mgr.set_objects(detections)

        # 4. Run local planner
        control_targets = planner.plan(state_mgr)

        # 5. Collision avoidance correction
        safe_controls = avoidance.avoid(state_mgr, control_targets)

        # 6. Send commands to FCU
        control_mgr.execute(safe_controls)

        # 7. Failsafe monitoring
        failsafe.check(state_mgr)

        # 8. Stream telemetry to ground
        await comms_mgr.send_telemetry(state_mgr)

        # 9. Mission control
        mission_exec.update(state_mgr)

        await asyncio.sleep(1.0 / 10)  # 10 Hz loop rate

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logging.warning("Shutting down Drone_Onboard_AV safely...")
