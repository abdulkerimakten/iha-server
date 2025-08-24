import os
from dotenv import load_dotenv
from .mission_manager import MissionManager
from .utils.logger import setup_logger

def main():
    load_dotenv()
    log = setup_logger("iha-server")
    required = ["VEHICLE_CONN","YOLO_MODEL","IDA_HOST","IDA_PORT"]
    missing = [k for k in required if not os.getenv(k)]
    if missing:
        raise SystemExit(f"Missing env vars: {', '.join(missing)}")
    mgr = MissionManager(os.environ, log)
    mgr.run()

if __name__ == "__main__":
    main()