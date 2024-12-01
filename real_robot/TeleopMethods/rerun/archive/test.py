import rerun as rr
from urdf_loader import URDFLogger
from common import CAMERA_NAMES, log_angle_rot, blueprint_row_images, extract_extrinsics, log_cartesian_velocity, POS_DIM_NAMES, link_to_world_transform, blueprint_raw

def main():
    rr.init("rerun_example_external_data_loader_urdf", recording_id="Test1")
    rr.spawn()
    # The most important part of this: log to standard output so the Rerun Viewer can ingest it!
    rr.stdout()

    filepath = "./URDF/test.urdf"
    urdf_logger = URDFLogger(filepath)
    urdf_logger.log()
    import time
    time.sleep(10)


if __name__ == "__main__":
    main()