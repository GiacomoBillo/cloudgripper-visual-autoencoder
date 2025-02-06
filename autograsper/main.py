import logging
import sys

from coordinator import DataCollectionCoordinator
from custom_graspers.example_grasper import ExampleGrasper
from custom_graspers.manual_grasper import ManualGrasper
from custom_graspers.calibrate_grasper import CalibrateGrasper
from custom_graspers.greedy_grasper import GreedyGrasper

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    config_file = "autograsper/config.ini"
    # exampleGrasper = ManualGrasper(config_file)
    grasper = GreedyGrasper(config_file, max_num_samples=10)
    #grasper = CalibrateGrasper(config_file)
    coordinator = DataCollectionCoordinator(config_file, grasper)
    coordinator.run()


if __name__ == "__main__":
    sys.exit(main())
