from logging import getLogger
from commander.location import Location

logger = getLogger("ComputerVisionModule")

class ComputerVisionModule:

    def detectTag(self) -> Location:
        logger.info("Detecting tag...")
        return Location(0, 0, 0)

    def scanAprilTag(self, location: Location):
        logger.info(f"Scanning AprilTag at {location}")
        
    def storeLocationData(self) -> Location:
        logger.info("Storing location data")
        return Location(0, 0, 0)

    def saveCurrentPose(self) -> Location:
        logger.info("Saving pose")
        return Location(0, 0, 0)

    def calibrateCamera(self):
        logger.info("Calibrating camera...")