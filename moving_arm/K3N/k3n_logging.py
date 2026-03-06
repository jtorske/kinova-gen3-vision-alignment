import logging
from datetime import datetime

def setup_logging(level=logging.INFO):
    """
    Call once at application startup.
    """

    logging.basicConfig(
        level=level,
        format="[%(asctime)s] [%(levelname)s] [%(name)s] %(message)s",
        datefmt="%Y-%m-%d %H:%M:%S",
    )