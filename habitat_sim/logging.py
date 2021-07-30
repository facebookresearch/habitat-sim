r"""A simple Google-style logging wrapper.

Taken from https://github.com/benley/python-glog and adapted
"""

import logging
from logging import LogRecord

from habitat_sim._ext.habitat_sim_bindings.core import LoggingContext

DEBUG = logging.DEBUG
INFO = logging.INFO
WARNING = logging.WARNING
WARN = logging.WARN
ERROR = logging.ERROR
FATAL = logging.FATAL

logger = logging.getLogger(__file__)
handler = logging.StreamHandler()


logger.setLevel(ERROR if LoggingContext.current().sim_is_quiet else INFO)


def format_message(record: LogRecord) -> str:
    try:
        record_message = "%s" % (record.msg % record.args)
    except TypeError:
        record_message = record.msg
    return record_message


class HabitatSimFormatter(logging.Formatter):
    def format(self, record: LogRecord) -> str:
        record_message = "[Sim] %s" % (format_message(record),)
        record.getMessage = lambda: record_message  # type: ignore
        return logging.Formatter.format(self, record)


handler.setFormatter(HabitatSimFormatter())
logger.addHandler(handler)
