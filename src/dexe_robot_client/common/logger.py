# ----------------------------------------------------------------------------
# Copyright (c) 2021-2025 DexForce Technology Co., Ltd.
#
# All rights reserved.
# ----------------------------------------------------------------------------

import logging
import logging.handlers
import colorlog
import pathlib
import threading
from datetime import datetime
from pprint import pformat
from .config import LOG_DIR, PACKAGE_NAME

def get_logger() -> logging.Logger:
    return Logger.get_instance()


LEVEL_COLORS = {
    "DEBUG": "blue",
    "INFO": "green",
    "WARNING": "yellow",
    "ERROR": "red",
    "CRITICAL": "bold_red",
}


class Logger:
    class PrettyFilter(logging.Filter):
        def __init__(self,  
                     indent: int = 2, 
                     width: int = 100, 
                     sort_dicts: bool = False):
            super().__init__()
            self.indent = indent
            self.width = width
            self.sort_dicts = sort_dicts
            self._pretty_types = (dict, list, tuple, set)

        def _pp(self, obj):
            return pformat(obj, indent=self.indent, width=self.width, sort_dicts=self.sort_dicts)

        def filter(self, record: logging.LogRecord) -> bool:
            if isinstance(record.msg, self._pretty_types):
                record.msg = "\n" + self._pp(record.msg)
                record.args = ()

            return True

    class BaseMicroSecondFormatter:
        def formatTime(self, record, datefmt=None):
            dt = datetime.fromtimestamp(record.created)
            if datefmt:
                return dt.strftime(datefmt) + f",{dt.microsecond // 1000:03d}"
            else:
                return dt.strftime("%Y-%m-%d %H:%M:%S")

    class MicroSecondColoredFormatter(
        BaseMicroSecondFormatter, colorlog.ColoredFormatter
    ):
        pass

    class MicroSecondFormatter(BaseMicroSecondFormatter, logging.Formatter):
        pass

    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> logging.Logger:
        return cls(LOG_DIR / f"{PACKAGE_NAME}.log", PACKAGE_NAME)._logger

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, 
                 log_file_path="",
                 name="root",
                 console_level: int = logging.DEBUG,
                 file_level: int = logging.DEBUG,
                 pretty_in_console = True,
                 pretty_in_file = True):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True

        self._log_file_path = pathlib.Path(log_file_path)

        log_dir = self._log_file_path.parent
        if not log_dir.exists():
            log_dir.mkdir(parents=True, exist_ok=True)

        self._logger = logging.getLogger(name)
        self._logger.setLevel(logging.DEBUG)
        self._logger.propagate = False

        for handler in self._logger.handlers[:]:
            self._logger.removeHandler(handler)

        self.console_handler = logging.StreamHandler()
        self.console_handler.setLevel(console_level)
        self.console_handler.setFormatter(
            self.MicroSecondColoredFormatter(
                "[%(asctime)s] [%(log_color)s%(levelname)s%(light_white)s] [%(filename)s-%(lineno)d]: %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
                log_colors=LEVEL_COLORS,
            )
        )
        if pretty_in_console:
            self.console_handler.addFilter(
                self.PrettyFilter(indent=2, width=100, sort_dicts=False)
            )

        self.file_handler = logging.handlers.RotatingFileHandler(
            filename=self._log_file_path,
            maxBytes=104856700,
            backupCount=20,
            encoding="utf8",
        )
        self.file_handler.setLevel(file_level)
        self.file_handler.setFormatter(
            self.MicroSecondFormatter(
                "[%(asctime)s] [%(levelname)s] [%(filename)s-%(lineno)d]: %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
            )
        )
        if pretty_in_file:
            self.file_handler.addFilter(
                self.PrettyFilter(indent=2, width=100, sort_dicts=False)
            )

        self._logger.addHandler(self.console_handler)
        self._logger.addHandler(self.file_handler)