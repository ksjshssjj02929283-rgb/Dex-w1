'''
----------------------------------------------------------------------------
Copyright (c) 2021-2026 DexForce Technology Co., Ltd.

All rights reserved.
----------------------------------------------------------------------------
'''

import logging
import logging.handlers
import colorlog
import math
import pathlib
import threading
import time
from collections import OrderedDict
from datetime import datetime
from pprint import pformat
from typing import Hashable, Optional
from .config import LOG_DIR, PACKAGE_NAME


def get_logger() -> "ThrottledLogger":
    return DexeRobotClientLogger.get_instance()


LEVEL_COLORS = {
    "DEBUG": "blue",
    "INFO": "green",
    "WARNING": "yellow",
    "ERROR": "red",
    "CRITICAL": "bold_red",
}


class ThrottleFilter(logging.Filter):
    """Suppress repeated log records during a caller-defined time interval."""

    _INTERVAL_ATTR = "_dexe_throttle_interval"
    _KEY_ATTR = "_dexe_throttle_key"

    def __init__(self, max_keys: int = 4096):
        super().__init__()
        if max_keys <= 0:
            raise ValueError("max_keys must be greater than zero")
        self._max_keys = max_keys
        self._last_emitted = OrderedDict()
        self._lock = threading.Lock()

    def filter(self, record: logging.LogRecord) -> bool:
        interval = getattr(record, self._INTERVAL_ATTR, None)
        if interval is None or interval == 0:
            return True

        configured_key = getattr(record, self._KEY_ATTR, None)
        key = configured_key if configured_key is not None else (
            record.pathname,
            record.lineno,
            record.levelno,
            record.getMessage(),
        )
        now = time.monotonic()

        with self._lock:
            last_emitted = self._last_emitted.get(key)
            if last_emitted is not None and now - last_emitted < interval:
                return False

            self._last_emitted[key] = now
            self._last_emitted.move_to_end(key)
            if len(self._last_emitted) > self._max_keys:
                self._last_emitted.popitem(last=False)
        return True

    def clear(self):
        """Forget all throttle state, mainly useful when restarting a task."""
        with self._lock:
            self._last_emitted.clear()


class ThrottledLogger(logging.Logger):
    """A standard logger that accepts optional per-call throttling arguments.

    Example::

        logger.warning("connection lost", throttle_interval=5.0)
        logger.info("state changed", throttle_interval=1.0,
                    throttle_key="robot-state")
    """

    def _log(self,
             level,
             msg,
             args,
             exc_info=None,
             extra=None,
             stack_info=False,
             stacklevel=1,
             throttle_interval: Optional[float] = None,
             throttle_key: Optional[Hashable] = None):
        if throttle_interval is not None:
            if (isinstance(throttle_interval, bool) or
                    not isinstance(throttle_interval, (int, float)) or
                    not math.isfinite(throttle_interval) or
                    throttle_interval < 0):
                raise ValueError(
                    "throttle_interval must be a finite number greater than "
                    "or equal to zero")
            try:
                hash(throttle_key)
            except TypeError as exc:
                raise TypeError("throttle_key must be hashable") from exc

            extra = dict(extra) if extra is not None else {}
            extra[ThrottleFilter._INTERVAL_ATTR] = float(throttle_interval)
            extra[ThrottleFilter._KEY_ATTR] = throttle_key

        super()._log(
            level,
            msg,
            args,
            exc_info=exc_info,
            extra=extra,
            stack_info=stack_info,
            # Account for this wrapper so filename/lineno still point
            # at the application call site.
            stacklevel=stacklevel + 1)


class DexeRobotClientLogger:

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
            return pformat(obj,
                           indent=self.indent,
                           width=self.width,
                           sort_dicts=self.sort_dicts)

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

    class MicroSecondColoredFormatter(BaseMicroSecondFormatter,
                                      colorlog.ColoredFormatter):
        pass

    class MicroSecondFormatter(BaseMicroSecondFormatter, logging.Formatter):
        pass

    _instance = None
    _lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> ThrottledLogger:
        return cls(LOG_DIR / f"{PACKAGE_NAME}.log", PACKAGE_NAME)._logger

    @classmethod
    def set_console_level(cls, level: int):
        """
        动态设置 console handler 日志等级
        """
        logger = cls.get_instance()

        for h in logger.handlers:
            if isinstance(h, logging.StreamHandler):
                h.setLevel(level)

    def __new__(cls, *args, **kwargs):
        if not cls._instance:
            with cls._lock:
                if not cls._instance:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self,
                 log_file_path="",
                 name="root",
                 console_level: int = logging.INFO,
                 file_level: int = logging.DEBUG,
                 pretty_in_console=True,
                 pretty_in_file=True):
        if hasattr(self, "_initialized") and self._initialized:
            return
        self._initialized = True

        self._log_file_path = pathlib.Path(log_file_path)

        log_dir = self._log_file_path.parent
        if not log_dir.exists():
            log_dir.mkdir(parents=True, exist_ok=True)

        self._logger = logging.getLogger(name)
        # Keep logging.getLogger(name)'s registered singleton while extending
        # it with the optional keyword arguments implemented above.
        if not isinstance(self._logger, ThrottledLogger):
            self._logger.__class__ = ThrottledLogger
        self._logger.setLevel(logging.DEBUG)
        self._logger.propagate = False

        for handler in self._logger.handlers[:]:
            self._logger.removeHandler(handler)
        self.throttle_filter = ThrottleFilter()
        self._logger.addFilter(self.throttle_filter)

        self.console_handler = logging.StreamHandler()
        self.console_handler.setLevel(console_level)
        self.console_handler.setFormatter(
            self.MicroSecondColoredFormatter(
                "[%(asctime)s] [%(log_color)s%(levelname)s%(light_white)s] [%(filename)s-%(lineno)d]: %(message)s",
                datefmt="%Y-%m-%d %H:%M:%S",
                log_colors=LEVEL_COLORS,
            ))
        if pretty_in_console:
            self.console_handler.addFilter(
                self.PrettyFilter(indent=2, width=100, sort_dicts=False))

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
            ))
        if pretty_in_file:
            self.file_handler.addFilter(
                self.PrettyFilter(indent=2, width=100, sort_dicts=False))

        self._logger.addHandler(self.console_handler)
        self._logger.addHandler(self.file_handler)
