import logging


class Logger:
    _instance = None

    LOG_LEVELS = {
        'NOTSET': logging.NOTSET,
        'DEBUG': logging.DEBUG,
        'INFO': logging.INFO,
        'WARNING': logging.WARNING,
        'ERROR': logging.ERROR,
        'CRITICAL': logging.CRITICAL
    }

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(Logger, cls).__new__(cls)
            cls._instance._initialize_logger()
        return cls._instance
    
    def _create_console_handler(self, level: int) -> logging.Handler:
        """Creates and returns a console handler with the specified log level"""
        # Create console handler
        console_handler = logging.StreamHandler()
        console_handler.setLevel(level)

        # Create formatter and add to the handlers
        formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
        console_handler.setFormatter(formatter)

        return console_handler
    
    def _initialize_logger(self) -> None:
        """Initializes the base logger for the system"""
        self.logger = logging.getLogger("MyLogger")
        if not self.logger.hasHandlers():
            self.logger.setLevel(logging.DEBUG)

            # Add the handler to the logger
            console_handler = self._create_console_handler(logging.DEBUG)
            self.logger.addHandler(console_handler)

    def get_logger(self, name: str, level: str = 'DEBUG') -> logging.Logger:
        """
        Return a logger instance with a specific name and logging level.
        This method ensures that each logger gets a single handler.
        """
        level = self.LOG_LEVELS.get(level.upper(), logging.DEBUG)
        
        # Get a logger with a specific name
        logger = logging.getLogger(name)
        logger.setLevel(level)

        if not logger.hasHandlers():
            console_handler = self._create_console_handler(logging.DEBUG)

            logger.addHandler(console_handler)

        # Prevent the logger from propagating to the root logger
        logger.propagate = False

        return logger
    