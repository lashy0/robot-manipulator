import logging
import logging.config


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
        """Creates and returns a console handler with the specified log level.
        
        Args
        ----
        level : int
            The logging level for the handler.
        
        Returns
        -------
        handle : logging.Handle
            Configured console handler.
        """
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
        """Return a logger instance with a specific name and logging level.
        This method ensures that each logger gets a single handler.

        Args
        ----
        name : str
            The name of the logger.
        
        level : str, optional
            The logging level.
        
        Returns
        -------
        logger : logging.Logger
            Configured logger instance.
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
    
    @classmethod
    def configure_from_file(config_path: str) -> None:
        """Configures the logging system from a given JSON configuration file.

        Args
        ----
        config_path : str
            Path to the logging configuration file.
        
        Raises
        ------
        ValueError:
            If the file format is unsupported.
        """
        with open(config_path, 'r') as file:
            if config_path.endswith('.json'):
                import json

                config = json.load(file)
                logging.config.dictConfig(config)
            else:
                raise ValueError(
                    "Unsupported configuration file format. Use JSON"
                )

    @classmethod
    def configure_logger_for_module(cls, module_name: str, level: str = 'DEBUG') -> logging.Logger:
        """Configure a logger for a specific module with a given log level.
        
        Args
        ----
        module_name : str
            The name of the module to configure the logger for.
        
        level : str, optional
            The logging level.
        
        Returns
        -------
        logger : logging.Logger
            Configured logger instance for the module.
        """
        instance = cls()
        logger = logging.getLogger(module_name)
        log_level = cls.LOG_LEVELS.get(level.upper(), logging.DEBUG)
        logger.setLevel(log_level)

        if not logger.hasHandlers():
            console_handle = instance._create_console_handler(log_level)
            logger.addHandler(console_handle)
        
        logger.propagate = False
        return logger
