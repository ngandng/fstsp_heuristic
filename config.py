import yaml

class Config:
    _config = None  # Store config globally

    @classmethod
    def load_config(cls, file_path="config.yaml"):
        if cls._config is None:  # Load once
            with open(file_path, "r") as f:
                cls._config = yaml.safe_load(f)
        return cls._config

# Global access point
config = Config.load_config()
