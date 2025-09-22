from abc import ABC, abstractmethod
from typing import Dict, Any

class VisualizationBase(ABC):
    @abstractmethod
    def initialize(self, aircraft_params: Dict[str, Any]) -> None:
        pass
    
    @abstractmethod
    def update(self, state_vector: Dict[str, float], time: float) -> None:
        pass
    
    @abstractmethod
    def cleanup(self) -> None:
        pass