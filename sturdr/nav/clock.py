import numpy as np
from dataclasses import dataclass

@dataclass(frozen=True)
class NavigationClock:
    """
    Dataclass of navigation clock Allan variance values
    """

    h0: float
    h1: float
    h2: float

LOW_QUALITY_TCXO = NavigationClock(h0=2e-19, h1=7e-21, h2=2e-20)
HIGH_QUALITY_TCXO = NavigationClock(h0=2e-21, h1=1e-22, h2=2e-20)
OCXO = NavigationClock(h0=2e-25, h1=7e-25, h2=6e-25)
RUBIDIUM = NavigationClock(h0=2e-22, h1=4.5e-26, h2=1e-30)
CESIUM = NavigationClock(h0=2e-22, h1=5e-27, h2=1.5e-33)

def GetNavClock(clock_name: str):
    """
    Factory function that retrieves requested clock Allan variance values

    Parameters
    ----------
    clock_name : str
        name of clock

    Returns
    -------
    NavigationClock
        clock Allan variance values
    """
    CLOCKS = {
        "lowqualitytcxo": LOW_QUALITY_TCXO,
        "highqualitytcxo": HIGH_QUALITY_TCXO,
        "ocxo": OCXO,
        "rubidium": RUBIDIUM,
        "cesium": CESIUM,
    }

    clock_name = "".join([i for i in clock_name if i.isalnum()]).casefold()
    clock = CLOCKS.get(clock_name.casefold(), CESIUM)  # defaults to cesium

    return clock