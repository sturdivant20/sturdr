"""

SturDR
======

Daniel Sturdivant's GNSS Software Defined Radio

Contains the following modules:

1. `acquisition`
2. `beamsteer`
3. `discriminator`
4. `gnsssignal`
5. `lockdetectors`
6. `tracking`
7. `FftwWrapper`
8. `SturDR`

"""

from __future__ import annotations
from sturdr._sturdr_core import FftwWrapper
from sturdr._sturdr_core import SturDR
from sturdr._sturdr_core import acquisition
from sturdr._sturdr_core import beamsteer
from sturdr._sturdr_core import discriminator
from sturdr._sturdr_core import lockdetectors
from sturdr._sturdr_core import tracking
from . import _sturdr_core

__all__: list = [
    "__doc__",
    "__version__",
    "acquisition",
    "beamsteer",
    "discriminator",
    "lockdetectors",
    "tracking",
    "FftwWrapper",
    "SturDR",
]
__version__: str = "1.0.0"
