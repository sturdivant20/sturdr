"""

SturDR
======

Daniel Sturdivant's GNSS Software Defined Radio

Contains the following modules:

1. `acquisition`
2. `discriminator`
3. `gnsssignal`
4. `lockdetectors`
5. `tracking`
6. `BeamFormer`
7. `FftwWrapper`
8. `SturDR`

"""

from __future__ import annotations
from sturdr._sturdr_core import BeamFormer
from sturdr._sturdr_core import FftwWrapper
from sturdr._sturdr_core import SturDR
from sturdr._sturdr_core import acquisition
from sturdr._sturdr_core import discriminator
from sturdr._sturdr_core import gnsssignal
from sturdr._sturdr_core import lockdetectors
from sturdr._sturdr_core import tracking
from . import _sturdr_core

__all__: list = [
    "__doc__",
    "__version__",
    "acquisition",
    "discriminator",
    "gnsssignal",
    "lockdetectors",
    "tracking",
    "BeamFormer",
    "FftwWrapper",
    "SturDR",
]
__version__: str = "1.0.0"
