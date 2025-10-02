from dataclasses import dataclass
from enum import IntEnum


class MouseButtonGtkId(IntEnum):
    LEFT_MOUSE_BUTTON = (
        1  # These numbers are assigned in GTK to the mouse buttons
    )
    MIDDLE_MOUSE_BUTTON = 2
    RIGHT_MOUSE_BUTTON = 3


@dataclass
class RgbColour:
    red: float
    green: float
    blue: float
