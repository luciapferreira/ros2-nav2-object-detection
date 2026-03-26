from enum import Enum
from typing import Tuple
from nav_msgs.msg import OccupancyGrid

class OccupancyGrid2d:
    class CostValues(Enum):
        FreeSpace = 0
        LethalObstacle = 100
        NoInformation = -1

    def __init__(self, map_msg: OccupancyGrid):
        self.map = map_msg

    def get_cost(self, mx: int, my: int) -> int:
        return self.map.data[my * self.map.info.width + mx]

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        wx = self.map.info.origin.position.x + (mx + 0.5) * self.map.info.resolution
        wy = self.map.info.origin.position.y + (my + 0.5) * self.map.info.resolution
        return wx, wy

    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        ox, oy = self.map.info.origin.position.x, self.map.info.origin.position.y
        if wx < ox or wy < oy:
            raise ValueError("World coordinates out of bounds")
        mx = int((wx - ox) / self.map.info.resolution)
        my = int((wy - oy) / self.map.info.resolution)
        if mx >= self.map.info.width or my >= self.map.info.height:
            raise ValueError("World coordinates out of bounds")
        return mx, my

    def size_x(self) -> int:
        return self.map.info.width

    def size_y(self) -> int:
        return self.map.info.height
