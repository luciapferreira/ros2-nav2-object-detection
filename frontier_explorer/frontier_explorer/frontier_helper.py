from collections import deque
from enum import Enum
from typing import List, Tuple
import numpy as np

from .occupancy_grid import OccupancyGrid2d

class FrontierHelper:
    class PointClassification(Enum):
        MapOpen = 1
        MapClosed = 2
        FrontierOpen = 4
        FrontierClosed = 8

    class FrontierPoint:
        def __init__(self, x: int, y: int):
            self.mapX = x
            self.mapY = y
            self.classification = 0

    class FrontierCache:
        def __init__(self):
            self.cache = {}

        def get_point(self, x: int, y: int):
            key = (x, y)
            if key not in self.cache:
                self.cache[key] = FrontierHelper.FrontierPoint(x, y)
            return self.cache[key]

        def clear(self):
            self.cache.clear()

    # ---------------------------
    # Core helper functions
    # ---------------------------
    @staticmethod
    def get_neighbors(pt, grid: OccupancyGrid2d, cache):
        for dx in range(-1, 2):
            for dy in range(-1, 2):
                nx, ny = pt.mapX + dx, pt.mapY + dy
                if 0 <= nx < grid.size_x() and 0 <= ny < grid.size_y():
                    yield cache.get_point(nx, ny)

    @staticmethod
    def is_frontier(pt, grid: OccupancyGrid2d, cache, occ_threshold: int):
        if grid.get_cost(pt.mapX, pt.mapY) != OccupancyGrid2d.CostValues.NoInformation.value:
            return False
        has_free = False
        for n in FrontierHelper.get_neighbors(pt, grid, cache):
            cost = grid.get_cost(n.mapX, n.mapY)
            if cost > occ_threshold:
                return False
            if cost == OccupancyGrid2d.CostValues.FreeSpace.value:
                has_free = True
        return has_free

    @staticmethod
    def find_free(mx: int, my: int, grid: OccupancyGrid2d, cache):
        bfs = deque([cache.get_point(mx, my)])
        while bfs:
            loc = bfs.popleft()
            if grid.get_cost(loc.mapX, loc.mapY) == 0:
                return loc
            for n in FrontierHelper.get_neighbors(loc, grid, cache):
                if n.classification == 0:
                    n.classification = 1
                    bfs.append(n)
        return cache.get_point(mx, my)

    @staticmethod
    def centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
        arr = np.array(points)
        return float(arr[:, 0].mean()), float(arr[:, 1].mean())

    @staticmethod
    def detect_frontiers(pose, grid: OccupancyGrid2d, min_frontier_size: int, occ_threshold: int):
        cache = FrontierHelper.FrontierCache()
        mx, my = grid.world_to_map(pose.position.x, pose.position.y)
        free_point = FrontierHelper.find_free(mx, my, grid, cache)
        free_point.classification = FrontierHelper.PointClassification.MapOpen.value
        map_queue = deque([free_point])
        frontiers = []

        while map_queue:
            p = map_queue.popleft()
            if p.classification & FrontierHelper.PointClassification.MapClosed.value:
                continue

            if FrontierHelper.is_frontier(p, grid, cache, occ_threshold):
                p.classification |= FrontierHelper.PointClassification.FrontierOpen.value
                frontier_queue = deque([p])
                new_frontier = []

                while frontier_queue:
                    q = frontier_queue.popleft()
                    if q.classification & (FrontierHelper.PointClassification.MapClosed.value |
                                           FrontierHelper.PointClassification.FrontierClosed.value):
                        continue
                    if FrontierHelper.is_frontier(q, grid, cache, occ_threshold):
                        new_frontier.append(q)
                        for w in FrontierHelper.get_neighbors(q, grid, cache):
                            if not w.classification:
                                w.classification = FrontierHelper.PointClassification.FrontierOpen.value
                                frontier_queue.append(w)
                    q.classification |= FrontierHelper.PointClassification.FrontierClosed.value

                if len(new_frontier) >= min_frontier_size:
                    points_world = [grid.map_to_world(f.mapX, f.mapY) for f in new_frontier]
                    frontiers.append(FrontierHelper.centroid(points_world))

            for v in FrontierHelper.get_neighbors(p, grid, cache):
                if not v.classification:
                    v.classification = FrontierHelper.PointClassification.MapOpen.value
                    map_queue.append(v)

            p.classification |= FrontierHelper.PointClassification.MapClosed.value

        return frontiers
