#!/usr/bin/env python3

import rclpy
import math
import cv2
import numpy as np
from typing import Union
from std_msgs.msg import Header
from nav_msgs.msg import GridCells, OccupancyGrid, Path
from geometry_msgs.msg import Point, Quaternion, Pose, PoseStamped
from priority_queue import PriorityQueue
import tf_transformations


DIRECTIONS_OF_4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
DIRECTIONS_OF_8 = [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]


class PathPlanner:

    @staticmethod
    def grid_to_index(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the index corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The index.
        """
        return p[1] * mapdata.info.width + p[0]

    @staticmethod
    def get_cell_value(mapdata: OccupancyGrid, p: "tuple[int, int]") -> int:
        """
        Returns the cell corresponding to the given (x,y) coordinates in the occupancy grid.
        :param p [(int, int)] The cell coordinate.
        :return  [int] The cell.
        """
        return mapdata.data[PathPlanner.grid_to_index(mapdata, p)]

    @staticmethod
    def euclidean_distance(
        p1: "tuple[float, float]", p2: "tuple[float, float]"
    ) -> float:
        """
        Calculates the Euclidean distance between two points.
        :param p1 [(float, float)] first point.
        :param p2 [(float, float)] second point.
        :return   [float]          distance.
        """
        return math.sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

    @staticmethod
    def grid_to_world(mapdata: OccupancyGrid, p: "tuple[int, int]") -> Point:
        """
        Transforms a cell coordinate in the occupancy grid into a world coordinate.
        :param mapdata [OccupancyGrid] The map information.
        :param p [(int, int)] The cell coordinate.
        :return        [Point]         The position in the world.
        """
        x = (p[0] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.x
        y = (p[1] + 0.5) * mapdata.info.resolution + mapdata.info.origin.position.y
        return Point(x=x, y=y, z=0.0)

    @staticmethod
    def world_to_grid(mapdata: OccupancyGrid, wp: Point) -> "tuple[int, int]":
        """
        Transforms a world coordinate into a cell coordinate in the occupancy grid.
        :param mapdata [OccupancyGrid] The map information.
        :param wp      [Point]         The world coordinate.
        :return        [(int,int)]     The cell position as a tuple.
        """
        x = int((wp.x - mapdata.info.origin.position.x) / mapdata.info.resolution)
        y = int((wp.y - mapdata.info.origin.position.y) / mapdata.info.resolution)
        return (x, y)

    @staticmethod
    def path_to_poses(
        mapdata: OccupancyGrid, path: "list[tuple[int, int]]"
    ) -> "list[PoseStamped]":
        """
        Converts the given path into a list of PoseStamped.
        :param mapdata [OccupancyGrid] The map information.
        :param  path   [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [[PoseStamped]] The path as a list of PoseStamped (world coordinates).
        """
        poses = []
        for i in range(len(path) - 1):
            cell = path[i]
            next_cell = path[i + 1]
            if i != len(path) - 1:
                angle_to_next = math.atan2(
                    next_cell[1] - cell[1], next_cell[0] - cell[0]
                )
            q = tf_transformations.quaternion_from_euler(0, 0, angle_to_next)
            poses.append(
                PoseStamped(
                    header=Header(frame_id="map"),
                    pose=Pose(
                        position=PathPlanner.grid_to_world(mapdata, cell),
                        orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3]),
                    ),
                )
            )
        return poses

    @staticmethod
    def is_cell_in_bounds(mapdata: OccupancyGrid, p: "tuple[int, int]") -> bool:
        width = mapdata.info.width
        height = mapdata.info.height
        x = p[0]
        y = p[1]

        if x < 0 or x >= width:
            return False
        if y < 0 or y >= height:
            return False
        return True

    @staticmethod
    def is_cell_walkable(mapdata: OccupancyGrid, p: "tuple[int, int]") -> bool:
        """
        A cell is walkable if all of these conditions are true:
        1. It is within the boundaries of the grid;
        2. It is free (not occupied by an obstacle)
        :param mapdata [OccupancyGrid] The map information.
        :param p       [(int, int)]    The coordinate in the grid.
        :return        [bool]          True if the cell is walkable, False otherwise
        """
        if not PathPlanner.is_cell_in_bounds(mapdata, p):
            return False

        WALKABLE_THRESHOLD = 50
        return PathPlanner.get_cell_value(mapdata, p) < WALKABLE_THRESHOLD

    @staticmethod
    def neighbors(
        mapdata: OccupancyGrid,
        p: "tuple[int, int]",
        directions: "list[tuple[int, int]]",
        must_be_walkable: bool = True,
    ) -> "list[tuple[int, int]]":
        """
        Returns the neighbors cells of (x,y) in the occupancy grid given directions to check.
        :param mapdata           [OccupancyGrid] The map information.
        :param p                 [(int, int)]    The coordinate in the grid.
        :param directions        [[(int,int)]]   A list of directions to check for neighbors.
        :param must_be_walkable  [bool]          Whether or not the cells must be walkable
        :return                  [[(int,int)]]   A list of 4-neighbors.
        """
        neighbors = []
        for direction in directions:
            candidate = (p[0] + direction[0], p[1] + direction[1])
            if (
                must_be_walkable and PathPlanner.is_cell_walkable(mapdata, candidate)
            ) or (
                not must_be_walkable
                and PathPlanner.is_cell_in_bounds(mapdata, candidate)
            ):
                neighbors.append(candidate)
        return neighbors

    @staticmethod
    def neighbors_of_4(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[int, int]]":
        return PathPlanner.neighbors(mapdata, p, DIRECTIONS_OF_4, must_be_walkable)

    @staticmethod
    def neighbors_of_8(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[int, int]]":
        return PathPlanner.neighbors(mapdata, p, DIRECTIONS_OF_8, must_be_walkable)

    @staticmethod
    def neighbors_and_distances(
        mapdata: OccupancyGrid,
        p: "tuple[int, int]",
        directions: "list[tuple[int, int]]",
        must_be_walkable: bool = True,
    ) -> "list[tuple[tuple[int, int], float]]":
        """
        Returns the neighbors cells of (x,y) in the occupancy grid given directions to check and their distances.
        :param mapdata           [OccupancyGrid] The map information.
        :param p                 [(int, int)]    The coordinate in the grid.
        :param directions        [[(int,int)]]   A list of directions to check for neighbors.
        :param must_be_walkable  [bool]          Whether or not the cells must be walkable
        :return                  [[(int,int)]]   A list of 4-neighbors.
        """
        neighbors = []
        for direction in directions:
            candidate = (p[0] + direction[0], p[1] + direction[1])
            if not must_be_walkable or PathPlanner.is_cell_walkable(mapdata, candidate):
                distance = PathPlanner.euclidean_distance(direction, (0, 0))
                neighbors.append((candidate, distance))
        return neighbors

    @staticmethod
    def neighbors_and_distances_of_4(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[tuple[int, int], float]]":
        return PathPlanner.neighbors_and_distances(
            mapdata, p, DIRECTIONS_OF_4, must_be_walkable
        )

    @staticmethod
    def neighbors_and_distances_of_8(
        mapdata: OccupancyGrid, p: "tuple[int, int]", must_be_walkable: bool = True
    ) -> "list[tuple[tuple[int, int], float]]":
        return PathPlanner.neighbors_and_distances(
            mapdata, p, DIRECTIONS_OF_8, must_be_walkable
        )

    @staticmethod
    def get_grid_cells(
        mapdata: OccupancyGrid, cells: "list[tuple[int, int]]"
    ) -> GridCells:
        """
        Returns a GridCells message for the given cells.
        :param mapdata [OccupancyGrid] The map information.
        :param cells   [[(int,int)]]   The cells as a list of tuples.
        :return        [GridCells]     The cells as a GridCells message.
        """
        grid_cells = GridCells()
        grid_cells.header.frame_id = "map"
        grid_cells.cell_width = mapdata.info.resolution
        grid_cells.cell_height = mapdata.info.resolution
        grid_cells.cells = [PathPlanner.grid_to_world(mapdata, cell) for cell in cells]
        return grid_cells

    @staticmethod
    def calc_cspace(
        mapdata: OccupancyGrid, include_cells: bool
    ) -> "tuple[OccupancyGrid, Union[GridCells, None]]":
        """
        Calculates the C-space (configuration space) of a given map.
        :param mapdata        [OccupancyGrid] The map information.
        :param include_cells  [bool]          Whether or not to include the cells in the return value.
        :return              [tuple]          A tuple containing the C-space and the cells.
        """
        # Create a new map for the C-space
        cspace = OccupancyGrid()
        cspace.header = mapdata.header
        cspace.info = mapdata.info
        cspace.data = list(mapdata.data)

        # Create a new map for the cells
        cells = None
        if include_cells:
            cells = []

        # For each cell in the map
        for i in range(mapdata.info.width):
            for j in range(mapdata.info.height):
                # If the cell is an obstacle
                if PathPlanner.get_cell_value(mapdata, (i, j)) > 50:
                    # Mark all cells in a 5x5 grid around it as obstacles
                    for di in range(-2, 3):
                        for dj in range(-2, 3):
                            ni = i + di
                            nj = j + dj
                            if PathPlanner.is_cell_in_bounds(mapdata, (ni, nj)):
                                idx = PathPlanner.grid_to_index(mapdata, (ni, nj))
                                cspace.data[idx] = 100
                                if include_cells:
                                    cells.append((ni, nj))

        # Return the C-space and the cells
        return cspace, PathPlanner.get_grid_cells(mapdata, cells) if include_cells else None

    @staticmethod
    def get_cost_map_value(cost_map: np.ndarray, p: "tuple[int, int]") -> int:
        return cost_map[p[1]][p[0]]

    @staticmethod
    def show_map(name: str, map: np.ndarray):
        cv2.imshow(name, map)
        cv2.waitKey(1)

    @staticmethod
    def calc_cost_map(mapdata: OccupancyGrid) -> np.ndarray:
        """
        Calculates the cost map for the given map.
        :param mapdata [OccupancyGrid] The map information.
        :return        [np.ndarray]    The cost map.
        """
        # Create a new map for the cost map
        cost_map = np.zeros((mapdata.info.height, mapdata.info.width), dtype=np.uint8)

        # For each cell in the map
        for i in range(mapdata.info.width):
            for j in range(mapdata.info.height):
                # If the cell is an obstacle
                if PathPlanner.get_cell_value(mapdata, (i, j)) > 50:
                    # Mark all cells in a 5x5 grid around it as obstacles
                    for di in range(-2, 3):
                        for dj in range(-2, 3):
                            ni = i + di
                            nj = j + dj
                            if PathPlanner.is_cell_in_bounds(mapdata, (ni, nj)):
                                cost_map[nj][ni] = 100

        # Apply a distance transform to the cost map
        cost_map = cv2.distanceTransform(
            (255 - cost_map).astype(np.uint8), cv2.DIST_L2, 5
        )

        # Normalize the cost map
        cost_map = cv2.normalize(cost_map, None, 0, 255, cv2.NORM_MINMAX)

        # Return the cost map
        return cost_map

    @staticmethod
    def create_hallway_mask(
        mapdata: OccupancyGrid, cost_map: np.ndarray, threshold: int
    ) -> np.ndarray:
        """
        Creates a mask for hallways in the given map.
        :param mapdata   [OccupancyGrid] The map information.
        :param cost_map  [np.ndarray]    The cost map.
        :param threshold [int]           The threshold for hallways.
        :return          [np.ndarray]    The hallway mask.
        """
        # Create a new map for the hallway mask
        hallway_mask = np.zeros((mapdata.info.height, mapdata.info.width), dtype=np.uint8)

        # For each cell in the map
        for i in range(mapdata.info.width):
            for j in range(mapdata.info.height):
                # If the cell is a hallway
                if PathPlanner.is_hallway_cell(mapdata, cost_map, (i, j), threshold):
                    hallway_mask[j][i] = 255

        # Return the hallway mask
        return hallway_mask

    @staticmethod
    def is_hallway_cell(
        mapdata: OccupancyGrid,
        cost_map: np.ndarray,
        p: "tuple[int, int]",
        threshold: int,
    ) -> bool:
        """
        Checks if the given cell is a hallway.
        :param mapdata   [OccupancyGrid] The map information.
        :param cost_map  [np.ndarray]    The cost map.
        :param p         [(int, int)]    The cell coordinate.
        :param threshold [int]           The threshold for hallways.
        :return          [bool]          True if the cell is a hallway, False otherwise.
        """
        # If the cell is not walkable, it is not a hallway
        if not PathPlanner.is_cell_walkable(mapdata, p):
            return False

        # Get the cost of the cell
        cost = PathPlanner.get_cost_map_value(cost_map, p)

        # If the cost is below the threshold, it is not a hallway
        if cost < threshold:
            return False

        # Count the number of walkable neighbors
        walkable_neighbors = 0
        for neighbor in PathPlanner.neighbors_of_8(mapdata, p):
            if PathPlanner.is_cell_walkable(mapdata, neighbor):
                walkable_neighbors += 1

        # If the number of walkable neighbors is 2, it is a hallway
        return walkable_neighbors == 2

    @staticmethod
    def get_first_walkable_neighbor(
        mapdata, start: "tuple[int, int]"
    ) -> "tuple[int, int]":
        """
        Gets the first walkable neighbor of the given cell.
        :param mapdata [OccupancyGrid] The map information.
        :param start   [(int, int)]    The cell coordinate.
        :return        [(int, int)]    The first walkable neighbor.
        """
        # For each direction
        for direction in DIRECTIONS_OF_8:
            # Get the neighbor
            neighbor = (start[0] + direction[0], start[1] + direction[1])

            # If the neighbor is walkable, return it
            if PathPlanner.is_cell_walkable(mapdata, neighbor):
                return neighbor

        # If no walkable neighbor is found, return the start
        return start

    @staticmethod
    def a_star(
        mapdata: OccupancyGrid,
        cost_map: np.ndarray,
        start: "tuple[int, int]",
        goal: "tuple[int, int]",
    ) -> "tuple[Union[list[tuple[int, int]], None], Union[float, None], tuple[int, int], tuple[int, int]]":
        """
        A* algorithm for finding the shortest path between two points.
        :param mapdata  [OccupancyGrid] The map information.
        :param cost_map [np.ndarray]    The cost map.
        :param start    [(int, int)]    The start cell.
        :param goal     [(int, int)]    The goal cell.
        :return         [tuple]         A tuple containing the path, the cost, the start, and the goal.
        """
        # If the start or goal is not walkable, return None
        if not PathPlanner.is_cell_walkable(mapdata, start):
            start = PathPlanner.get_first_walkable_neighbor(mapdata, start)
        if not PathPlanner.is_cell_walkable(mapdata, goal):
            goal = PathPlanner.get_first_walkable_neighbor(mapdata, goal)

        # Initialize the open and closed sets
        open_set = PriorityQueue()
        open_set.put(start, 0)
        came_from = {start: None}
        cost_so_far = {start: 0}

        # While the open set is not empty
        while not open_set.empty():
            # Get the current cell
            current = open_set.get()

            # If the current cell is the goal, return the path
            if current == goal:
                path = []
                while current is not None:
                    path.append(current)
                    current = came_from[current]
                path.reverse()
                return path, cost_so_far[goal], start, goal

            # For each neighbor of the current cell
            for neighbor, distance in PathPlanner.neighbors_and_distances_of_8(
                mapdata, current
            ):
                # Calculate the new cost
                new_cost = cost_so_far[current] + distance

                # If the neighbor is not in the cost_so_far or the new cost is less than the old cost
                if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                    # Update the cost_so_far
                    cost_so_far[neighbor] = new_cost

                    # Calculate the priority
                    priority = new_cost + PathPlanner.euclidean_distance(neighbor, goal)

                    # Add the neighbor to the open set
                    open_set.put(neighbor, priority)

                    # Update the came_from
                    came_from[neighbor] = current

        # If no path is found, return None
        return None, None, start, goal

    @staticmethod
    def path_to_message(mapdata: OccupancyGrid, path: "list[tuple[int, int]]") -> Path:
        """
        Converts the given path into a Path message.
        :param mapdata [OccupancyGrid] The map information.
        :param path    [[(int,int)]]   The path as a list of tuples (cell coordinates).
        :return        [Path]          The path as a Path message.
        """
        path_msg = Path()
        path_msg.header.frame_id = "map"
        path_msg.poses = PathPlanner.path_to_poses(mapdata, path)
        return path_msg