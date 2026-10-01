"""Grid world model for the 2D autonomy simulation.

The world is a rectangular grid of cells addressed by ``(x, y)`` tuples,
with ``0 <= x < width`` and ``0 <= y < height``. A cell is either free or
occupied by an obstacle. The world is the ground truth: the drone agent
never reads it directly except through its sensor model, so the world can
change mid-mission (a new obstacle appears) without the agent knowing
until the obstacle enters its sensor radius.
"""

from __future__ import annotations

from dataclasses import dataclass, field

Cell = tuple[int, int]


@dataclass
class GridWorld:
    """A rectangular grid with a set of obstacle cells."""

    width: int
    height: int
    obstacles: set[Cell] = field(default_factory=set)

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("world dimensions must be positive")
        for cell in self.obstacles:
            if not self.in_bounds(cell):
                raise ValueError(f"obstacle out of bounds: {cell}")

    def in_bounds(self, cell: Cell) -> bool:
        """True when the cell lies inside the grid."""
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def is_occupied(self, cell: Cell) -> bool:
        """True when the cell holds an obstacle."""
        return cell in self.obstacles

    def is_free(self, cell: Cell) -> bool:
        """True when the cell is in bounds and holds no obstacle."""
        return self.in_bounds(cell) and cell not in self.obstacles

    def add_obstacle(self, cell: Cell) -> None:
        """Place an obstacle. Raises ValueError when out of bounds."""
        if not self.in_bounds(cell):
            raise ValueError(f"obstacle out of bounds: {cell}")
        self.obstacles.add(cell)

    def remove_obstacle(self, cell: Cell) -> None:
        """Remove an obstacle if present."""
        self.obstacles.discard(cell)

    def copy(self) -> "GridWorld":
        """An independent copy with the same dimensions and obstacles."""
        return GridWorld(self.width, self.height, set(self.obstacles))
