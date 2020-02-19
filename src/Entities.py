import functools
import warnings
from enum import Enum


class Slot:
    def __init__(self, x, y):
        self.row = x
        self.col = y
        self.Name = "("+str(x) + "," + str(y) + ")"

    def __eq__(self, other):
        return self.row == other.row and self.col == other.col

    def __ne__(self, other):
        return self.row != other.row or self.col != other.col

    def __hash__(self):
        return hash((self.row, self.col))

    def go_west(self):
        return Slot(self.row, self.col - 1)
    def go_east(self):
        return Slot(self.row, self.col + 1)

    def go_north(self):
        return Slot(self.row - 1, self.col)

    def go_south(self):
        return Slot(self.row + 1, self.col)

    def increase_cols(self):
        return Slot(self.row, self.col + 1)

    def decrease_rows(self):
        return Slot(self.row - 1, self.col)

    def increase_rows(self):
        return Slot(self.row + 1, self.col)

    def __str__(self):
        return str(int(self.row)) + "," + str(int(self.col))

    def __repr__(self):
        return str(self)

    def __getitem__(self, item):
        if item == 0:
            return self.row
        elif item == 1:
            return self.col
        else:
            raise IndexError()

    def GoByDirection(self, direction):
        # type: (Direction) -> Slot

        if direction == Direction.N:
            return self.decrease_rows()
        elif direction == Direction.E:
            return self.increase_cols()
        elif direction == Direction.S:
            return self.increase_rows()
        elif direction == Direction.W:
            return self.go_west()


class Orientation:
    def __init__(self):
        self.Horizontal = 0
        self.Vertical = 1


class Direction(Enum):
    N = 0
    E = 1
    S = 2
    W = 3
    Z = 5


def deprecated(func):
    """This is a decorator which can be used to mark functions
    as deprecated. It will result in a warning being emitted
    when the function is used."""
    @functools.wraps(func)
    def new_func(*args, **kwargs):
        warnings.simplefilter('always', DeprecationWarning)  # turn off filter
        warnings.warn("Call to deprecated function {}.".format(func.__name__),
                      category=DeprecationWarning,
                      stacklevel=2)
        warnings.simplefilter('default', DeprecationWarning)  # reset filter
        return func(*args, **kwargs)
    return new_func
