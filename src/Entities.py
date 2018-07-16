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

    def GoLeft(self):
        return Slot(self.row, self.col - 1)
    def GoRight(self):
        return Slot(self.row, self.col + 1)
    def GoUp(self):
        return Slot(self.row - 1, self.col)
    def GoDown(self):
        return Slot(self.row + 1, self.col)



    def __str__(self):
        return str(int(self.row)) + "," +str(int(self.col))

    def __repr__(self):
        return str(self)