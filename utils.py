import arcade

"""
    class for 2D position

    Args:
        x: position x
        y: position y
"""


class Position2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y


"""
    environment class which contain all the basic information of the graph
"""


class environment2D:
    def __init__(self, numBugs):
        self.widthX = 100
        self.widthY = 100
        self.DIAMETER = 1

        self.numBugs = numBugs
        self.randomness = 1.0
        self.neighborhood = 4
        self.maxNeighborhood = 20
        self.repulsionRadius = 4
        self.maxSensorRange = 8 * self.DIAMETER

        self.particleOutsideMode = 0
        self.form = 0

        self.objects = []
        self.objectLocation = []

    # method to define shape form
    def setForm(self, val):
        self.form = val

    # method to define particle mode, mode0: move, mode1: delete
    def setMode(self, val):
        self.particleOutsideMode = val

    # method to add agent into the environment
    def setObjectLocation(self, obj, location):
        self.objects.append(obj)
        self.objectLocation.append(location)

    # method to calculate distance between two agent
    def calculateDistance(self, positionA, positionB):
        return ((positionA.x-positionB.x)**2 + (positionA.y-positionB.y)**2)**.5

    # method to obtain the agent's neighbors within sensing range
    def getNeighborsWithinDistance(self, location, maxNum=0):
        neighbors = []
        numNeighbors = 0
        if maxNum <= 0:
            maxNum = self.maxNeighborhood
        for agent in self.objects:
            position = agent.realPos
            if numNeighbors > maxNum:
                return neighbors
            if self.calculateDistance(location, position) < self.maxSensorRange:
                neighbors.append(agent)
                numNeighbors += 1
        return neighbors

    # method to return agent's location
    def getObjectLocation(self, agent):
        return agent.realPos


"""
    function to determine the markers

    Args:
        agent: agent
        numStep: number of time step
        environment: the environment contain basic parameters and agents list
"""


def marker(agent, numStep, environment):
    if environment.form == 0:
        inside = 961 > (agent.realPos.x - 50) ** 2 + (agent.realPos.y - 50) ** 2 > 361
    elif environment.form == 1:
        inside = 961 > (agent.realPos.x - 50) ** 2 + (agent.realPos.y - 50) ** 2 > 361 and 49 < agent.realPos.x
    elif environment.form == 2:
        inside = 961 > (agent.realPos.x - 50) ** 2 + (agent.realPos.y - 50) ** 2 > 361
    else:
        inside = None
        print('Error form: must be 0, 1, 2')
    if inside:
        return arcade.color.GREEN
    elif numStep < 500:
        return arcade.color.RED
    else:
        return arcade.color.WHITE
