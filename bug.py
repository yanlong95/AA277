import arcade

from random import choice, random
from utils import Position2D, environment2D, marker

"""
    the function to calculate gradient

    Args:
        neighbor: the three neighbors list
        actual: the real position
        dist: the sensing distance list
        
    returns:
        the gradient in Position2D class
    
"""


def calculateGradient(neighbor, actual, dist):
    x = actual.x
    y = actual.y

    gX = 0.0
    gY = 0.0

    for i in range(len(neighbor)):
        nx = neighbor[i].x
        ny = neighbor[i].y
        d = dist[i]

        gX += -2 * (nx - x) * ((nx - x) ** 2 + (ny - y) ** 2 - d) ** .5 / ((nx - x) ** 2 + (ny - y) ** 2) ** .5
        gY += -2 * (ny - y) * ((nx - x) ** 2 + (ny - y) ** 2 - d) ** .5 / ((nx - x) ** 2 + (ny - y) ** 2) ** .5

    return Position2D(gX, gY)


"""
    the function to determine if the location is inside the shape
    shape form 0: default shape (whole donut, 20 < x,y < 30 & 70 < x,y < 80)
    shape form 1: first shape (half donut, 20 < x,y < 30 & 70 < x,y < 80)
    shape form 2: second shape (whole donut, 20 < x,y < 30 & 70 < x,y < 80)

    Args:
        x: location x
        y: location y
        environment: the environment contain basic parameters and agents list

    returns:
        True if the location is inside the shape otherwise return false

"""


def insideShape(x, y, environment):
    if environment.form == 0:
        inside = 900 > (x - 50) ** 2 + (y - 50) ** 2 > 400
    elif environment.form == 1:
        inside = 900 > (x - 50) ** 2 + (y - 50) ** 2 > 400 and 50 < x
    elif environment.form == 2:
        inside = 900 > (x - 50) ** 2 + (y - 50) ** 2 > 400
    else:
        inside = None
        print('Error form: must be 0, 1, 2')
    return inside


"""
    the function to determine if the location is inside the shape

    Args:
        location: location of the agent
        environment: the environment contain basic parameters and agents list

    returns:
        True if the location is inside the shape otherwise return false

"""


def isLocationInsideShape(location, environment):
    return insideShape(location.x, location.y, environment)


"""
    the function to determine if the two agent are conflict

    Args:
        a: first agent
        b: second agent
        DIAMETER: the DIAMETER the agents

    returns:
        True if the the agents are conflict otherwise return false

"""


def conflict(a, b, DIAMETER):
    if ((b.x < a.x < b.x + DIAMETER) or (b.x < a.x + DIAMETER < b.x + DIAMETER)) and \
            ((b.y < a.y < b.y + DIAMETER) or (b.y < a.y + DIAMETER < b.y + DIAMETER)):
        return True
    else:
        return False


"""
    the function to determine if the position is acceptable 

    Args:
        location: location of the agent
        agent: agent itself
        keepInShape: if the agent have to keep inside the shape
        environment: the environment contain basic parameters and agents list

    returns:
        True if the location is acceptable  otherwise return false

"""


def acceptablePosition(location, agent, keepInShape, environment):
    if not (0 <= location.x < environment.widthX and 0 <= location.y < environment.widthY) or \
            (keepInShape and not isLocationInsideShape(location, environment)):
        return False

    # get the neighbors within the sensing range
    nearbyObjects = environment.getNeighborsWithinDistance(location)

    # determine if the agent is conflict its neighbors
    if len(nearbyObjects) > 0:
        for i in range(len(nearbyObjects)):
            if nearbyObjects[i] is not None and (agent is None or nearbyObjects[i] != agent):
                ta = nearbyObjects[i]
                if conflict(location, environment.getObjectLocation(ta), environment.DIAMETER):
                    return False
    return True


"""
    BUg class: the class of each agent

    Args:
        position: the true position of the agent
        isKnown: if the agent own its true position
        lost: if the agent has an perceived position 
        inShape: if the agent is in shape (by perceived position)
        trueInShape: if the agent is in shape (by real position)
"""


class Bug:
    def __init__(self, position, isKnown):
        self.realPos = position
        self.perceivedPos = Position2D(0.0, 0.0)
        self.lost = True
        self.inShape = False
        self.trueInShape = False

        if isKnown:
            self.perceivedPos = position
            self.lost = False

    # the method return the neighbors of the agent
    def getNeighborhood(self, environment):
        if environment is not None:
            # return the neighbors with in the range
            return environment.getNeighborsWithinDistance(self.realPos, environment.neighborhood)
        else:
            print('Environment is not available!')
            return None

    # the method to return the perceived position
    def getPerceivedPosition(self):
        return self.perceivedPos

    # the method to return the real position
    def getRealPosition(self):
        return self.realPos

    # the method to determine agent status
    def isLost(self):
        return self.lost

    # Try to estimate the location from neighbors
    def tryToLocalizate(self, bugs):
        if bugs is not None and len(bugs) > 3:
            neighbors = []
            numNeighborsWithPosition = 0

            for i in range(len(bugs)):
                if numNeighborsWithPosition >= 3:
                    break

                # only use the information from agents who know their position
                neighbor = bugs[i]
                if not neighbor.isLost():
                    neighbors.append(neighbor)
                    numNeighborsWithPosition += 1

            if numNeighborsWithPosition == 3:
                self.perceivedPos = self.calculatePosition(neighbors)
                self.lost = False

    # Calculate the position using the gradient method
    def calculatePosition(self, neighbors):
        realPos = self.getRealPosition()

        maxError = 0.1
        beta = 0.5
        iter = 0
        maxIterations = 500

        neighborsPos = []
        dist = []
        for i in range(len(neighbors)):
            neighborsPos.append(neighbors[i].getPerceivedPosition())
            dX = neighbors[i].getRealPosition().x - realPos.x
            dY = neighbors[i].getRealPosition().y - realPos.y
            dist.append((dX ** 2 + dY ** 2) ** .5)

        # estimate current position
        if self.lost:
            currentPos = Position2D(realPos.x + (random() - 0.5) * 4, realPos.y + (random() - 0.5) * 4)
        else:
            currentPos = self.getPerceivedPosition()

        gradient = calculateGradient(neighborsPos, currentPos, dist)

        # iterate until converge (500 steps)
        while iter < maxIterations and abs(gradient.x) < maxError and abs(gradient.y) < maxError:
            currentPos = Position2D(currentPos.x - beta * gradient.x, currentPos.y - beta * gradient.y)
            iter += 1

        return currentPos

    # Generate a random motion vector
    def randomMove(self, environment):
        perceivedPos = self.getPerceivedPosition()
        tries = 0

        x = random() * 2 - 1.0
        y = random() * 2 - 1.0
        vRandom = Position2D(x, y)

        # Not allow it to leave the container if it is already inside the shape
        while tries < 20 and not acceptablePosition(
                Position2D(perceivedPos.x + vRandom.x, perceivedPos.y + vRandom.y), self, self.inShape, environment):
            x = random() * 2 - 1.0
            y = random() * 2 - 1.0
            vRandom = Position2D(x, y)
            tries += 1

        if tries >= 20:
            vRandom = Position2D(0.0, 0.0)
        return vRandom

    # calculate the movement vector
    def calculateMovementVector(self, bugs, environment):
        mX = 0.0
        mY = 0.0
        repulsionRadius = environment.repulsionRadius

        if bugs is not None and len(bugs) > 0:
            for i in range(len(bugs)):
                neighbor = bugs[i]

                if neighbor == self:
                    continue

                dX = neighbor.getPerceivedPosition().x - self.perceivedPos.x
                dY = neighbor.getPerceivedPosition().y - self.perceivedPos.y
                distance = (dX ** 2 + dY ** 2) ** .5

                # if too close, move the opposite direction
                if distance < repulsionRadius:
                    mX -= dX * (repulsionRadius - distance) / distance
                    mY -= dY * (repulsionRadius - distance) / distance

        return Position2D(mX, mY)

    # the method to update the agent's information and status for each time step
    def step(self, numStep, time1, time2, environment):
        index = environment.objects.index(self)

        # to change shape during simulation
        if numStep == time1:  # default whole donut
            environment.setForm(1)  # half donut
            environment.setMode = 1
        elif numStep == time2:
            environment.setForm(2)  # repair
            environment.setMode = 0

        vMove = Position2D(0.0, 0.0)

        # The agent does not know his position
        if self.lost:
            bugs = self.getNeighborhood(environment)
            self.tryToLocalizate(bugs)
            vMove = self.randomMove(environment)
        else:
            self.inShape = isLocationInsideShape(self.perceivedPos, environment)
            self.trueInShape = isLocationInsideShape(self.realPos, environment)

            # Check if the agent is inside the figure
            if not self.inShape:
                bugs = self.getNeighborhood(environment)

                # Find neighbor inside shape
                if environment.particleOutsideMode == 0:
                    iter = 0
                    iterMax = 3
                    foundNeighborInside = False
                    while not foundNeighborInside and iter < iterMax:
                        if bugs is not None and len(bugs) > 1:
                            for i in range(len(bugs)):
                                if foundNeighborInside is True:
                                    break
                                n = bugs[i]
                                if not n.lost and isLocationInsideShape(n.perceivedPos, environment):
                                    vMove = Position2D((n.perceivedPos.x - self.perceivedPos.x) / 2,
                                                       (n.perceivedPos.y - self.perceivedPos.y) / 2)
                                    foundNeighborInside = True
                        iter += 1

                        if not foundNeighborInside and iter < iterMax:
                            environment.getNeighborsWithinDistance(self.realPos)

                    if not foundNeighborInside:
                        vMove = self.randomMove(environment)
                # delete particle
                else:
                    del environment.objects[index]
                    return
            # the agent is inside and not lost
            else:
                bugs = self.getNeighborhood(environment)
                if len(bugs) > 1:
                    vMove = self.calculateMovementVector(bugs, environment)
                    if not acceptablePosition(Position2D(vMove.x + self.perceivedPos.x, vMove.y + self.perceivedPos.y),
                                              self, True, environment):
                        vMove = self.randomMove(environment)

                else:
                    vMove = self.randomMove(environment)
        # calculate movement of the agent and update the positions
        dx = vMove.x
        dy = vMove.y
        self.perceivedPos = Position2D(self.perceivedPos.x + dx, self.perceivedPos.y + dy)
        self.realPos = Position2D(self.realPos.x + dx, self.realPos.y + dy)
        environment.objects[index] = self


"""
    initialize the environment randomly.

    Args:
        scale: the enlarge the windows
        blank: empty space for graph sides
        environment: the initialized environment class 
"""


def start(environment, scale=6, blank=5):
    for x in range(environment.numBugs):
        # random sampling until the agent is acceptable
        location = Position2D(random() * environment.widthX, random() * environment.widthY)
        while not acceptablePosition(location, None, False, environment):
            location = Position2D(random() * environment.widthX, random() * environment.widthY)

        # random assign agent who has the true position
        newBug = Bug(location, choice([True, False]))
        environment.setObjectLocation(newBug, location)

    # draw the graph
    arcade.open_window(2 * blank + scale * env2D.widthX, 2 * blank + scale * env2D.widthY, 'swarm')
    arcade.set_background_color(arcade.color.WHITE)
    arcade.start_render()
    for i in range(len(env2D.objects)):
        agent = env2D.objects[i]
        arcade.draw_circle_filled(blank + scale * agent.realPos.x, blank + scale * agent.realPos.y,
                                  scale * env2D.DIAMETER, arcade.color.RED)
    arcade.finish_render()
    arcade.run()

    return environment


"""
    update the environment randomly.

    Args:
        scale: the enlarge the windows
        blank: empty space for graph sides
        environment: the initialized environment class 
"""


def run(environment, scale=6, blank=5):
    numStep = 0
    maxStep = 1000
    time1 = 500     # time to change from default shape form to first shape form
    time2 = 700     # time to change from first shape form to second shape form (recover)

    while numStep < maxStep:
        for agent in environment.objects:
            agent.step(numStep, time1, time2, environment)
        numStep += 1

        # draw the graph for each 100 time step
        if (numStep + 1) % 100 == 0:
            print('Step: ', numStep + 1)
            arcade.open_window(2 * blank + scale * env2D.widthX, 2 * blank + scale * env2D.widthY, 'swarm')
            arcade.set_background_color(arcade.color.WHITE)
            arcade.start_render()
            for agent in environment.objects:
                color = marker(agent, numStep, environment)
                arcade.draw_circle_filled(blank + scale * agent.realPos.x, blank + scale * agent.realPos.y,
                                          scale * env2D.DIAMETER, color)
            arcade.finish_render()
            arcade.run()


if __name__ == '__main__':
    numBugs = 300
    scale = 6
    blank = 5 * scale
    env2D = environment2D(numBugs)
    env2D = start(env2D)
    run(env2D, scale, blank)
