class Obstacle:
    def __init__(self, obstacleId, vertices=None):
        self.id = obstacleId
        self.vertices = vertices if vertices else []
    
    def addVertex(self, x, y):
        self.vertices.append((float(x), float(y)))
    
    def setVertices(self, vertices):
        self.vertices = [(float(x), float(y)) for x, y in vertices]
    
    def getVertices(self):
        return self.vertices
    
    def toDict(self):
        return {'id': self.id, 'vertices': self.vertices}
    
    def __repr__(self):
        return f"Obstacle(id={self.id}, vertices={len(self.vertices)})"