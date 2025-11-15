class Obstacle:
    '''
    polygon obstacle with vertices in mm (zone-local, bottom-left origin)
    '''
    
    def __init__(self, obstacleId, vertices=None):
        self.id = obstacleId
        self.vertices = vertices if vertices else []
    
    def addVertex(self, x, y):
        '''
        add vertex to obstacle polygon
        '''
        self.vertices.append((float(x), float(y)))
    
    def setVertices(self, vertices):
        '''
        set all vertices at once
        '''
        self.vertices = [(float(x), float(y)) for x, y in vertices]
    
    def getVertices(self):
        '''
        get list of vertices as (x, y) tuples in mm
        '''
        return self.vertices
    
    def toDict(self):
        '''
        convert to dictionary for JSON serialization
        '''
        return {'id': self.id, 'vertices': self.vertices}
    
    def __repr__(self):
        return f"Obstacle(id={self.id}, vertices={len(self.vertices)})"
