#  Cartesian transformation between two coordinate systems
from math import cos, sin, radians

# Degrees --> radians
def trig(angle):
  r = radians(angle)
  return cos(r), sin(r)

#  Returns the transformation matrix
def matrix(rotation=(0,0,0), translation=(0,0,0)): 
  xC, xS = trig(rotation[0])
  yC, yS = trig(rotation[1])
  zC, zS = trig(rotation[2])
  dX = translation[0]
  dY = translation[1]
  dZ = translation[2]

  return [[zC*yC, zC*yS*xS-zS*xC , zC*yS*xC+zS*xS, dX],    # Rotation done in the order z, y', x''  
     [zS*yC, zS*yS*xS+zC*xC, zS*yS*xC-zC*xS, dY],
     [-yS, yC*xS, yC*xC, dZ],
     [0, 0, 0, 1]]
# Applying transformations
def transform(point=(0,0,0), vector=(0,0,0)):
  p = [0,0,0]
  for r in range(3):
    p[r] += vector[r][3]
    for c in range(3):
      p[r] += point[c] * vector[r][c]
  return p

