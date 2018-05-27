# Adam Heaney
# Lab 8 - Particle Filter

from grid import *
from particle import Particle
from utils import *
from setting import *
import numpy as np

## Math helpers ##

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def magnitude(self):
        return ((self.x * self.x) + (self.y * self.y))**(0.5)

def sqr(x):
    return x * x

# compute angle between (-180,180]
# faster than diff_heading_deg
def shiftAngle(angle):
    angleMod = abs(angle) % 360.0
    if angle > 0.0:
        if angleMod <= 180.0:
            return angleMod
        else:
            return angleMod - 360.0
    else:
        if angleMod < 180.0:
            return -angleMod
        else:
            return (-angleMod) + 360.0

def clamp01(x):
    if x < 0.0:
        return 0.0
    elif x > 1.0:
        return 1.0
    else:
        return x

# helper that adds gaussian noise to a particle
def add_particle_noise(particle, trans_sigma, rot_sigma):
    return Particle(
        particle.x + random.gauss(0.0, trans_sigma),
        particle.y + random.gauss(0.0, trans_sigma),
        particle.h + random.gauss(0.0, rot_sigma))

TRANS_SIGMA = 1.0
HEADING_SIGMA = 10.0

# helper that computes a gaussian pdf for a particle
NORM_COEFF = 1.0 / (math.tau**(1.5) * HEADING_SIGMA * sqr(TRANS_SIGMA))
NORM_TRANSEXP_COEFF = -1.0 / (2.0 * sqr(TRANS_SIGMA))
NORM_HEADINGEXP_COEFF = -1.0 / (2.0 * sqr(HEADING_SIGMA))
def gaussianPdf(dx, dy, dh):
    return NORM_COEFF * math.exp(
        (NORM_TRANSEXP_COEFF * sqr(dx)) +
        (NORM_TRANSEXP_COEFF * sqr(dy)) +
        (NORM_HEADINGEXP_COEFF * sqr(dh)))

ODOM_NOISE_SCALAR = 0.3
REPLACE_FROM_MEAN_ERROR_SCALAR = 0.1

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """

    vMove = Vector2(odom[0], odom[1])

    # Move each particle by odom and add noise.
    cntr = 0
    while cntr < len(particles):
        particle = particles[cntr]
        vRotated = rotate_point(vMove.x, vMove.y, particle.h)
        particle.x += vRotated[0]
        particle.y += vRotated[1]
        particle.h += odom[2]
        particles[cntr] = add_particle_noise(particle, vMove.magnitude() * ODOM_NOISE_SCALAR, odom[2] * ODOM_NOISE_SCALAR)
        particles[cntr].h = shiftAngle(particles[cntr].h)
        cntr += 1

    return particles

def measurement_update(particles, measured_marker_list, grid):
    """ Particle filter measurement update

        Arguments: 
        particles -- input list of particle represents belief \tilde{p}(x_{t} | u_{t})
                before meansurement update (but after motion update)

        measured_marker_list -- robot detected marker list, each marker has format:
                measured_marker_list[i] = (rx, ry, rh)
                rx -- marker's relative X coordinate in robot's frame
                ry -- marker's relative Y coordinate in robot's frame
                rh -- marker's relative heading in robot's frame, in degree

                * Note that the robot can only see markers which is in its camera field of view,
                which is defined by ROBOT_CAMERA_FOV_DEG in setting.py
				* Note that the robot can see mutliple markers at once, and may not see any one

        grid -- grid world map, which contains the marker information, 
                see grid.py and CozGrid for definition
                Can be used to evaluate particles

        Returns: the list of particles represents belief p(x_{t} | u_{t})
                after measurement update
    """

    # if nothing was sensed, make no updates
    if len(measured_marker_list) == 0:
        return particles

    particleWeights = []
    sumOfWeights = 0

    accumulatedError = (0.0, 0.0, 0.0)
    errorSamples = 0

    # compute a weight for each particle
    for particle in particles:
        weight = 0

        if grid.is_in(particle.x, particle.y):

            # compute the pose of the grid markers relative to the particle
            markersRelativeToParticle = []
            for marker in grid.markers:
                m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])
                mr_x, mr_y = rotate_point(m_x - particle.x, m_y - particle.y, -particle.h)
                mr_h = shiftAngle(m_h - particle.h)
                markersRelativeToParticle.append((mr_x, mr_y, mr_h))

            # compute the weight of the particles based on its probability of sensing
            # landmarks with the same relative poses
            for sensedMarker in measured_marker_list:
                maxW = 0.0
                maxWError = (0,0,0)

                for particleMarker in markersRelativeToParticle:
                    dx = sensedMarker[0] - particleMarker[0]
                    dy = sensedMarker[1] - particleMarker[1]
                    dh = shiftAngle(sensedMarker[2] - particleMarker[2])
                    w = gaussianPdf(dx, dy, dh)

                    if w > maxW:
                        maxW = w
                        maxWError = (abs(dx), abs(dy), abs(dh))
                    weight += w

                # accumulate the error of the world marker that most closely matches each sensed marker
                # this is used later to determine how many random particles to add
                accumulatedError = (
                    accumulatedError[0] + maxWError[0],
                    accumulatedError[1] + maxWError[1],
                    accumulatedError[2] + maxWError[2])
                errorSamples += 1

        particleWeights.append(weight)
        sumOfWeights += weight

    if sumOfWeights == 0:
        print("sum of weights is 0, recreateing all particles")
        particles = Particle.create_random(PARTICLE_COUNT, grid)
        return particles

    # normalize weights and resample
    pWCounter = 0
    while pWCounter < len(particleWeights):
        particleWeights[pWCounter] /= sumOfWeights
        pWCounter = pWCounter + 1

    particles = np.random.choice(particles, len(particles), True, particleWeights)

    if errorSamples == 0:
        return particles

    # add some new completely random particles based on the mean error
    maxDim = (grid.width + grid.height) * 0.5
    meanErrorDxy = Vector2(accumulatedError[0], accumulatedError[1]).magnitude() / errorSamples / maxDim
    meanErrorDh = accumulatedError[2] / errorSamples / 180.0
    meanError = (meanErrorDxy + meanErrorDh) * 0.5

    numParticlesToRandomlyAdd = len(particles) * REPLACE_FROM_MEAN_ERROR_SCALAR * clamp01(meanError)
    cntr = 0
    while cntr < numParticlesToRandomlyAdd:
        particles[random.randint(0, len(particles) - 1)] = Particle(*grid.random_free_place())
        cntr = cntr + 1

    return particles
