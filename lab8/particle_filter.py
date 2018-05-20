from grid import *
from particle import Particle
from utils import *
from setting import *
from scipy.stats import norm
import numpy as np

class Vector2:
    def __init__(self, x, y):
        self.x = x
        self.y = y
    def magnitude(self):
        return ((self.x * self.x) + (self.y * self.y))**(0.5)

def add_particle_noise(particle, trans_sigma, rot_sigma):
    particle.x = add_gaussian_noise(particle.x, trans_sigma)
    particle.y = add_gaussian_noise(particle.y, trans_sigma)
    particle.h = add_gaussian_noise(particle.h, rot_sigma)

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

def angleDifference(a1, a2):
    difference = abs(shiftAngle(a1) - shiftAngle(a2))
    if difference > 180.0:
        return 360.0 - difference
    else:
        return difference

def motion_update2(particles, odom):
    return particles

def motion_update(particles, odom):
    """ Particle filter motion update

        Arguments: 
        particles -- input list of particle represents belief p(x_{t-1} | u_{t-1})
                before motion update
        odom -- odometry to move (dx, dy, dh) in *robot local frame*

        Returns: the list of particles represents belief \tilde{p}(x_{t} | u_{t})
                after motion update
    """
    #motion_particles = []
    #return motion_particles

    vMove = Vector2(odom[0], odom[1])
    movementLength = vMove.magnitude()

    # Move each particle and add noise.
    for particle in particles:
        vRotated = rotate_point(vMove.x, vMove.y, particle.h)
        particle.x += vRotated[0]
        particle.y += vRotated[1]
        particle.h += odom[2]
        add_particle_noise(particle, movementLength * 0.2, odom[2] * 0.2)

    return particles

def measurement_update3(particles, measured_marker_list, grid):
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

    #if particles:
    #    return particles

    # if nothing was sensed, make no updates
    if len(measured_marker_list) == 0:
        return particles

    TRANS_SCALE = 12.0
    HEADING_SCALE = 12.0
    # WEIGHTED_RESAMPLE_PERCENTAGE = 0.5
    UNIFORM_RESAMPLE_PERCENTAGE = 0.05

    particleWeights = []

    sumOfWeights = 0
    for particle in particles:
        importance = 0

        if not grid.is_in(particle.x, particle.y):
            particle.x = min(grid.width, max(0, particle.x))
            particle.y = min(grid.height, max(0, particle.y))
            print("Clamped particle ", str(particle.x), str(particle.y))

        if True:
        #if grid.is_in(particle.x, particle.y):
            #markersRelativeToParticle = particle.read_markers(grid)

            # p particle sensed the same markers?
            # p marker was sensed?
            # Wparticle = sum<global markers>(P(getting the given sensor input))

            markersRelativeToParticle = []
            for marker in grid.markers:
                m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])
                # rotate marker into robot frame
                mr_x, mr_y = rotate_point(m_x - particle.x, m_y - particle.y, -particle.h)
                mr_h = diff_heading_deg(m_h, particle.h)
                markersRelativeToParticle.append((mr_x, mr_y, mr_h))
            for particleMarker in markersRelativeToParticle:
                pMarkers = []

                for sensedMarker in measured_marker_list:
                    dx = (sensedMarker[0] - particleMarker[0])# / TRANS_SCALE
                    dy = (sensedMarker[1] - particleMarker[1])# / TRANS_SCALE
                    #dh = angleDifference(sensedMarker[2], particleMarker[2])# / HEADING_SCALE
                    dh = diff_heading_deg(sensedMarker[2], particleMarker[2])
                    nx = norm.pdf(dx, scale=TRANS_SCALE) / norm.pdf(0, scale=TRANS_SCALE)
                    ny = norm.pdf(dy, scale=TRANS_SCALE) / norm.pdf(0, scale=TRANS_SCALE)
                    nh = norm.pdf(dh, scale=HEADING_SCALE) / norm.pdf(0, scale=HEADING_SCALE)
                    w = nx * ny * nh
                    pMarkers.append(w)
                importance += max(pMarkers)
                #print("imp ", str(importance))
        particleWeights.append(importance)
        sumOfWeights += importance
        #print("sow ", str(sumOfWeights))

    if sumOfWeights is 0:
        print("SoW is 0, recreateing all particles")
        particles = Particle.create_random(PARTICLE_COUNT, grid)
    else:
        pWCounter = 0
        while pWCounter < len(particleWeights):
            particleWeights[pWCounter] /= sumOfWeights
            pWCounter = pWCounter + 1

        #test code
        maxW = max(particleWeights)
        pWCounter = 0
        while pWCounter < len(particleWeights):
            particles[pWCounter].weight = particleWeights[pWCounter] / maxW # test code
            pWCounter = pWCounter + 1

        particles = np.random.choice(particles, len(particles), True, particleWeights)

    numParticlesToRandomlyResample = len(particles) * UNIFORM_RESAMPLE_PERCENTAGE
    cntr = 0
    while cntr < numParticlesToRandomlyResample:
        particles[random.randint(0, len(particles) - 1)] = Particle(*grid.random_free_place())
        cntr = cntr + 1

    return particles

class WeightedParticle:
    def __init__(self, particle):
        self.particle = particle
        self.weight = 0
        self.maxRandomValue = 0
        self.resampled = False

# ------------------------------------------------------------------------
def measurement_update2(particles, measured_marker_list, grid):
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
    # measured_particles = []
    # return measured_particles

    # if nothing was sensed, make no updates
    if len(measured_marker_list) == 0:
        return particles

    TRANS_SCALE = 2.0
    HEADING_SCALE = 5.0
    WEIGHTED_RESAMPLE_PERCENTAGE = 0.5
    UNIFORM_RESAMPLE_PERCENTAGE = 0.01

    accumulatedErrorX = 0.0
    accumulatedErrorY = 0.0
    accumulatedErrorH = 0.0

    weighted_particles = []

    totalSum = 0
    #highestParticleProbability = None
    for particle in particles:
        sumOfProbabilities = 0
        if not grid.is_in(particle.x, particle.y):
            accumulatedErrorX += grid.width * 0.5 * len(measured_marker_list)
            accumulatedErrorY += grid.height * 0.5 * len(measured_marker_list)
            accumulatedErrorH += 90.0 * len(measured_marker_list)
        else:
            markersRelativeToParticle = particle.read_markers(grid) #[]
            #for marker in grid.markers:
            #    m_x, m_y, m_h = parse_marker_info(marker[0], marker[1], marker[2])
            #    # rotate marker into robot frame
            #    mr_x, mr_y = rotate_point(m_x - particle.x, m_y - particle.y, -particle.h)
            #    mr_h = diff_heading_deg(m_h, particle.h)
            #    markersRelativeToParticle.append((mr_x, mr_y, mr_h))
            # for measuredMarker in measured_marker_list:
            #     for marker in markersRelativeToParticle:
            #         dx = measuredMarker[0] - marker[0]
            #         dy = measuredMarker[1] - marker[1]
            #         dh = angleDifference(measuredMarker[2], marker[2])
            #         nx = norm.pdf(dx, scale=TRANS_SCALE)
            #         ny = norm.pdf(dy, scale=TRANS_SCALE)
            #         nh = norm.pdf(dh, scale=HEADING_SCALE)
            for sensedMarker in measured_marker_list:
                highestProb = 0.0
                errorHighestProb = (0, 0, 0)
                for particleMarker in markersRelativeToParticle:
                    dx = (sensedMarker[0] - particleMarker[0])# / TRANS_SCALE
                    dy = (sensedMarker[1] - particleMarker[1])# / TRANS_SCALE
                    #dh = angleDifference(sensedMarker[2], particleMarker[2])# / HEADING_SCALE
                    dh = diff_heading_deg(sensedMarker[2], particleMarker[2])
                    nx = norm.pdf(dx, scale=TRANS_SCALE) / norm.pdf(0, scale=TRANS_SCALE)
                    ny = norm.pdf(dy, scale=TRANS_SCALE) / norm.pdf(0, scale=TRANS_SCALE)
                    nh = norm.pdf(dh, scale=HEADING_SCALE) / norm.pdf(0, scale=HEADING_SCALE)
                    p = nx * ny * nh
                    sumOfProbabilities += p
                    if p > highestProb:
                        highestProb = p
                        errorHighestProb = (abs(dx), abs(dy), abs(dh))
                accumulatedErrorX += errorHighestProb[0]
                accumulatedErrorY += errorHighestProb[1]
                accumulatedErrorH += errorHighestProb[2]
        weightedParticle = WeightedParticle(particle)
        weightedParticle.weight = sumOfProbabilities
        weighted_particles.append(weightedParticle)
        totalSum += sumOfProbabilities
        #if highestParticleProbability is None or sumOfProbabilities > highestParticleProbability:
        #    highestParticleProbability = sumOfProbabilities
        #weighted_particles[particle] = sumOfProbabilities

    numerrorSamples = (len(particles) * len(measured_marker_list))
    meanErrorX = accumulatedErrorX / numerrorSamples / (grid.width * 0.5)
    meanErrorY = accumulatedErrorY / numerrorSamples / (grid.width * 0.5)
    meanErrorH = accumulatedErrorH / numerrorSamples / 180.0
    meanError = (meanErrorX + meanErrorY + meanErrorH) / 3.0

    print("mean error: " + str(meanError))

    if totalSum is 0.0:
    #if highestParticleProbability is None or highestParticleProbability <= 0.0:
        weighted_particles = []

    maxWeight = 0.0

    particleWeights = []
    #totalWeightAdj = 0.0

    #maxRandomValue = 0.0
    for wParticle in weighted_particles:
        #wParticle.weight = (highestParticleProbability - wParticle.weight) / highestParticleProbability
        #totalWeightAdj += wParticle.weight
        wParticle.weight = wParticle.weight / totalSum
        #maxRandomValue += wParticle.weight
        #wParticle.maxRandomValue = maxRandomValue
        particleWeights.append(wParticle.weight)
        if wParticle.weight > maxWeight:
            maxWeight = wParticle.weight

    #for wParticle in weighted_particles:
    #    particleWeights.append(wParticle.weight / totalWeightAdj)

    #numParticlesToResample = len(particles) * meanError # * 2.0

    #numActuallyResampled = 0
    #weighted_particles = np.random.choice(weighted_particles, int(round(numParticlesToResample)), True, particleWeights)
    weighted_particles = np.random.choice(weighted_particles, len(weighted_particles), True, particleWeights)
    #numResampled = len(randSamples)
    #for sample in randSamples:
    #    sample.particle = Particle(*grid.random_free_place())
    #    sample.weight = 0.0
    #    if not sample.resampled:
    #        numActuallyResampled = numActuallyResampled + 1
    #    sample.resampled = True

    # numResampled = 0
    # numActuallyResampled = 0
    # while numResampled < numParticlesToResample:
    #     resampleValue = random.uniform(0.0, maxRandomValue)
    #     for wParticle in weighted_particles:
    #         if resampleValue < wParticle.maxRandomValue:
    #             wParticle.particle = Particle(*grid.random_free_place())
    #             wParticle.weight = 0.0
    #             if not wParticle.resampled:
    #                 numActuallyResampled = numActuallyResampled + 1
    #             wParticle.resampled = True
    #             break
    #     numResampled = numResampled + 1

    #print("particles resampled: " + str(numResampled) + " actual: " + str(numActuallyResampled))
    # sorted_particles = sorted(weighted_particles, key=weighted_particles.get)

    measured_particles = []
    for wParticle in weighted_particles:
        #test code
        if maxWeight > 0.0:
            wParticle.particle.weight = wParticle.weight / maxWeight
        measured_particles.append(wParticle.particle)
    # numParticlesToResample = len(particles) * WEIGHTED_RESAMPLE_PERCENTAGE
    # for particle in sorted_particles:htrehtdf
    #     if len(measured_particles) < numParticlesToResample:
    #         measured_particles.append(Particle(*grid.random_free_place()))
    #     else:
    #         measured_particles.append(particle)

    #numParticlesToRandomlyResample = len(particles) * UNIFORM_RESAMPLE_PERCENTAGE
    #cntr = 0
    #while cntr < numParticlesToRandomlyResample:
    #    measured_particles[random.randint(0, len(measured_particles) - 1)] = Particle(*grid.random_free_place())
    #    cntr = cntr + 1

    return measured_particles


