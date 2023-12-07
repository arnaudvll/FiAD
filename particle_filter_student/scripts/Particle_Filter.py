import random


from common.Particle import Particle
from common.ToolBox import distance_to_obstacle,update_coord_according_scale
import math

class Particle_Filter:

    NB_PARTICLES=200
    FIXED_PLANE_Y = 100
    increment = 0
    DISTANCE_ERROR = random.randint(1,30)

    width=0
    height=0

    MOTION_PLANNER_MIN=-1
    MOTION_PLANNER_MAX=5

    SCALE_FACTOR=10

    obs_grid=[]
    particle_list=[]


    def __init__(self,width,height,obs_grid):
        self.width=width
        self.height=height
        self.obs_grid=obs_grid
        self.particle_list=self.getRandParticle(self.NB_PARTICLES, 0, width, 0, height)

    def resetParticle(self):
        self.particle_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, 0, self.height)

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------------- COMPUTED RANDOM PARTICLES--------------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def getRandParticle(self,nbr, start_x, max_x, start_y, max_y):
        particle_list = []
        ###################################
        ##### TODO
        ##   nbr: number of particles
        ##   start_x: min x possible coordinate
        ##   max_x: max x possible coordinate
        ##   start_y: min y possible coordinate
        ##   max_y: max y possible coordinate
        #####
        ## Use the Particle object to fill the list particle_list
        ##
        #particle_list = [[0 for _ in range(start_x, max_x + 1)] for _ in range(start_y, max_y + 1)]
        for _ in range(nbr):
            rand_x = random.randint(start_x, max_x)
            rand_y = random.randint(start_y, max_y)

            weight = random.random()
            proba = random.random()

            particle = Particle(rand_x, rand_y, weight, proba)
            particle_list.append(particle)

        return particle_list

        # ----------------------------------------------------------------------------------------------------------------
        # ----------------------------------- UPDATE PARTICLE ACCORDING NEX POSE-----------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def updateParticle(self,plane_pose):
        # process particle according motion planning
        self.particle_list = self.motion_prediction()

        current_distance_to_obstacle = distance_to_obstacle(plane_pose['x'], plane_pose['y'], self.obs_grid,self.width,self.height,self.SCALE_FACTOR)

        self.weightingParticle_list(current_distance_to_obstacle)


        # ----------------------------------------------------------------------------------------------------------------
        # -------------------------------------- MOTION PREDICTION AND RESAMPLING   --------------------------------------
        # ----------------------------------------------------------------------------------------------------------------
    def motion_prediction(self):
        new_particle_list = []
        choices = {}
        for i in range(len(self.particle_list)):
            choices[self.particle_list[i].id()] = self.particle_list[i].w

            ###################################
            ##### TODO
            ##   self.particle_list: list of available particles
            ##
            #####
            ## Use the function self.weighted_random_choice(choices) returning
            #  coordinate from a particle according a
            ##  roulette wheel algorithm
            #  Note that weighted_random_choice return a string containing coodinate x and y of the selected particle
            #   coord = self.weighted_random_choice(choices)
            #   x_coord = int(coord.split('_')[0])
            #   y_coord = int(coord.split('_')[1])
        for i in range(self.NB_PARTICLES):
            coord = self.weighted_random_choice(choices)
            x_coord = int(coord.split('_')[0])
            y_coord = int(coord.split('_')[1])
            weight = choices[coord]
        
            rand_movement_x = 1 + self.increment + round(random.gauss(0, self.width/120))
            rand_movement_y = round(random.gauss(0, self.height/150))

            new_particle = Particle(x_coord + rand_movement_x, y_coord + rand_movement_y, weight, 1)
            new_particle_list.append(new_particle)

        return new_particle_list

        # -------------------------------------------------------
        # ----------- SELECT PARTICLE  -----------
        # -------------------------------------------------------
    def weighted_random_choice(self,choices):
        ###################################
        ##### TODO
        ##   choices: dictionary holding particle coordination as key
        ##  and weight as value
        ##  return the selected particle key
        #####
        weighted_list = []

        for particle in choices.keys():
            for _ in range(round(choices[particle] * 100)):
                weighted_list.append(particle)
        random_choice = random.choice(weighted_list)

        return random_choice

    # ----------------------------------------------------------------------------------------------------------------
    # --------------------------------------------- EVALUATE PARTICLE (proba) ---------------------------------------
    # ----------------------------------------------------------------------------------------------------------------
    def weightingParticle_list(self,observed_distance):
        sum_weights = 0
        for i in range(len(self.particle_list)):
            #Compute individual particle weight
            current_weight = self.weightingParticle(self.particle_list[i].x,  self.particle_list[i].y, observed_distance)
            self.particle_list[i].w = current_weight
            sum_weights += current_weight
        for i in range(len(self.particle_list)):
            if sum_weights != 0:
                #compute proba sucha as weight is normalized
                self.particle_list[i].proba = self.particle_list[i].w / float(sum_weights)
            else:
                self.particle_list[i].proba = 0


    # -----------------------------------------------------
    #  ----------- EVALUATE PARTICLE (Weight)  -----------
    # -----------------------------------------------------
    def weightingParticle(self,p_x, p_y, observed_distance):
        ###################################
        ##### TODO
        ##   p_x: x coordinate of the particle p
        ##  p_y: y coordinate of the particle p
        ##  observed_distance: distance to the ground
        ##  measure by the probe
        ##
        ## return weight corresponding to the given particle
        ## according observation
        ##
        ## Note ue the function distance_to_obstacle to get the
        ## estimate particle to the ground distance
        particle_distance = distance_to_obstacle(p_x, p_y, self.obs_grid, self.width, self.height, self.SCALE_FACTOR)
        weight = 1/(1 +  0.5 * (particle_distance - observed_distance)**2 / self.DISTANCE_ERROR)

        return weight
