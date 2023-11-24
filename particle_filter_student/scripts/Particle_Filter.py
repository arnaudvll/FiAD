import random


from common.Particle import Particle
from common.ToolBox import distance_to_obstacle,update_coord_according_scale
import math

class Particle_Filter:

    NB_PARTICLES=50
    FIXED_PLANE_Y = 100
    increment = 0
    DISTANCE_ERROR = 2

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
        self.particle_list=self.getRandParticle(self.NB_PARTICLES,0,width,self.FIXED_PLANE_Y,self.FIXED_PLANE_Y)

    def resetParticle(self):
        self.particle_list = self.getRandParticle(self.NB_PARTICLES, 0, self.width, self.FIXED_PLANE_Y, self.FIXED_PLANE_Y)

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
            particle = Particle(rand_x, self.FIXED_PLANE_Y, 1, 1)
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
        
            rand_movement = 1 + self.increment + round(random.gauss(0, 1))
            new_particle = Particle(x_coord + rand_movement, y_coord, 1, 1)
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
            current_weight = self.weightingParticle(self.particle_list[i].x,  self.FIXED_PLANE_Y+50, observed_distance)
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
        weight = 1/(1 + 5* abs(particle_distance - observed_distance))
        return weight
