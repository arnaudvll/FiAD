
import numpy as np

def distance_to_obstacle(x,y,grid,width,height,scale):
    try:
        distance=distance_to_obstacle_compute(x,y,grid,width,height,scale)[0]
        return distance
    except TypeError:
        print("no value available")
        return 0


def distance_to_obstacle_coord(x,y,grid,width,height,scale):
    return distance_to_obstacle_compute(x,y,grid,width,height,scale)


def distance_to_obstacle_compute(x,y,grid,width,height,scale):
    try:
        distance=0
        cell_x=0
        cell_y=0

        if int(round(width / scale)) > int(round(x / scale)) and int(round(height / scale)) > int(round(y / scale)):
            x_to_grid =int(round(x / scale))
            y_to_grid = int(round(y / scale))
        else:
            # FIXME in case of outboung value
            return 10000
        targeted_y =y_to_grid
        while targeted_y < int(height / scale) :
            if grid[targeted_y][x_to_grid]== 100:
                break
            distance+=1
            targeted_y+=1
            cell_x=x_to_grid*scale
            cell_y=targeted_y*scale
        # print "---> "+str(distance)
        return distance,cell_x,cell_y
    except IndexError:
        print("error during distance evaluation")




def update_coord_according_scale(x,y,scale):
    return int(round(x/scale)),int(round(y/scale))



    # Input a pandas series 
def std(particleslist):
    index=[]
    serie_part=[]
    for i in range(0, len(particleslist)):
        serie_part.append(round(particleslist[i].x / 10, 1))
        index.append(i)
    p_particles = np.array(serie_part)
    compute_std=np.std(p_particles)
    return compute_std
