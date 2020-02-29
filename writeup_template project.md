## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation that includes...

### < Here's my answer to Explain the Starter Code > **
As for backyard_flyer_solution.py, it will respond each task against sent message at each locfal position. What was modified in motion planning.py is a new method so called plan_path consist of reading obstacle map, making grid with safety margin.

planning_utils .py is a module which provide create grid, a-star and heeuristic function.

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. Explain briefly how you accomplished this in your code.

###< Here's my explanation how to set my global homeposition >

Home position data is listed in 1st row in "colliders.csv file as shown below.

lat0 37.792480, lon0 -122.397450

posX,posY,posZ,halfSizeX,halfSizeY,halfSizeZ

-310.2389,-439.2315,85.5,5,5,85.5

In order to get lateral and longitudinal data as float data type, I wrote below code. "readline()" read 1st row data, "strip()" work as delete space, "split(',') will split data as each element by ','.   

        lat0 = 0.0
        lon0 = 0.00
        with open("colliders.csv", 'r') as csv_file:
            first_line = csv_file.readline().split(',')
            print(first_line)
    
            lat0 = float(first_line[0].strip().split(' ')[1])
            lon0 = float(first_line[1].strip().split(' ')[1])


For retrieve current global position and convert to current local position using global_to_local(), we learned about the 'global_to_local' which how to convert GPPS position (longitude, latitude, altitude) to NED frame (North, East, Down) 
in Lesson2, 5. Geodetic to NED Excercise.

Here I show current global position.

        print("Show current global position")
        print(self.global_position)

This code will give us local (NED) position.

        local_north, local_east, local_down = global_to_local(self.global_position, self.global_home)


And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
Here as long as you successfully determine your local position relative to global home you'll be all set. Explain briefly how you accomplished this in your code.

As I explained above, 'global_to_local' gives us the local position with attributes, self.global_position and self.global_home.  

Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. As long as it works you're good to go!

        grid_start = ((local_north + north_offset) , (local_east +-east_offset)) 


#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.

Here I set goal as latitude=30, longitude = 50, altitude = 0 in global frame.
And make it to local with north and east offset following code.
       
        goal_global = [ 30, 50, 0 ]
        goal_local = global_to_local (goal_global, self.global_home)
        grid_goal = (goal_local[0] +- north_offset, goal_local[1] +- east_offset)
        print( "Goal in local frame (NED)" )
        print(grid_goal[0], grid_goal[1])
        
#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

I have added cost for diagonal movement as below.

    SOUTH_EAST = (1, 1, np.sqrt(2))
    NORTH_EAST = (-1, 1, np.sqrt(2))
    SOUTH_WEST = (1, -1, np.sqrt(2))
    NORTH_WEST = (-1, -1, np.sqrt(2))   
    
I also added code if the node is off the grid or it's obstacle.

    if x + 1 > n or y + 1 > m or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)
    if x - 1 < 0 or y + 1 > m or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if x + 1 > n or y - 1 < 0 or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if x - 1 < 0 or y - 1 < 0 or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.

To prune ubnecessary waypoints, I have added in 2 functions, which is "collision_check" and "pruned_path_exec" in planning_utils.py.

As for collision_check, I used bresenham function.

    def collision_check(p1, p2, grid):
        # Calculate privided line with p1-p2 using the Bresenham algorithm
        covered_cells = list(bresenham(p1[0], p1[1], p2[0], p2[1]))

        # Check if any obstacle in the cell
        for cell in covered_cells:
            if grid[cell[0], cell[1]]:
                return False
        return True

    def pruned_path_exec(path, grid):
        pruned_path = [p for p in path]

        finished_prunning = False
        while not finished_prunning:
            pruned_check = False
            for i in range(len(pruned_path) - 2):
                p1, p2, p3 = pruned_path[i],pruned_path[i+1], pruned_path[i+2]

                # As we fly directly from p1 to p3, remove p2
                if collision_check(p1, p3, grid):
                    pruned_path.remove(p2)
                    pruned_check = True
                    break
            finished_prunning = not pruned_check

        return pruned_path

In the end, I added execusion code in the motion_planning.py as below.

        print('Calculating pruned path')
        path = pruned_path_exec(path, grid)
        print('Number of waypoints: {}'.format(len(path)))


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


