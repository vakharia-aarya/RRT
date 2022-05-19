# Rapidly Exploring Random Trees (RRT)

Path planning in robotics is an important task and a robotics field of it own. A path planning algorithm gives a feasible collision-free path for going from one place to another.  

Path planning in robotics presents a challenge that algorithms like A* cannot over come. Since in real-world scenarios the search space is continuous and there are no pre-determined nodes that the robot can travel over. RRTs work by randomly sampling points and then connected to the closest available node. 

Before connecting a node, we check to see if the vertex lies outside of an obstacle. If it does the node is connected else a new point is sampled in the search space. 

**Output**

![Path (red) from source to destination](https://github.com/vakharia-aarya/RRT/blob/main/test_planar_rrt_graph.png)
