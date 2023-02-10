# TravelingSalesmanProblem

Using a branch and bound algorithm and reducing a matrix, the final project for my Algorithm Design and Analysis course was to test a tried and true algorithm used to solve the traveling salesman problem. The goal of this problem is to find the most efficient route between a set of cities. However, you cannot revisit cities after you have already visited them. 

The branch and bound algorithm branches off potential paths in a tree-like structure (using recursion to further test these branches). In this way, the algorithm explores many possible paths in order to find the most efficient solution.

There are some functions that are called but not included in my code. This is because they were given to us with the default code and I wanted to include only the parts that were written by me. These functions are listed below for reference as to their functionality.

### self.defaultRandomTour()

This is a function given to us that finds a random, valid tour to use as reference during the branch and bound algorithm. It runs in O(n) time due to the fact that the function has a while loop that runs until a valid path is found. This served as a good indicator as to whether or not there was even a valid path to begin with, since this function will only return a valid path if it exists. 
