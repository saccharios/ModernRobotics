I implemented an RRT.

The sampling is from uniform distribution over [-0.5,0.5] x [-0.5,0.5], but every tenth sample is guaranteed to be the end-point at [0.5,0.5]

To store the tree, I use the anytree package.
Unfortunately, to generate the required csv-files I had to implement quite some functions to make it work.

Furthermore, I decided to add classes for a Point, Line and Disk to be able to add functions such as distance_to and intersects in those classes.
Normally I would split up the file into multiple files, so the reader is immediately pointed to the interesting parts of the algorithm.
I have omitted this step out of lazyness.

I have added 3 screenshot of different possible solutions.
