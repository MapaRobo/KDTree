# KDTree Better Version
## Note:
The other version had consistent memory issues so this is a new library without that fault

## Usage:
1. Type ```g++ testerFile.cpp treeWrapper.cpp  KDTree.cpp -o tester.o && ./tester.o```
2. Make sure gnuplot is installed
3. Copy text from plot.plt and run in gnuplot


Simple C++ static KD-Tree implementation with minimal functionality.

- points are given as STL vectors (and inserted in their own STL vector) so supports n-dimensional points for any n
- makes full trees, (i.e. does not cut-off the branching at some arbitrary level) giving the nearest neighbor query have (strong) logarithmic complexity.
- builds the tree in one go (does not support adding nodes, the tree is built from a list of points and cannot be altered afterwards)
- points are assumed to be STL vectors
- it provides the following queries:
	- nearest neighbor
	- neighbors within a given distance

## License and copyright

© J. Frederico Carvalho
Licensed under the [BSD3 License](LICENSE)
