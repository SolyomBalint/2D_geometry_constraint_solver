# Project Milestones

## Needed For 0.1.0 Beta release

 - [ ] Implement bottom-up decomposition algorithm 
 - [ ] Implement constraint graph class detection (under-, over-, well-constrained)
 - [ ] Implement solver geometric element: circle, Constraints: point-line-distance, point-circle-distance, tangency and so on
 - [ ] Implement algorithm testing (Get constraint graph generating)
 - [ ] Add Architecture, user and interface documentation hosted on github pages
 - [ ] Given there are tests, create github workflows for automatic building and testing
 - [ ] Implement comprehensive heuristics for solver
 - [ ] Modularize CMake targets to libraries, and provide correct install logic
 - [ ] Project [cleanup](#project-cleanup)

## Project Cleanup

 - [ ] Remove OGDF and implement the algorithms: separations pairs, cut vertices, induced subgraph
 - [ ] Remove the interdependency between the graph representation and the top-down decomposition
 - [ ] Implement edge case covering for top-down method (currently it's not correctly checked whether subgraphs are under-constrainted)
 - [ ] Refactor solver code, it is currently a mess
 - [ ] Modularize the solving pipeline, currently each stage is in one or two files
 - [ ] Remove the GUI into another repository. The GUI itself should be rewritten later
