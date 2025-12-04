# 2D Geometric Constraint Solver

This project aims to implement a fully-fledged 2D geometric constraint solver. The framework was developed as a university project at the Budapest University of Technology and Economics (BME), Faculty of Electrical Engineering and Informatics (VIK), Department of Control Engineering and Information Technology.

## Current State

The project currently focuses on laying down the foundation of a broader solver framework. At the moment, a constraint-graph-based representation is used for the geometric constraint system (GCS). This data structure dictates the implemented algorithms:

 * Top-down decomposition: The constraint graph is decomposed using a top-down method based on [Owen's](https://dl.acm.org/doi/10.1145/112515.112573) papers.

 * Newton-Raphson-based solver: Once the GCS is decomposed into smaller quadratically solvable problems, numerical iterative methods are used to identify the possible roots of the created equation systems.

 * Triangle-orientation-based heuristics: The orientation of the canvas-drawn shapes is used to determine the correct solution for the system.

## License

This project is licensed under the GNU General Public License v3.0 or later - see the [LICENSE](LICENSE) file for the full license text.

The GPL requirement stems from the dependency on OGDF (Open Graph Drawing Framework), which is GPL-licensed. For detailed information about third-party dependencies and their licenses, see the [NOTICE](NOTICE) file.


