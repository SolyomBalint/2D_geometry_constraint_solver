# 2D Geometric Constraint Solver

This project was created  at the Department of Control Engineering and Information Technology, Faculty of Electrical Engineering and Informatics, Budapest University of Technology and Economics (BME VIK IIT) and it was supported by the Pro Progressio Initiative with the Shapr3D scholarship.

Its main goal is to explore 2D geometric constraint solving, with a particular focus on decomposition algorithms. The project serves both as a proof of concept and as a starting point for further development and research.

## Main Features

The project includes a basic GTK4-based 2D modeller for creating points and lines with distance and angle constraints. Models can be saved to and loaded from JSON files.

The graphical interface allows users to:
- interactively solve constraint systems,
- inspect generated decompositions,
- follow the step-by-step recombination process for top-down decompositions.

## Decomposition Algorithms

Constraint systems are represented as constraint graphs. The project currently works with classical top-down and bottom-up decomposition approaches, including:

- *S-tree / deficit-based top-down decomposition*
- *Bottom-up decomposition using bipartite cluster graphs*

## Solving Strategies

Although the main emphasis of the project is on decomposition algorithms, it also includes a basic solver to demonstrate how a complete solving workflow can operate.

An iterative numerical solver based on the Newton-Raphson method is used to compute possible solutions. Several heuristics are also applied to help select not just any valid solution, but one that better matches the user's intended result.

At the moment, the solver is still fairly limited and may produce degenerate solutions. In addition, for top-down decomposition, only a clean post-order traversal can currently be solved. This is an implementation limitation rather than a theoretical one.

# Demo

<https://github.com/user-attachments/assets/95b050c4-c6c2-4102-bcd2-b91fe33dab3e>

## License

This project is licensed under the GNU General Public License v3.0 or later. See the [LICENSE](LICENSE) file for the full license text.

The GPL requirement comes from the project's dependency on OGDF (Open Graph Drawing Framework), which is licensed under the GPL. For more information about third-party dependencies and their licenses, see the [NOTICE](NOTICE) file.
