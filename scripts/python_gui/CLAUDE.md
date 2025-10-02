# Python GUI for 2D Geometric Constraint Solver

## Overview
This directory contains a GTK3+ based Python GUI application for the 2D Geometric Constraint Solver. The GUI provides an interactive interface for drawing geometric shapes and visualizing constraint solving algorithms.

## Architecture

### Main Components

#### Application Layer
- **`gtkapplication.py`** - Main application entry point using GTK Application framework
  - `GeometricConstraintSolverApp` class extends `Gtk.Application`
  - Handles application lifecycle, CSS theme loading, and keyboard shortcuts
  - Entry point: `main()` function

#### Main Window
- **`GcsGui/mainwindow.py`** - Main application window with tabbed interface
  - `MainWindow` class extends `Gtk.ApplicationWindow`
  - Creates two main tabs: "Draw Model" and "Graph Rendering"
  - Integrates drawing area and constraint system components
  - Window size: 1920x1200 pixels

#### Model Layer
- **`GcsGui/model/drawmodel.py`** - Core drawing model and geometric shapes
  - `ShapeManager` - Container for drawable shapes
  - `Drawable` - Abstract base class for all drawable objects
  - Shape implementations: `Point`, `Line`, `Circle`
  - Canvas coordinate system: `CanvasCoord` class
  - Cairo-based rendering for all shapes

- **`GcsGui/model/gcs.py`** - Geometric Constraint System integration
  - `GeometricConstraintSystem` - Main constraint solver interface
  - References `ShapeManager` for shape data

#### View Layer
- **`GcsGui/view/drawmodelview.py`** - Interactive drawing canvas
  - `DrawingCanvasWidget` - Main drawing area with Cairo rendering
  - `DrawingLayout` - UI layout with controls and canvas
  - Drawing modes: Point, Line, Circle
  - Mouse interaction: Left (draw), Right (select), Middle (add constraints)
  - Grid-based canvas (40px grid)
  - Shape selection and constraint dialog system

- **`GcsGui/view/gcsview.py`** - Graph visualization
  - Uses `graph-tool` library for graph rendering
  - `own_small_example()` - Creates sample graph with 3 vertices and edges
  - SFDP layout algorithm for graph positioning

#### Common Components
- **`GcsGui/common/commondatastructs.py`** - Shared data structures
  - `RgbColour` - RGB color representation (floats 0.0-1.0)
  - `MouseButtonGtkId` - GTK mouse button constants

#### Themes
- **`themes/custom.css`** - Paper-like visual theme
  - Clean, textured appearance reminiscent of paper
  - Custom styling for all GTK widgets
  - Support for both light interface elements

## Key Features

### Drawing Capabilities
- **Point Drawing**: Click to place points with configurable radius
- **Line Drawing**: Two-click line creation between points
- **Circle Drawing**: Click-and-drag circle creation with dynamic radius
- **Shape Selection**: Right-click to select shapes (highlights in red)
- **Constraint Addition**: Middle-click on two selected shapes to add constraints
- **Undo Functionality**: Remove last drawn shape
- **Canvas Clearing**: Clear all shapes at once

### UI Controls
- **Drawing Method Selector**: ComboBox to choose Point/Line/Circle mode
- **Color Controls**: RGB sliders (0.0-1.0 range) for shape colors
- **Line Width**: Adjustable line width (1-30 pixels)
- **Grid Display**: 40px grid for alignment assistance

### Technical Implementation
- **Event Handling**: Mouse press/release/motion events for interactive drawing
- **Hit Testing**: Geometric algorithms for shape selection
- **Cairo Graphics**: Hardware-accelerated 2D graphics rendering
- **GTK3+ Widgets**: Native Linux desktop integration

## Usage Patterns

### Running the Application
```bash
cd scripts/python_gui
python3 gtkapplication.py
```

### Drawing Workflow
1. Select drawing method from dropdown
2. Adjust color and line width as needed
3. Draw shapes on canvas using appropriate mouse actions
4. Select shapes for constraint application
5. Add constraints between selected shapes

## Dependencies
- **GTK3+**: GUI framework (gi.repository)
- **Cairo**: 2D graphics library
- **graph-tool**: Graph visualization library
- **Python 3.6+**: Core language support

## Development Notes
- The GUI serves as a research proof-of-concept
- Shape lifetime is bound to MainWindow for simplicity
- CSS theme loading includes error handling
- All coordinates use float precision for accuracy
- Shape hit testing uses geometric distance calculations