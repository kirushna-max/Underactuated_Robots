import time
import matplotlib.pyplot as plt
import mpld3
import numpy as np
from IPython.display import Markdown, display
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Expression,
    LeafSystem,
    LogVectorOutput,
    MakeVectorVariable,
    MeshcatVisualizer,
    MultibodyPlant,
    Parser,
    RigidTransform_,
    Simulator,
    SpatialInertia_,
    StartMeshcat,
    ToLatex,
    UnitInertia_,
)

from underactuated import ConfigureParser, ManipulatorDynamics, running_as_notebook

if running_as_notebook:
    mpld3.enable_notebook()

# Start the visualizer (run this cell only once, each instance consumes a port)
meshcat = StartMeshcat()

def double_pendulum_demo():
    # Set up a block diagram with the robot (dynamics) and a visualization block.
    builder = DiagramBuilder()
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

    # Load the double pendulum from Universal Robot Description Format
    parser = Parser(plant)
    ConfigureParser(parser)
    parser.AddModelsFromUrl("package://underactuated/models/double_pendulum.urdf")
    plant.Finalize()

    builder.ExportInput(plant.get_actuation_input_port())
    MeshcatVisualizer.AddToBuilder(builder, scene_graph, meshcat)
    meshcat.Set2dRenderMode(xmin=-2.8, xmax=2.8, ymin=-2.8, ymax=2.8)

    logger = LogVectorOutput(plant.get_state_output_port(), builder)

    diagram = builder.Build()

    # Set up a simulator to run this diagram
    simulator = Simulator(diagram)

    if running_as_notebook:
        simulator.set_target_realtime_rate(1.0)

    # Set the initial conditions
    context = simulator.get_mutable_context()
    context.SetContinuousState(
        [1.0, 1.0, 0.0, 0.0]
    )  # (theta1, theta2, theta1dot, theta2dot)
    diagram.get_input_port(0).FixValue(context, [0.0, 0.0])  # Zero input torques

    # Simulate
    simulator.AdvanceTo(10.0)

    # Plot the results
    plt.figure()
    fields = ["shoulder", "elbow"]
    log = logger.FindLog(context)
    for i in range(2):
        plt.subplot(2, 1, i + 1)
        plt.plot(log.sample_times(), log.data()[(i, i + 2), :].transpose())
        plt.legend(["position", "velocity"])
        plt.xlabel("t")
        plt.ylabel(fields[i])
        plt.grid(True)
    display(mpld3.display())


double_pendulum_demo()

print("Simulation finished. Press Ctrl+C to stop the Meshcat server.")
while True:
    time.sleep(1)
