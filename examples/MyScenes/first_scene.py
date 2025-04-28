# Required import for python
import Sofa
import SofaRuntime


def main():
    # Make sure to load all necessary libraries
    SofaRuntime.importPlugin("Sofa.Component.StateContainer")

    # Call the above function to create the scene graph
    root = Sofa.Core.Node("root")
    createScene(root)


# Function called when the scene graph is being created
def createScene(root):
    # Needed to use components [FreeMotionAnimationLoop]
    root.addObject('RequiredPlugin', name='Sofa.Component.AnimationLoop')
    # Needed to use components [BVHNarrowPhase,BruteForceBroadPhase]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Collision.Detection.Algorithm')
    # Needed to use components [LocalMinDistance]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Collision.Detection.Intersection')
    # Needed to use components [LineCollisionModel,PointCollisionModel,TriangleCollisionModel]
    root.addObject('RequiredPlugin', name='Sofa.Component.Collision.Geometry')
    # Needed to use components [RuleBasedContactManager]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Collision.Response.Contact')
    # Needed to use components [UncoupledConstraintCorrection]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Constraint.Lagrangian.Correction')
    # Needed to use components [GenericConstraintSolver]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Constraint.Lagrangian.Solver')
    # Needed to use components [MeshOBJLoader]
    root.addObject('RequiredPlugin', name='Sofa.Component.IO.Mesh')
    # Needed to use components [CGLinearSolver]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.LinearSolver.Iterative')
    # Needed to use components [RigidMapping]
    root.addObject('RequiredPlugin', name='Sofa.Component.Mapping.NonLinear')
    # Needed to use components [UniformMass]
    root.addObject('RequiredPlugin', name='Sofa.Component.Mass')
    # Needed to use components [EulerImplicitSolver]
    root.addObject('RequiredPlugin', name='Sofa.Component.ODESolver.Backward')
    # Needed to use components [MechanicalObject]
    root.addObject('RequiredPlugin', name='Sofa.Component.StateContainer')
    # Needed to use components [MeshTopology]
    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Topology.Container.Constant')
    # Needed to use components [VisualGrid]
    root.addObject('RequiredPlugin', name='Sofa.Component.Visual')
    # Needed to use components [OglModel,OglSceneFrame]
    root.addObject('RequiredPlugin', name='Sofa.GL.Component.Rendering3D')

    root.addObject('RequiredPlugin',
                   name='Sofa.Component.Collision.Detection.Algorithm')

    # Physics
    # Gravity force
    root.gravity = [0.0, -9.81, 0.0]
    # Time step of the simulation
    root.dt = 0.01

    # Mechanical model to use default physics for future elements
    totalMass = 1.0
    volume = 1.0
    inertiaMatrix = [1., 0., 0., 0., 1., 0., 0., 1.]

    # Collision pipeline
    # WHAT o DefaultCollision não é mais usado. Para isso precisamos do CollisionPipeline mesmo, igual na linha abaixo
    root.addObject(
        'CollisionPipeline')
    root.addObject('FreeMotionAnimationLoop')
    root.addObject('GenericConstraintSolver',
                   tolerance="1e-6", maxIterations="1000")
    root.addObject('BruteForceBroadPhase')
    root.addObject('BVHNarrowPhase')
    root.addObject('RuleBasedContactManager', responseParams="mu=" +
                   str(0.0), name='Response', response='FrictionContactConstraint')
    root.addObject('LocalMinDistance', alarmDistance=10,
                   contactDistance=5, angleCone=0.01)

    # Grid
    root.addObject("VisualGrid", nbSubdiv=10, size=1000)

    confignode = root.addChild("Config")

    # Visualization of vectors
    confignode.addObject('OglSceneFrame', style="Arrows", alignment="TopRight")

    # Sphere
    # Mechanical Object
    sphere = root.addChild("sphere")
    # Defines the system to be solved at each time step of the simulation
    sphere.addObject('EulerImplicitSolver', name='odesolver')
    # Solving method (solves the equations governing the model at each time step, and updates the MechanicalObject)
    sphere.addObject('CGLinearSolver', name='Solver',
                     iterations=25, tolerance=1e-05, threshold=1e-05)
    sphere.addObject('MechanicalObject', name="mstate", template="Rigid3",
                     translation2=[0., 0., 0.], rotation2=[0., 0., 0.],
                     showObjectScale=50)
    sphere.addObject('UniformMass', name='mass', vertexMass=[
                     totalMass, volume, inertiaMatrix[:]])
    # Object in charge of computing the constraint forces of the sphere
    sphere.addObject('UncoupledConstraintCorrection')

    # Visual Object
    sphereVisu = sphere.addChild("VisualModel")
    sphereVisu.loader = sphereVisu.addObject('MeshOBJLoader',
                                             name='loader',
                                             filename="mesh/ball.obj")
    sphereVisu.addObject('OglModel', name="model", src="@loader",
                         scale3d=[100]*3, color=[0., 1., 0.], updateNormals=False)
    sphereVisu.addObject('RigidMapping')

    # Collision subnode for the sphere
    sphereCollision = sphere.addChild('collision')
    sphereCollision.addObject('MeshOBJLoader', name="loader",
                              filename="mesh/ball.obj", triangulate="true", scale=45.0)
    sphereCollision.addObject('MeshTopology', src="@loader")
    sphereCollision.addObject('MechanicalObject')
    sphereCollision.addObject('TriangleCollisionModel')
    sphereCollision.addObject('LineCollisionModel')
    sphereCollision.addObject('PointCollisionModel')
    sphereCollision.addObject('RigidMapping')

    # Floor Object
    floor = root.addChild('floor')

    floor.addObject('MechanicalObject', name='mstate', template='Rigid3', translation2=[
                    0.0, -300.0, 0.0], rotation2=[0., 0., 0.], showObjectScale=5.0)
    floor.addObject('UniformMass', name="mass", vertexMass=[
                    totalMass, volume, inertiaMatrix[:]])

    # Collision subnode for the floor
    floorCollis = floor.addChild('collision')
    floorCollis.addObject('MeshOBJLoader', name="loader",
                          filename="mesh/floor.obj", triangulate="true", scale=5.0)
    floorCollis.addObject('MeshTopology', src="@loader")
    floorCollis.addObject('MechanicalObject')
    floorCollis.addObject('TriangleCollisionModel',
                          moving=False, simulated=False)
    floorCollis.addObject('LineCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('PointCollisionModel', moving=False, simulated=False)
    floorCollis.addObject('RigidMapping')

    # Visualization subnode for the floor
    floorVisu = floor.addChild('VisualModel')
    floorVisu.loader = floorVisu.addObject(
        'MeshOBJLoader', name='loader', filename='mesh/floor.obj')
    floorVisu.addObject('OglModel', name='model', src='@loader',
                        scale3d=[5.0]*3, color=[1., 1., 0.], updateNormals=False)
    floorVisu.addObject('RigidMapping')

    return root


# Function used only if this script is called from a python environment
if __name__ == '__main__':
    main()
