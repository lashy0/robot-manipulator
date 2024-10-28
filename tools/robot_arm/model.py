"""
Defines a model of a robotic arm using the IKPy library.

This module constructs a kinematic chain for a robotic arm, represented by a series
of 'URDFLink' instances. The arm model includes a base, shoulder, elbow, wrist, and
fixed joints representing the rotational wrist and gripper. Each link is characterized
by its range of motion, origin position, and orientation.

Attributes:
    ROBOT_CHAIN (ikpy.chain.Chain): Defines the robotic arm chain, linking various joints
    and setting their constraints.

    START_ANGLE_POSITION (numpy.ndarray): An array of zeros representing the starting joint
    angles for the arm's initial position.

Notes:
    - The bounds for each joint are limited to (-π/2, π/2) radians, except for the
      fixed joints (wrist_rotational and gripper) which are set to None.
"""

import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink


ROBOT_CHAIN = Chain(
    name='robot_arm',
    links=[
        URDFLink(
            name="base",
            bounds=(-np.pi/2, np.pi/2),
            origin_translation=[0, 0, 0],
            origin_orientation=[0, 0, 0],
            rotation=[0, 0, 1],
        ),
        # The first link (base joint, shoulder)
        URDFLink(
            name="shoulder",
            bounds=(-np.pi/2, np.pi/2),
            origin_translation=[0, 0, 0.02],
            origin_orientation=[0, 0, 0],
            rotation=[0, -1, 0],
        ),
        # The second link (elbow joint)
        URDFLink(
            name="elbow",
            bounds=(-np.pi/2, np.pi/2),
            origin_translation=[0, 0, 0.105],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
        ),
        # The thrid link (wrist joint)
        URDFLink(
            name="wrist",
            bounds=(-np.pi/2, np.pi/2),
            origin_translation=[0, 0, 0.096],
            origin_orientation=[0, 0, 0],
            rotation=[0, 1, 0],
        ),
        # Fourth link (wrist rotational component)
        URDFLink(
            name="wrist_rotational",
            bounds=None,
            origin_translation=[0, 0, 0.065],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type='fixed',
        ),
        # Manipulator (gripper)
        URDFLink(
            name="gripper",
            bounds=None,
            origin_translation=[0, 0, 0.11],
            origin_orientation=[0, 0, 0],
            rotation=None,
            joint_type='fixed',
        )
    ],
    active_links_mask = [True, True, True, True, False, False]
)

# Values in radians
START_ANGLE_POSITION = np.zeros(len(ROBOT_CHAIN.links))


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    from ikpy.utils.plot import init_3d_figure

    fig, ax = init_3d_figure()

    ROBOT_CHAIN.plot(START_ANGLE_POSITION, ax)

    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    ax.set_xlim(-0.5, 0.5)
    ax.set_ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.5)

    plt.show()
