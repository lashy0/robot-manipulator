import numpy as np
from ikpy.chain import Chain
from ikpy.link import URDFLink


ROBOT_CHAIN = Chain(name='robot_arm', links=[
    URDFLink(
        name="base",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0],
        origin_orientation=[0, 0, 0],
        rotation=[0, 0, 1],
    ),
    # Первое звено (соединение с базой, плечо)
    URDFLink(
        name="shoulder",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.02],
        origin_orientation=[0, 0, 0],
        rotation=[0, -1, 0],
    ),
    # Второе звено (локоть)
    URDFLink(
        name="elbow",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.105],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    # Третье звено (запястье)
    URDFLink(
        name="wrist",
        bounds=(-np.pi/2, np.pi/2),
        origin_translation=[0, 0, 0.096],
        origin_orientation=[0, 0, 0],
        rotation=[0, 1, 0],
    ),
    # Четвертое звено (крутит захват)
    URDFLink(
        name="wrist_rotational",
        bounds=None,
        origin_translation=[0, 0, 0.065],
        origin_orientation=[0, 0, 0],
        rotation=None,
        joint_type='fixed',
    ),
    # Манипулятор (захват)
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

START_ANGLE_POSITION = [0] * len(ROBOT_CHAIN.links)


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
