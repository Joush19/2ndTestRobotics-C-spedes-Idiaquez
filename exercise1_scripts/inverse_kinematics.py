import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point
import numpy as np


class InverseKinematics(Node):

    def __init__(self):
        super().__init__('inverse_kinematics')
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        self.target_sub = self.create_subscription(Point, 'target_position',
                                                   self.target_callback, 10)

        # Articulaciones: q1, q2, q3, q6
        self.q = np.array([-0.22, 0.7, 0.03, 0.0])

        # Longitudes de eslabones
        self.l1 = -1.0
        self.l2 = 0.2
        self.l3 = 1.0
        self.l4 = 0.15  # Nuevo eslabón añadido al final (ej. soporte pinza)

        self.timer = self.create_timer(0.1, self.update_joints)
        self.target_pos = np.array([1.2, 0.5, 0.0])  # Posición objetivo por defecto

        self.step_size = 0.05
        self.max_iterations = 100
        self.tolerance = 0.01
        self.damping_factor = 0.1

    def forward_kinematics(self, q):
        q1, q2, q3, q6 = q

        # Incluye l4 como una extensión adicional alineada con la dirección de l3
        L = self.l3 + self.l4  # Longitud efectiva del último brazo
        x = L * np.sin(q1) * np.sin(q3) + L * np.cos(q1) * np.cos(q2) * np.cos(q3) + self.l2 * np.cos(q1) * np.cos(q2)
        y = L * np.sin(q1) * np.cos(q2) * np.cos(q3) + self.l2 * np.sin(q1) * np.cos(q2) - L * np.sin(q3) * np.cos(q1)
        z = L * np.sin(q2) * np.cos(q3) + self.l2 * np.sin(q2) + self.l1
        return np.array([x, y, z])

    def jacobian(self, q):
        q1, q2, q3, _ = q
        L = self.l3 + self.l4

        j11 = -L * np.sin(q1) * np.cos(q2) * np.cos(q3) - self.l2 * np.sin(q1) * np.cos(q2) + L * np.sin(q3) * np.cos(q1)
        j12 = -L * np.sin(q2) * np.cos(q1) * np.cos(q3) - self.l2 * np.sin(q2) * np.cos(q1)
        j13 = L * np.sin(q1) * np.cos(q3) - L * np.sin(q3) * np.cos(q1) * np.cos(q2)
        j21 = L * np.sin(q1) * np.sin(q3) + L * np.cos(q1) * np.cos(q2) * np.cos(q3) + self.l2 * np.cos(q1) * np.cos(q2)
        j22 = -L * np.sin(q1) * np.sin(q2) * np.cos(q3) - self.l2 * np.sin(q1) * np.sin(q2)
        j23 = -L * np.sin(q1) * np.sin(q3) * np.cos(q2) - L * np.cos(q1) * np.cos(q3)
        j31 = 0
        j32 = L * np.cos(q2) * np.cos(q3) + self.l2 * np.cos(q2)
        j33 = -L * np.sin(q2) * np.sin(q3)

        return np.array([[j11, j12, j13], [j21, j22, j23], [j31, j32, j33]])

    def target_callback(self, msg):
        self.target_pos = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f"New target received: [{msg.x}, {msg.y}, {msg.z}]")

    def update_joints(self):
        current_pos = self.forward_kinematics(self.q)
        error = self.target_pos - current_pos
        error_norm = np.linalg.norm(error)

        if error_norm > self.tolerance:
            J = self.jacobian(self.q)
            JtJ = J.T @ J
            damping = self.damping_factor * np.eye(JtJ.shape[0])
            J_dls = np.linalg.solve(JtJ + damping, J.T) @ error
            self.q[:3] += J_dls * self.step_size  # Solo actualizamos q1, q2, q3

        # Estado conjunto
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['q1', 'q2', 'q3', 'q6', 'q4']
        q4 = 0.01  # valor estático para la pinza
        msg.position = self.q.tolist() + [q4]
        self.joint_pub.publish(msg)


def main():
    rclpy.init()
    node = InverseKinematics()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

