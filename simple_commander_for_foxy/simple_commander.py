
import rclpy
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
#author: emiraykin



# This script only can be used for adding navigation goals 

# For setting initial pose you can have a look here:
# https://answers.ros.org/question/392682/how-to-use-nav2_simple_commander-in-foxy/


def main():
    rclpy.init()

    # Create a ROS 2 node
    node = rclpy.create_node('simple_commander')

    # Create an action client called "navigate_to_pose" with action definition "NavigateToPose"
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose')

    # Wait until the action server has started up and started listening for goals
    while not client.wait_for_server(timeout_sec=3.0):
        print('Action server not available!')
        #rclpy.shutdown()
        #return

    # Define a list of goal coordinates
    goal_coordinates = [
        (2.0, 0.0, 0.0),  # x, y, theta for Goal 1
        (0.0, 2.0, 1.57),  # x, y, theta for Goal 2
        (-2.0, -0.5, 0.0),  # x, y, theta for Goal 3
    ]

    # Loop over the goal coordinates and send goals one by one
    #while True:
    for goal_x, goal_y, goal_theta in goal_coordinates:
        # Create a goal message with the PoseStamped message
        goal_msg = NavigateToPose.Goal()

        # Set the goal target pose
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = goal_x
        goal_msg.pose.pose.position.y = goal_y
        goal_msg.pose.pose.orientation.z = goal_theta

        # Send the goal to the action server
        goal_handle_future = client.send_goal_async(goal_msg)

        #print("id: " + str(i))

        # Wait for the goal to complete
        while rclpy.ok():
            rclpy.spin_once(node)
            if goal_handle_future.done():
                goal_handle = goal_handle_future.result()
                if goal_handle.accepted:
                    print('Goal accepted.')
                    result_future = goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(node, result_future)
                    result = result_future.result().result
                    if result!=False:
                        print('Goal execution done!')
                    else:
                        print('Goal execution failed!')
                    break
                else:
                    print('Goal rejected.')
                    #break #単純に次のポイントに進もうとすると別のソフトウェアのつなぎで断続してrejectedになる


    print("Task Competed!")
    # Clean up resources
    rclpy.shutdown()

if __name__ == '__main__':
    main()

