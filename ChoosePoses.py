import rclpy

from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions, TurtleBot4Navigator


def main(args=None):
    rclpy.init(args=args)

    navigator = TurtleBot4Navigator()


    # Set initial pose
    initial_pose = navigator.getPoseStamped([0.1256, -0.0387, 0.0880], TurtleBot4Directions.NORTH)
    navigator.setInitialPose(initial_pose)

    # Wait for Nav2
    navigator.waitUntilNav2Active()


    # Prepare goal pose options
    goal_options = [
        {'name': 'Home',
         'pose': navigator.getPoseStamped([-2.3577, 1.9174, -0.0014], TurtleBot4Directions.EAST)},

        {'name': 'Dining',
         'pose': navigator.getPoseStamped([2.5819, -1.9250, -0.0014], TurtleBot4Directions.EAST)},

        {'name': 'Kitchen',
         'pose': navigator.getPoseStamped([1.7129, 3.4903, 0.0024], TurtleBot4Directions.NORTH)},

        {'name': 'Parlour',
         'pose': navigator.getPoseStamped([-2.1933, 3.9677, -0.0014], TurtleBot4Directions.NORTH_WEST)},


        {'name': 'Exit',
         'pose': None}
    ]

    navigator.info('Welcome to the mail delivery service.')

    while True:
        # Create a list of the goals for display
        options_str = 'Please enter the number corresponding to the desired robot goal position:\n'
        for i in range(len(goal_options)):
            options_str += f'    {i}. {goal_options[i]["name"]}\n'

        # Prompt the user for the goal location
        raw_input = input(f'{options_str}Selection: ')

        selected_index = 0

        # Verify that the value input is a number
        try:
            selected_index = int(raw_input)
        except ValueError:
            navigator.error(f'Invalid goal selection: {raw_input}')
            continue

        # Verify that the user input is within a valid range
        if (selected_index < 0) or (selected_index >= len(goal_options)):
            navigator.error(f'Goal selection out of bounds: {selected_index}')

        # Check for exit
        elif goal_options[selected_index]['name'] == 'Exit':
            break

        else:
            # Navigate to requested position
            navigator.startToPose(goal_options[selected_index]['pose'])

    rclpy.shutdown()


if _name_ == '_main_':
    main()
