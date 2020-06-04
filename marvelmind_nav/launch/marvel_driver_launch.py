# Based on https://github.com/ros2/launch_ros/blob/master/launch_ros/examples/lifecycle_pub_sub_launch.py 

"""Launch a lifecycle marvel_node."""

import os
import sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))  # noqa
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'launch'))  # noqa

import launch
import launch.actions
import launch.events

from launch_ros import get_default_launch_description
import launch_ros.actions
import launch_ros.events
import launch_ros.events.lifecycle

import lifecycle_msgs.msg

def generate_launch_description(argv=sys.argv[1:]):
    """Run lifecycle nodes via launch."""
    ld = launch.LaunchDescription()


    # Prepare the marvel node.
    marvel_node = launch_ros.actions.LifecycleNode(
        node_name='marvelmind_nav', package='marvelmind_nav', node_executable='marvelmind_nav', output='screen')

    # When the marvel reaches the 'inactive' state, make it take the 'activate' transition.
    register_event_handler_for_marvel_reaches_inactive_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=marvel_node, goal_state='inactive',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'marvelmind_nav' reached the 'inactive' state, 'activating'."),
                launch.actions.EmitEvent(event=launch_ros.events.lifecycle.ChangeState(
                    lifecycle_node_matcher=launch.events.matches_action(marvel_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )

    # When the marvel node reaches the 'active' state, log a message 
    register_event_handler_for_marvel_reaches_active_state = launch.actions.RegisterEventHandler(
        launch_ros.event_handlers.OnStateTransition(
            target_lifecycle_node=marvel_node, goal_state='active',
            entities=[
                launch.actions.LogInfo(
                    msg="node 'marvelmind_nav' reached the 'active' state.")
            ],
        )
    )

    # Make the marvel node take the 'configure' transition.
    emit_event_to_request_that_marvel_does_configure_transition = launch.actions.EmitEvent(
        event=launch_ros.events.lifecycle.ChangeState(
            lifecycle_node_matcher=launch.events.matches_action(marvel_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )# Add the actions to the launch description.
    # The order they are added reflects the order in which they will be executed.
    ld.add_action(register_event_handler_for_marvel_reaches_inactive_state)
    ld.add_action(register_event_handler_for_marvel_reaches_active_state)
    ld.add_action(marvel_node)
    ld.add_action(emit_event_to_request_that_marvel_does_configure_transition)

    # print('Starting introspection of launch description...')
    # print('')

    # print(launch.LaunchIntrospector().format_launch_description(ld))

    print('')
    print('Starting launch of launch description...')
    print('')

    # ls = launch.LaunchService(argv=argv, debug=True)
    ls = launch.LaunchService(argv=argv)
    ls.include_launch_description(ld)
    return ld