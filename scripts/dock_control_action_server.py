#!/usr/bin/env python3
"""
Dock Station Control Action Server
Controls the dock station covers with REVOLUTE (hinge) joints using actionlib.
Supports OPEN, CLOSE, and STOP commands with real-time feedback.
Uses dock_msgs/Commands for standardized command enumerations.
"""

import rospy
import actionlib
import threading
from datetime import datetime, timezone
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from dock_msgs.msg import LidControlAction, LidControlGoal, LidControlResult, LidControlFeedback
from dock_msgs.msg import Commands, Lid
from dock_msgs.srv import LidControlTrigger, LidControlTriggerResponse


class DockControlActionServer:
    def __init__(self):
        rospy.init_node('dock_control_action_server', anonymous=False)

        # Publishers for position commands
        self.kapak1_pub = rospy.Publisher(
            '/dock_station/kapak1j_position_controller/command',
            Float64,
            queue_size=10
        )
        self.kapak2_pub = rospy.Publisher(
            '/dock_station/kapak2j_position_controller/command',
            Float64,
            queue_size=10
        )
        
        # Publisher for lid state (for UI)
        self.lid_state_pub = rospy.Publisher(
            '/dock/rx/devices/telemetry/lid',
            Lid,
            queue_size=10,
            latch=True
        )

        # Subscriber for joint states
        self.joint_state_sub = rospy.Subscriber(
            '/dock_station/joint_states',
            JointState,
            self.joint_state_callback
        )

        # State variables
        self.current_state = Lid.STATE_CLOSED
        self.kapak1_position = 0.0
        self.kapak2_position = 0.0
        self.lock = threading.Lock()
        self.stop_requested = False

        # Load configuration parameters
        self.OPEN_POSITION = rospy.get_param('~dock_station/open_position', 3.14)
        self.CLOSED_POSITION = rospy.get_param('~dock_station/closed_position', 0.0)
        self.LEFT_DIRECTION = rospy.get_param('~dock_station/left_direction', 1.0)
        self.RIGHT_DIRECTION = rospy.get_param('~dock_station/right_direction', -1.0)
        self.POSITION_TOLERANCE = rospy.get_param('~dock_station/position_tolerance', 0.05)
        self.TIMEOUT = rospy.get_param('~dock_station/timeout', 30.0)
        self.CONTROL_RATE = rospy.get_param('~dock_station/control_rate', 10)
        self.FEEDBACK_RATE = rospy.get_param('~dock_station/feedback_rate', 5)

        # Action server
        self.action_server = actionlib.SimpleActionServer(
            '/dock_station/lid_control',
            LidControlAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        self.action_server.start()

        # Service server
        self.service = rospy.Service(
            '/dock_station/lid_control_trigger',
            LidControlTrigger,
            self.handle_lid_control_trigger
        )

        rospy.loginfo("Dock Control Action Server initialized")
        rospy.loginfo("Configuration:")
        rospy.loginfo(f"  Open position: {self.OPEN_POSITION} rad")
        rospy.loginfo(f"  Closed position: {self.CLOSED_POSITION} rad")
        rospy.loginfo(f"  Left direction: {self.LEFT_DIRECTION}")
        rospy.loginfo(f"  Right direction: {self.RIGHT_DIRECTION}")
        rospy.loginfo(f"  Position tolerance: {self.POSITION_TOLERANCE} rad")
        rospy.loginfo(f"  Timeout: {self.TIMEOUT}s")
        rospy.loginfo(f"  Control rate: {self.CONTROL_RATE}Hz")
        rospy.loginfo(f"  Feedback rate: {self.FEEDBACK_RATE}Hz")
        rospy.loginfo("Action server available at: /dock_station/lid_control")
        rospy.loginfo("Service available at: /dock_station/lid_control_trigger")

    def get_iso_timestamp(self):
        """Generate ISO8601 timestamp string."""
        return datetime.now(timezone.utc).strftime('%Y-%m-%dT%H:%M:%S.%fZ')

    def handle_lid_control_trigger(self, req):
        """Service handler that triggers action goals without waiting."""
        rospy.loginfo(f"Received trigger: command={req.command}, transaction_id={req.transaction_id}")

        # Validate command
        if req.command not in [Commands.DOCK_LID_OPEN, Commands.DOCK_LID_CLOSE, Commands.DOCK_LID_STOP]:
            return LidControlTriggerResponse(
                success=False,
                message=f"Invalid command: {req.command}. Use 0=CLOSE, 1=OPEN, 2=STOP",
                transaction_id=req.transaction_id,
                ts=self.get_iso_timestamp()
            )

        # Create and send action goal
        goal = LidControlGoal()
        goal.command = req.command
        goal.params = req.params
        goal.transaction_id = req.transaction_id
        goal.ts = req.ts

        # Send goal to action server
        self.action_server.send_goal(goal)

        return LidControlTriggerResponse(
            success=True,
            message="Action goal sent successfully",
            transaction_id=req.transaction_id,
            ts=self.get_iso_timestamp()
        )

    def execute_callback(self, goal):
        """
        Action server callback for handling lid control goals.
        Supports OPEN, CLOSE, and STOP commands.
        """
        rospy.loginfo(f"Received goal: command={goal.command}, transaction_id={goal.transaction_id}")

        # Reset stop flag
        with self.lock:
            self.stop_requested = False

        # Create result message
        result = LidControlResult()
        result.transaction_id = goal.transaction_id
        result.error_code = 0

        # Handle different commands
        if goal.command == Commands.DOCK_LID_OPEN:
            success, lid_msg, message = self.execute_open(goal)
        elif goal.command == Commands.DOCK_LID_CLOSE:
            success, lid_msg, message = self.execute_close(goal)
        elif goal.command == Commands.DOCK_LID_STOP:
            success, lid_msg, message = self.execute_stop(goal)
        else:
            success = False
            lid_msg = Lid(state=self.current_state, percentage=-1, ts=self.get_iso_timestamp())
            message = f"Invalid command: {goal.command}"
            result.error_code = 1
            rospy.logerr(message)

        result.success = success
        result.lid = lid_msg
        result.message = message

        # Check if goal was preempted
        if self.action_server.is_preempt_requested():
            self.action_server.set_preempted(result)
        elif result.success:
            self.action_server.set_succeeded(result)
        else:
            self.action_server.set_aborted(result)

        rospy.loginfo(f"Action completed: {result.message}")

    def execute_open(self, goal):
        """Execute OPEN command."""
        rospy.loginfo("Opening dock station covers...")

        with self.lock:
            self.current_state = Lid.STATE_OPENING
        
        # Publish opening state to UI
        opening_msg = Lid(state=Lid.STATE_OPENING, percentage=0, ts=self.get_iso_timestamp())
        self.lid_state_pub.publish(opening_msg)

        # Send position commands with direction multipliers
        kapak1_target = self.OPEN_POSITION * self.LEFT_DIRECTION
        kapak2_target = self.OPEN_POSITION * self.RIGHT_DIRECTION

        self.kapak1_pub.publish(Float64(kapak1_target))
        self.kapak2_pub.publish(Float64(kapak2_target))

        # Wait for completion with feedback
        success = self.wait_for_position_with_feedback(
            kapak1_target,
            kapak2_target,
            Lid.STATE_OPENING,
            goal.transaction_id
        )

        with self.lock:
            if self.stop_requested:
                self.current_state = Lid.STATE_STOPPED
                lid_msg = Lid(state=Lid.STATE_STOPPED, percentage=-1, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return True, lid_msg, "Lid opening stopped"
            elif success:
                self.current_state = Lid.STATE_OPEN
                lid_msg = Lid(state=Lid.STATE_OPEN, percentage=100, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return True, lid_msg, "Lid opened successfully"
            else:
                self.current_state = Lid.STATE_OPEN
                lid_msg = Lid(state=Lid.STATE_OPEN, percentage=-1, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return False, lid_msg, "Timeout but may have reached position"

    def execute_close(self, goal):
        """Execute CLOSE command."""
        rospy.loginfo("Closing dock station covers...")

        with self.lock:
            self.current_state = Lid.STATE_CLOSING
        
        # Publish closing state to UI
        closing_msg = Lid(state=Lid.STATE_CLOSING, percentage=100, ts=self.get_iso_timestamp())
        self.lid_state_pub.publish(closing_msg)

        # Send position commands with direction multipliers
        kapak1_target = self.CLOSED_POSITION * self.LEFT_DIRECTION
        kapak2_target = self.CLOSED_POSITION * self.RIGHT_DIRECTION

        self.kapak1_pub.publish(Float64(kapak1_target))
        self.kapak2_pub.publish(Float64(kapak2_target))

        # Wait for completion with feedback
        success = self.wait_for_position_with_feedback(
            kapak1_target,
            kapak2_target,
            Lid.STATE_CLOSING,
            goal.transaction_id
        )

        with self.lock:
            if self.stop_requested:
                self.current_state = Lid.STATE_STOPPED
                lid_msg = Lid(state=Lid.STATE_STOPPED, percentage=-1, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return True, lid_msg, "Lid closing stopped"
            elif success:
                self.current_state = Lid.STATE_CLOSED
                lid_msg = Lid(state=Lid.STATE_CLOSED, percentage=0, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return True, lid_msg, "Lid closed successfully"
            else:
                self.current_state = Lid.STATE_CLOSED
                lid_msg = Lid(state=Lid.STATE_CLOSED, percentage=-1, ts=self.get_iso_timestamp())
                self.lid_state_pub.publish(lid_msg)
                return False, lid_msg, "Timeout but may have reached position"

    def execute_stop(self, goal):
        """Execute STOP command - stops current motion."""
        rospy.loginfo("Stopping lid motion...")

        with self.lock:
            self.stop_requested = True
            current_kapak1 = self.kapak1_position
            current_kapak2 = self.kapak2_position

        # Send current position as target to stop motion
        self.kapak1_pub.publish(Float64(current_kapak1))
        self.kapak2_pub.publish(Float64(current_kapak2))

        # Small delay to ensure command is sent
        rospy.sleep(0.1)

        with self.lock:
            self.current_state = Lid.STATE_STOPPED
        
        lid_msg = Lid(state=Lid.STATE_STOPPED, percentage=-1, ts=self.get_iso_timestamp())
        self.lid_state_pub.publish(lid_msg)
        return True, lid_msg, "Lid motion stopped"

    def wait_for_position_with_feedback(self, target_kapak1, target_kapak2, state, transaction_id):
        """
        Wait until both joints reach target positions (within tolerance).
        Publishes feedback at regular intervals.
        Returns True if successful, False if timeout or preempted.
        """
        rate = rospy.Rate(self.CONTROL_RATE)
        feedback_rate_counter = 0
        feedback_interval = self.CONTROL_RATE // self.FEEDBACK_RATE
        start_time = rospy.Time.now()

        # Get initial positions for percentage calculation
        with self.lock:
            initial_kapak1 = self.kapak1_position
            initial_kapak2 = self.kapak2_position

        distance_kapak1 = abs(target_kapak1 - initial_kapak1)
        distance_kapak2 = abs(target_kapak2 - initial_kapak2)
        total_distance = max(distance_kapak1, distance_kapak2)

        while not rospy.is_shutdown():
            # Check for preemption (action cancelled)
            if self.action_server.is_preempt_requested():
                rospy.loginfo("Goal preempted")
                self.action_server.set_preempted()
                with self.lock:
                    self.stop_requested = True
                return False

            # Check for stop request
            with self.lock:
                if self.stop_requested:
                    return False

            # Check timeout
            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed > self.TIMEOUT:
                rospy.logwarn(f"Timeout after {elapsed:.1f}s")
                return False

            # Get current positions
            with self.lock:
                current_kapak1 = self.kapak1_position
                current_kapak2 = self.kapak2_position

            # Calculate errors
            kapak1_error = abs(current_kapak1 - target_kapak1)
            kapak2_error = abs(current_kapak2 - target_kapak2)

            # Check if target reached
            if kapak1_error < self.POSITION_TOLERANCE and kapak2_error < self.POSITION_TOLERANCE:
                rospy.loginfo(f"Target reached in {elapsed:.1f}s")

                # Send final feedback
                feedback = LidControlFeedback()
                feedback.lid = Lid(state=state, percentage=100, ts=self.get_iso_timestamp())
                feedback.message = "Target reached"
                feedback.transaction_id = transaction_id
                self.action_server.publish_feedback(feedback)

                return True

            # Publish feedback at specified rate
            feedback_rate_counter += 1
            if feedback_rate_counter >= feedback_interval:
                feedback_rate_counter = 0

                # Calculate progress percentage
                if total_distance > 0:
                    remaining_kapak1 = abs(current_kapak1 - target_kapak1)
                    remaining_kapak2 = abs(current_kapak2 - target_kapak2)
                    max_remaining = max(remaining_kapak1, remaining_kapak2)
                    progress = 100.0 * (1.0 - max_remaining / total_distance)
                    percentage = int(max(0, min(100, progress)))
                else:
                    percentage = 100

                lid_msg = Lid(state=state, percentage=percentage, ts=self.get_iso_timestamp())
                feedback = LidControlFeedback()
                feedback.lid = lid_msg
                feedback.message = f"Progress: {percentage}%"
                feedback.transaction_id = transaction_id
                self.action_server.publish_feedback(feedback)
                
                # Also publish to UI topic
                self.lid_state_pub.publish(lid_msg)

            rate.sleep()

        return False

    def joint_state_callback(self, msg):
        """Update joint positions from joint_states topic"""
        try:
            with self.lock:
                if 'kapak1j' in msg.name:
                    kapak1_idx = msg.name.index('kapak1j')
                    self.kapak1_position = msg.position[kapak1_idx]

                if 'kapak2j' in msg.name:
                    kapak2_idx = msg.name.index('kapak2j')
                    self.kapak2_position = msg.position[kapak2_idx]

        except (ValueError, IndexError) as e:
            rospy.logwarn_throttle(5.0, f"Joint state processing error: {e}")

    def run(self):
        """Main loop - keeps node alive"""
        rospy.spin()


if __name__ == '__main__':
    try:
        server = DockControlActionServer()
        server.run()
    except rospy.ROSInterruptException:
        pass
