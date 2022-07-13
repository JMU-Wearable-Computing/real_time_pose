
import math
import socket
import numpy as np
from feedback.feedback_device import FeedbackDevice


class HapticGlove(FeedbackDevice):
    '''
    Implementation for Haptic Glove device
    '''

    # array of motor positions (around hand) in 3D space
    MOTORS = np.array([np.array([0,0,1]), np.array([0,0,-1]), np.array([0,-1,0]), np.array([0,1,0])])

    # default time out in seconds
    TIMEOUT = 10

    # default message to set motors to lowest intensity (150)
    MINIMUM_INTENSITY_MESSAGE = "/150/150/150/150"

    def __init__(self, tcp_ip: str, tcp_port: int, direction: str = "pull") -> None:
        '''
        Constructor for Haptic Glove

        Args:
            tcp_ip: IP Address for glove
            tcp_port: TCP Port for Glove
            direction: Direction that feedback should be provided (push/pull). Has no effect at present.
        '''
        super().__init__()
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.settimeout(self.TIMEOUT)
        self.tcp_ip = tcp_ip
        self.tcp_port = tcp_port
        self.direction = direction            
    
    def connect(self) -> None:
        self.socket.connect((self.tcp_ip, self.tcp_port))
        self.socket.settimeout(None)

    def disconnect(self) -> None:
        self.socket.close()
    
    def send_push_feedback(self, message: np.array) -> None:
        '''
        Sends PUSH feedback to glove. Stub, currently not implemented.

        Args:
            message:

        Returns:

        '''
        for _ in range(0,10):
            self.socket.send(f'{message}\n'.encode('ascii'))

    def send_pull_feedback(self, current_pt: np.array, goal_pt: np.array):
        '''
        Sends PULL feedback to the glove.

        Args:
            current_pt: current position on th screen
            goal_pt: goal position

        Returns: None

        '''

        # given PULL approach, find intensity for each motor
        intensity = self.find_intensity_array(current_pt, goal_pt, self.MOTORS)

        # generate message for transmission to glove
        message = self.make_message(intensity)

        # send the message 10x times (unsure why we repeat...)
        # render message as ASCII
        for _ in range(0,10):
            self.socket.send(f'{message}\n'.encode('ascii'))
    
    def stop_feedback(self) -> None:
        '''
        Feedback is complete. Silence all motors.

        Returns:
            None
        '''
        for _ in range(0,10):
            self.socket.send(f'{self.MINIMUM_INTENSITY_MESSAGE}\n'.encode('ascii'))

    def make_message(self, vect) -> str:
        '''
        Generate a message for the glove based upon intensity vector. Only 4 tactors
        are supported (hard coded).

        Args:
            vect: Vector of intensities for each motor.

        Returns: ASCII string for Glove

        '''
        return f'/{vect[1]}/{vect[0]}/{vect[2]}/{vect[3]}'

    def find_distance(self, vector1, vector2, normalized=False):
        '''
        Finds the distance between two vectors

        Args:
            vector1: Vector A
            vector2: Vector B
            normalized: Whether the two vectors should be normalized to unit vectors

        Returns: Distance between the two vectors

        '''
        if normalized:
            vector1 = vector1 / np.linalg.norm(vector1)
            vector2 = vector2 / np.linalg.norm(vector2)
        diff = vector1 - vector2
        distance = np.linalg.norm(diff)
        return distance

    def map_to_range(self, x, in_min, in_max, out_min, out_max, bounded=False):
        '''
        Mapping function x:{in_min,in_max} --> {out_min,out_max}

        Args:
            x: Target
            in_min: Lower bound of input domain
            in_max: Upper bound of input domain
            out_min: Lower bound of output range
            out_max: Upper bound of output range
            bounded: True if output should be bound to output range. False otherwise.

        Returns:
            Mapped value x from Domain into Range
        '''
        output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if bounded:
            if output < out_min:
                output = out_min
            if output > out_max:
                output = out_max
        return output

    def reverse_map_to_range(self, x, in_min, in_max, out_min, out_max, bounded=False):
        output = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
        if bounded:
            if output > out_min:
                output = out_min
            if output < out_max:
                output = out_max
        return output

    def find_intensity_array(self, current_pos, goal_pos, motor_positions) -> np.array:
        '''
        Determines the desired intensities of each motor based upon the current/goal positions
        and the motor locations

        Args:
            current_pos: Current position of the hand
            goal_pos: Goal/target positions
            motor_positions: Location of each motor

        Returns:
            Array of motor intensities

        '''

        # find vector between goal and current positions
        U = goal_pos - current_pos
        #print(f'Displacement vector: {U}')

        # normalize U to find distance to target
        D = np.linalg.norm(U)
        #print(f'Distance from goal: {D}')

        # given those distances, map range (0 - 0.6) into intensities for motors (155 - 255)
        I = self.map_to_range(D, 0, 0.6, 150, 255,  bounded=True)
        #print(f'Distance adjusted to range: {I}')

        motor_distance = [0.0,0.0,0.0,0.0]
        mapped = [0.0,0.0,0.0,0.0]

        # using the PULL mechanic (reverse_map) determine true intensity of each motor
        # many magic constants are present. Need to document.
        for i in range(0, len(motor_positions)):
            motor_distance[i] = self.find_distance(U, motor_positions[i], normalized=True)
            mapped[i] = self.reverse_map_to_range(motor_distance[i], 0.0, math.sqrt(2), 1, .59, bounded=True)

        # coefficients to adjust motor intensities based upon approach
        mapped = np.array(mapped)

        #print(f'Motor distances : {motor_distance}')
        #print(f'Motor intensity proportions: {mapped}')

        # resulting intensity for each motor
        intensity = np.array(I * mapped).astype(int)
        #print(f'Motor intensity array: {intensity}')
        return intensity
