class ControlPacket:
    """
    Data packet that is serialized into json and transmitted to the vehicle
    """
    def __init__(self):
        self.left_speed = 0.0
        self.right_speed = 0.0
        