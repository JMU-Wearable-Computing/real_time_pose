
import sys
import numpy as np
from data_logging.video_logger import VideoLogger
from frame_input.frame_input import FrameInput
from pose_detection.computer_vision.cv_model.blazepose import Blazepose
from pose_detection.computer_vision.cv_model.cv_model import CVModel
from pose_detection.pose_detection import PoseDetection
import cv2

class ComputerVision(PoseDetection):
    
    def __init__(self, queue, cv_model_name: str, frame_input: FrameInput, **kwargs) -> None:
        super().__init__(queue)
        
        if cv_model_name == "blazepose":
            self.cv_model = Blazepose()
        else:
            print("Unrecognized computer vision model: {cv_model_name}")
            sys.exit(1)
        
        self.frame_input = frame_input

        if "hide_video" in kwargs:
            self.hide_video = kwargs["hide_video"]
        else:
            self.hide_video = False

        if "record_video" in kwargs:
            self.video_logger: VideoLogger = kwargs["record_video"]
        else:
            self.video_logger = None
    
    def update_pose(self) -> bool:

        frame: np.ndarray = self.frame_input.get_frame()
        image = cv2.cvtColor(cv2.flip(frame, 1), cv2.COLOR_BGR2RGB)

        # To improve performance, optionally mark the image as not writeable to
        # pass by reference.
        image.flags.writeable = False
        self.pose = self.cv_model.get_pose(image)
        image.flags.writeable = True

        if not self.hide_video:
            image2 = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            if self.video_logger != None:
                self.video_logger.log(image2)
            cv2.imshow("MediaPipe Pose", image2)

        if cv2.waitKey(5) & 0xFF == 27:
            return False

        # super().add_pose_to_queue()
        return True
    
    def close(self):
        self.frame_input.close()