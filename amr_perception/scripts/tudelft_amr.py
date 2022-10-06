from easy_inference.providers.realsense import Realsense
from easy_inference.providers.utils import combine
from easy_inference.utils.boundingbox import BoundingBox
from easy_inference.utils.filters import filter_iou3d

import onnxruntime as ort
import numpy as np
import cv2

SHOW = True
ROS = True

if ROS:
    from easy_inference.utils.ros_connector import RosConnector
    ros_connector = RosConnector()

# ort.set_default_logger_severity(0)
ort_sess = ort.InferenceSession('yolov7-tiny.onnx', providers=['CUDAExecutionProvider'])

cam5 = Realsense(width=640, height=480, depth=True, device='215122255929')
cam4 = Realsense(width=640, height=480, depth=True, device='215122255934')
cam3 = Realsense(width=640, height=480, depth=True, device='215122254701')
cam2 = Realsense(width=640, height=480, depth=True, device='114222251376')
cam1 = Realsense(width=640, height=480, depth=True, device='215122255869')
providers = [cam1, cam2, cam3, cam4, cam5]

for frames in combine(*providers):
    rgb_frames = np.stack([f[1] for f in frames])
    depth_frames = np.stack([f[0] for f in frames])

    input = rgb_frames.transpose((0, 3, 1, 2))
    input = np.ascontiguousarray(input)
    input = input.astype(np.float32)
    input /= 255

    # output: [batch_id, x0, y0, x1, y1, class_id, conf]
    outputs = ort_sess.run(None, {'images': input})[0]

    # convert to BoundingBox for convenience
    boxes2d = [BoundingBox(*output[1:], batch_id=output[0]) for output in outputs]

    # filter classes
    boxes2d = [box2d for box2d in boxes2d if box2d.class_id == 0]

    # project to BoundingBox3d
    boxes3d = [box2d.to3d(depth_frames[box2d.batch_id], providers[box2d.batch_id]._depth_intr) for box2d in boxes2d]

    # filter iou overlap
    boxes3d = filter_iou3d(boxes3d)    

    if ROS: 
        ros_connector.publishBoundingBoxes3d(boxes3d)

    if SHOW:
        for box2d in boxes2d:
            cv2.rectangle(rgb_frames[box2d.batch_id], (box2d.x0, box2d.y0), (box2d.x1, box2d.y1), (0,255,0),2)

        cv2.imshow(f'frame', np.concatenate(
            (np.concatenate(rgb_frames[:3], axis=1), 
            np.concatenate((rgb_frames[3], rgb_frames[4], np.zeros_like(rgb_frames[4])), axis=1)),
            axis=0))

        if cv2.waitKey(1) == 27:
            break

cv2.destroyAllWindows()

