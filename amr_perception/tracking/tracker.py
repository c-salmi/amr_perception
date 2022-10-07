import numpy as np 
from amr_perception.tracking.kalman_filter import KalmanFilter
from scipy.optimize import linear_sum_assignment
from collections import deque


class Track:
    def __init__(self, detection, track_id, frequency):
        state = np.matrix([
            [detection[0]],
            [0],
            [detection[1]],
            [0]
        ])
        self.KF = KalmanFilter(init_state=state, frequency=frequency)
        self.trace = deque(maxlen=20)
        self.track_id = track_id
        self.skipped_frames = 0

    @property
    def prediction(self):
        return np.array([self.KF.pred_state[0, 0], self.KF.pred_state[2, 0]])

    def update(self, detection):
        self.KF.correct(np.matrix(detection).reshape(2, 1))


class Tracker:
    def __init__(self, dist_threshold, max_frame_skipped, max_trace_length, frequency):
        self.dist_threshold = dist_threshold
        self.max_frame_skipped = max_frame_skipped
        self.max_trace_length = max_trace_length
        self.current_track_id = 0
        self.tracks = []
        self.frequency = frequency

    def update(self, detections):
        # initialize tracks
        if len(self.tracks) == 0:
            for detection in detections:
                self.tracks.append(Track(detection, self.current_track_id, self.frequency))
                self.current_track_id += 1

        # calculate dist w.r.t. existing track predictions
        dists = np.array([np.linalg.norm(detections - track.prediction, axis=1) for track in self.tracks])

        # determine detection assignment
        assignment = np.array(linear_sum_assignment(dists)).T
        assignment = [a for a in assignment if dists[a[0], a[1]] < self.dist_threshold]

        # update state of existing tracks
        for track_idx, detection_idx in assignment:
            self.tracks[track_idx].update(detections[detection_idx])
            self.tracks[track_idx].skipped_frames = 0

        # propagate unassigned tracks using the prediction (Constant Velocity)
        assigned_track_idxs = [track_idx for track_idx, _ in assignment]
        for i, track in enumerate(self.tracks):
            if i not in assigned_track_idxs:
                track.update(track.prediction)
                track.skipped_frames += 1

        # create new tracks for detections without track
        assigned_det_idxs = [det_idx for _, det_idx in assignment]
        for i, det in enumerate(detections):
            if i not in assigned_det_idxs:
                self.tracks.append(Track(det, self.current_track_id, self.frequency))
                self.current_track_id += 1

        # delete tracks if skipped_frames too large
        self.tracks = [track for track in self.tracks if not track.skipped_frames > self.max_frame_skipped]

        # update all track predictions
        [track.KF.predict() for track in self.tracks]
        for track in self.tracks:
            track.trace.append(
                np.array([
                    track.KF.state[0, 0],
                    track.KF.state[1, 0],
                    track.KF.state[2, 0],
                    track.KF.state[3, 0]
                ])
            )









