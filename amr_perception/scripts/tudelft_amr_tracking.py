import numpy as np
import rospy
from copy import copy
from jsk_recognition_msgs.msg import BoundingBoxArray
from spencer_tracking_msgs.msg import TrackedPerson, TrackedPersons
from amr_perception.tracking import Tracker


class Detections:
    def __init__(self):
        self.boxes = BoundingBoxArray()
        self.boxes.boxes = []
        rospy.Subscriber('/detections3D', BoundingBoxArray, self.cb, queue_size=1)

    def cb(self, msg):
        self.boxes = msg

    def __iter__(self):
        return self

    def __next__(self):
        return copy(self.boxes)


def main():
    rospy.init_node("person_tracker")
    pub_viz = rospy.Publisher("tracks", TrackedPersons, queue_size=10)

    f = 10
    tracker = Tracker(
        dist_threshold=2.0,
        max_frame_skipped=5,
        max_trace_length=5,
        frequency=f
    )
    detector = Detections()

    rospy.sleep(1)

    r = rospy.Rate(f)

    while not rospy.is_shutdown():
        for detections in detector:
            centers = np.array([[box.pose.position.x, box.pose.position.y] for box in detections.boxes])

            # filter centers
            filtered_centers = []
            for center in centers:
                if any([np.linalg.norm(center - fc) < 0.5 for fc in filtered_centers]):
                    continue
                filtered_centers.append(center)

            if len(filtered_centers) > 0:
                print(filtered_centers)
                tracker.update(np.array(filtered_centers))

            persons = TrackedPersons()
            persons.header.frame_id = 'map' 
            for track in tracker.tracks:
                person = TrackedPerson()
                if len(track.trace) <= 4:
                    continue
                person.pose.pose.position.x = track.trace[-1][0]
                person.pose.pose.position.y = track.trace[-1][2]
                person.twist.twist.linear.x = track.trace[-1][1]
                person.twist.twist.linear.y = track.trace[-1][3]
                person.track_id = track.track_id
                persons.tracks.append(person)

            pub_viz.publish(persons)
            r.sleep()


if __name__ == '__main__':
    main()
