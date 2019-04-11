from djitellopy import Tello
import dlib
import cv2
import pygame
from pygame.locals import *
import numpy as np
import time
import face_recognition

# Speed of the drone
S = 60
turnSpeed = 35
# Frames per second of the pygame window display
FPS = 25

# Load a sample picture and learn how to recognize it.
tom_image = face_recognition.load_image_file("opencv_frame_0.png")
tom_face_encoding = face_recognition.face_encodings(tom_image)[0]

# Load a second sample picture and learn how to recognize it.
# ashkan_image = face_recognition.load_image_file("ashkan.jpg")
# ashkan_face_encoding = face_recognition.face_encodings(ashkan_image)[0]

# Create arrays of known face encodings and their names
known_face_encodings = [
    tom_face_encoding
    # ashkan_face_encoding
]
known_face_names = [
    "Addison"
    # "Ashkan"
]

# Just general purpose variables to specify the target by name and where they are
#specifiedTarget = "Tom"
#targetSeen = False
#targetLeftSide = 320 # set to default
#targetRightSide = 640
#hasTakenOff = False

# Initialize some variables
face_locations = [] # Pretty sure this is where the locations of the faces are
face_encodings = []
face_names = []


class FrontEnd(object):
    """ Maintains the Tello display and moves it through the keyboard keys.
        Press escape key to quit.
        The controls are:
            - T: Takeoff
            - L: Land
            - Arrow keys: Forward, backward, left and right.
            - A and D: Counter clockwise and clockwise rotations
            - W and S: Up and down.
    """

    def getTakeOff(self):
        return self.hasTakenOff

    def setTakeoff(self,hasTakenOff):
        self.hasTakenOff = hasTakenOff

    def __init__(self):
        # Init pygame
        pygame.init()

        # Creat pygame window
        pygame.display.set_caption("Tello video stream")
        self.screen = pygame.display.set_mode([960, 720])

        # Init Tello object that interacts with the Tello drone
        self.tello = Tello()

        # Drone velocities between -100~100
        self.for_back_velocity = 0
        self.left_right_velocity = 0
        self.up_down_velocity = 0
        self.yaw_velocity = 0
        self.speed = 10

###################################################################
        ##Drone Project Defined variables
        self.hasTakenOff = False
        self.specifiedTarget = known_face_names[0]
        self.targetSeen = False
        self.targetLeftSide = 320 # set to default
        self.targetRightSide = 640


###################################################################
        self.send_rc_control = False

        # create update timer
        pygame.time.set_timer(USEREVENT + 1, 50)

    def run(self):

        if not self.tello.connect():
            print("Tello not connected")
            return

        if not self.tello.set_speed(self.speed):
            print("Not set speed to lowest possible")
            return

        # In case streaming is on. This happens when we quit this program without the escape key.
        if not self.tello.streamoff():
            print("Could not stop video stream")
            return

        if not self.tello.streamon():
            print("Could not start video stream")
            return

        frame_read = self.tello.get_frame_read()

        should_stop = False

        detector = dlib.get_frontal_face_detector()

        spinCounter = 0

        while not should_stop:

            for event in pygame.event.get():
                if event.type == USEREVENT + 1:
                    self.update()
                elif event.type == QUIT:
                    should_stop = True
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        should_stop = True
                    else:
                        self.keydown(event.key)
                elif event.type == KEYUP:
                    self.keyup(event.key)

            if frame_read.stopped:
                frame_read.stop()
                break

            self.screen.fill([0, 0, 0])
            frame = cv2.cvtColor(frame_read.frame, cv2.COLOR_BGR2RGB)
            #dets = detector(frame)
            #for det in dets:
            #    cv2.rectangle(frame, (det.left(), det.top()), (det.right(), det.bottom()), color=(0,255,0), thickness=3)

            # Resize frame of video to 1/4 size for faster face recognition processing
            small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Only process every other frame of video to save time
            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            face_names = []

            for face_encoding in face_encodings:
                # See if the face is a match for the known face(s)
                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                # If a match was found in known_face_encodings, just use the first one.
                if True in matches:
                    first_match_index = matches.index(True)
                    name = known_face_names[first_match_index] # This might not actually be correct, given it only uses the first one

                face_names.append(name)

                for (top, right, bottom, left), name in zip(face_locations, face_names):
                    # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                    top *= 2
                    right *= 2
                    bottom *= 2
                    left *= 2

                    # Draw a box around the face
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                    # Draw a label with a name below the face
                    cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                    font = cv2.FONT_HERSHEY_DUPLEX
                    cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

                    # Updates when the target is seen
                    if (name == self.specifiedTarget):
                        self.targetSeen = True
                        self.targetLeftSide = left
                        self.targetRightSide = right

            frame = np.rot90(frame)
            frame = np.flipud(frame)
            frame = pygame.surfarray.make_surface(frame)
            self.screen.blit(frame, (0, 0))
            pygame.display.update()


            # Circles right until face detected
            #while (targetSeen != True and self.hasTakenOff):
                #self.tello.rotate_clockwise(self, 10)
                #self.keydown(self, pygame.K_a)
            if (self.targetSeen != True and self.hasTakenOff):
                self.yaw_velocity = turnSpeed
                spinCounter = spinCounter + 1
                if (spinCounter > (10*FPS)):
                    self.tello.land()
                    self.send_rc_control = False
                    self.tello.end()
                    #break ## ends loop and land drone
            else:
                if (self.targetLeftSide < 260):
                    #self.tello.rotate_clockwise(self, 10)
                    # self.keydown(self, pygame.K_a)
                    self.yaw_velocity = -turnSpeed
                    # rotate left, target outside of the middle 1/3
                elif (self.targetRightSide > 700):
                    #self.tello.rotate_counter_clockwise(self, 10)
                    # self.keydown(self, pygame.K_d)
                    self.yaw_velocity = turnSpeed
                else:
                    self.yaw_velocity = 0


            time.sleep(1 / FPS)

        # Call it always before finishing. I deallocate resources.
        self.tello.end()

    def keydown(self, key):
        """ Update velocities based on key pressed
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP:  # set forward velocity
            self.for_back_velocity = S
        elif key == pygame.K_DOWN:  # set backward velocity
            self.for_back_velocity = -S
        elif key == pygame.K_LEFT:  # set left velocity
            self.left_right_velocity = -S
        elif key == pygame.K_RIGHT:  # set right velocity
            self.left_right_velocity = S
        elif key == pygame.K_w:  # set up velocity
            self.up_down_velocity = S
        elif key == pygame.K_s:  # set down velocity
            self.up_down_velocity = -S
        elif key == pygame.K_a:  # set yaw clockwise velocity
            self.yaw_velocity = -S
        elif key == pygame.K_d:  # set yaw counter clockwise velocity
            self.yaw_velocity = S

    def keyup(self, key):
        """ Update velocities based on key released
        Arguments:
            key: pygame key
        """
        if key == pygame.K_UP or key == pygame.K_DOWN:  # set zero forward/backward velocity
            self.for_back_velocity = 0
        elif key == pygame.K_LEFT or key == pygame.K_RIGHT:  # set zero left/right velocity
            self.left_right_velocity = 0
        elif key == pygame.K_w or key == pygame.K_s:  # set zero up/down velocity
            self.up_down_velocity = 0
        elif key == pygame.K_a or key == pygame.K_d:  # set zero yaw velocity
            self.yaw_velocity = 0
        elif key == pygame.K_t:  # takeoff
            self.tello.takeoff()
            self.hasTakenOff = True
            self.send_rc_control = True
        elif key == pygame.K_l:  # land
            self.tello.land()
            self.send_rc_control = False

    def update(self):
        """ Update routine. Send velocities to Tello."""
        if self.send_rc_control:
            self.tello.send_rc_control(self.left_right_velocity, self.for_back_velocity, self.up_down_velocity,
                                       self.yaw_velocity)


def main():
    frontend = FrontEnd()

    # run frontend
    frontend.run()


if __name__ == '__main__':
    main()
