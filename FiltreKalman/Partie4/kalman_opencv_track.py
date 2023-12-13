import cv2
import numpy as np

def track_object(frame, measurement):
    # Kalman filter setup
    if 'kf' not in track_object.__dict__:
        track_object.kf = cv2.KalmanFilter(4, 2, 0)
        track_object.kf.measurementMatrix = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float32)
        track_object.kf.transitionMatrix = np.array([[1, 0, 1, 0], [0, 1, 0, 1], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32)
        track_object.kf.processNoiseCov = np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]], dtype=np.float32) * 0.01
        track_object.kf.measurementNoiseCov = np.array([[1, 0], [0, 1]], dtype=np.float32) * 1.0
        track_object.prediction = np.array([0, 0], dtype=np.float32)

    # Predict the next state
    track_object.prediction = track_object.kf.predict()

    # Check if the object is detected (e.g., blue ball)
    if measurement is not None:
        # Correct the state with the measurement
        track_object.kf.correct(measurement)

    return track_object.prediction, track_object.kf.errorCovPre

def main():
    cap = cv2.VideoCapture(0)

    # Color range for the object (blue in this example)
    lower_blue = np.array([100, 50, 50])
    upper_blue = np.array([140, 255, 255])

    while True:
        ret, frame = cap.read()

        # Convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create a mask for the blue object
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Get the largest contour (assumed to be the object)
            largest_contour = max(contours, key=cv2.contourArea)

            # Get the centroid and area of the largest contour
            M = cv2.moments(largest_contour)
            area = cv2.contourArea(largest_contour)
            if area >= MIN_CONTOUR_AREA:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                # Measurement (centroid and area of the object)
                measurement = np.array([cx, cy], dtype=np.float32)
            else:
                measurement = None
        else:
            # If no contours or the object is too small, set measurement to None
            measurement = None

        # Track the object using Kalman filter
        prediction, covariance = track_object(frame, measurement)

        # Draw the object on the frame
        if measurement is not None:
            cv2.circle(frame, (int(measurement[0]), int(measurement[1])), 5, (0, 255, 0), -1)  # measured position in green

        # Adjust the circle size based on the covariance matrix
        radius = int(np.sqrt(covariance[0, 0] + covariance[1, 1]))*RADIUS_FACTOR
        cv2.circle(frame, (int(prediction[0]), int(prediction[1])), radius, (0, 0, 255), 2)  # predicted position in red

        # Display the frame
        cv2.imshow("Object Tracking", frame)

        # Break the loop if 'q' key is pressed
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    RADIUS_FACTOR = 10
    MIN_CONTOUR_AREA = 2000
    main()

