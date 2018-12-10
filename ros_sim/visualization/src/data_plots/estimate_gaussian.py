import rospy

import tf
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from scipy.stats import norm
import numpy as np
import matplotlib.pyplot as plt


rotationData = []
angularVelocityData = []

def callback(data):
    angularVelocityData.append(data.angular_velocity)
    rotationData.append(data.orientation)

def listener(dataStream):
    rospy.init_node('estimate_gaussian_node', anonymous=True)

    rospy.Subscriber(dataStream, Imu, callback)

    start = rospy.Time.now()

    rospy.spin()

    print("Num Data Points:" + str(len(rotationData)))

    data = []

    for i in rotationData:
        quaternion = (
            i.x,
            i.y,
            i.z,
            i.w)
        data.append(tf.transformations.euler_from_quaternion(quaternion)[2])

    # Fit a normal distribution to the data:
    mu, std = norm.fit(data)

    print(mu, std)

    # Plot the histogram.
    plt.hist(data, bins=25, normed=True, alpha=0.6, color='g')

    # Plot the PDF.
    xmin, xmax = plt.xlim()
    x = np.linspace(xmin, xmax, 100)
    p = norm.pdf(x, mu, std)
    plt.plot(x, p, 'k', linewidth=2)
    title = "Fit results: mu = %.2f,  std = %.2f" % (mu, std)
    plt.title(title)

    plt.show()


if __name__ == '__main__':
    listener("/vehicles/v1/imu/data")