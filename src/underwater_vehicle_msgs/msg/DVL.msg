Header header

string name

# Measured velocity [m/s] in sensor frame
geometry_msgs/Vector3 velocity

# Row major, xyz axes
float64[9] velocity_covariance

# Bottom range, only valid if the velocity_reference is VELOCITY_REFERENCE_BOTTOM
float64 range

# Instrument reference data
uint8 VELOCITY_REFERENCE_UNKNOWN = 0
uint8 VELOCITY_REFERENCE_WATER = 1
uint8 VELOCITY_REFERENCE_BOTTOM = 2

uint8 velocity_reference