# Represents a polar histogram

Header header

# The amount of sectors/buckets into which the circle is divided
uint32 size

# The value for each sector in the range [0, 2*pi[, starting at 0 rad in
# Cartesian coordinates.
float32[] data

# Upper and lower threshold values for obstacle detection
float32 threshold_upper
float32 threshold_lower
