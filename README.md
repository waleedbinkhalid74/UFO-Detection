# UFO-Detection

There are two buoys on which radars are installed.

 The two radars detected several unidentified floating objects (ofnis) in the vicinity.
 When a radar detects an ofni, it provides an azimuth and an approximate distance:
 - The azimuth is a real number between -180 ° and 180 °.  It is 0 ° if the ofni is exactly north of the radar, 90 ° if the ofni is to the east.
 - The distance is given in meters.  We know that this data is not very precise (with a distance of 50 meters maximum compared to the actual value).

 The objective of this exercise is to code an algorithm allowing to calculate the precise position of the OFNS with respect to the first buoy.

 The entries are:
 - the couples (azimuth, distance) obtained on the first buoy,
 - the couples (azimuth, distance) obtained on the second buoy,
 - the relative position of the second buoy with respect to the first.

 All the coordinates (x, y) are expressed in the NE convention, this means that the x axis points to the north and the y axis points to the east.

 Simplifying assumptions:
 - Ofnis are all perceived by the two radars (no one by hiding another).
 - Ofnis have never gotten too close to each other.  The distance between two ofnis is always greater than or equal to 250 meters.
 - No ofnis is placed in the alignment of the two buoys.
