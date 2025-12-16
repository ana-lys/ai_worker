# CollisionCheck.msg
# Message for publishing collision check results

# Whether any collision is detected (distance < 0)
bool in_collision

# Array of distances for all contact pairs
# Negative values indicate penetration depth
float64[] distances

# Names of first geometry in each contact pair
string[] geom1_names

# Names of second geometry in each contact pair
string[] geom2_names