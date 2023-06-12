## base_2dnav
Handles Quori's navigation

### `roslaunch base_2dnav base_2dnav.launch`
Starts up everything needed to have Quori navigate a map

### `rosrun base_2dnav quori_wander`
Pick a random point on the map and attempt to travel there

### `rosrun base_2dnav quori_san`
Pick a point on the map to travel to depending on room density and vulnerability
