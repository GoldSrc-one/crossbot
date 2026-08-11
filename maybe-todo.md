doors, plats, trains... move them to their other positions (time) and run airmove sampling out of them too, note the time into the move. For reverse jump edges, note the time too.

sidemove sampling + yawrate in airmove ;-)

buttons and activators for doors and breakables -- the path finder should have a parameter "should I try to find the activators?" which would be ON by default. When the path finder would hit a door or a breakable that needs activation and this param would be OFF, the connection through that obstacle wouldn't be considered; if it were ON, a new, recursive path find would be launched from the starting point (or last activator position) to the activator position. If that path couldn't be found, the connection through the obstacle wouldn't be considered. If such path would be found, the path finding would continue from the sub-path end (the activator position).
Alternative solution -- let the activations be a part of the search state. If during the search an activator would be found, this would allow for re-visit of previsouly visited areas. And the doors and breakables would be considered unpassable (if they require activation and are not already open or broken) unless the correct activation happened on the path. It could be like a... 128 bitfield? That should be enough to cover all possible activations in a map (distinct by name, mapped to bits).

integrate namegiver
rename to goldclanker, .nav to .gcn
