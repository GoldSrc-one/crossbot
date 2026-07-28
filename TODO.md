a* path with airmoves does't do the funneling. The funneling should be attempted and the jump simulated. If the jump doesn't land, use the stored airmove.

the funneling thing that keeps some distance from the walls doesn't work consistently. It depends on portal orientation. Maybe we should move it in a direction that is perpendicular to the breaking point. Are we able to do this without accidently moving the point too far into possible wall in that direction?

doors, plats, trains... move them to their other positions (time) and run airmove sampling out of them too, note the time into the move. For reverse jump edges, note the time too.

doors and breakables shouldn't carve the areas, they should split them and stay split, marked, and connected
then for pathfinding, if the doors and breakables are unnamed and are not trigger only, we can pathfind straight through them.
if they are named and closed or unbroken (that is runtime info so in navgen debug we will assume so), navigate to the button/trigger first.

ladders... all floors that touch a ladder should be connected by a ladder connection. That would contain exact mounting and dismounting point, that would be the point in player height (that would depend if area is crouch or stand) on the ladder side facing the centroid. No funneling on ladders.

