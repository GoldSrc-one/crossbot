airmove sampler takes the best move? I should average them out. That is why the jumpBackDist was supposed to be -inf for non jumps, to short it out. But you could just have...
Ok I changed my mind, let's run it two times, once for drops and once for jumps. If a drop already landed, ignore the jump (drop is better).

Also airmove now works in hull3 by default. That assumes the bot always jumping from crouch. But we are simulating the jumps and drops from what height?
Ok lets changed this. We should set the hull based on the source area. If it is crouch, just use hull3 and use crouching height, but also use crouching maxspeed!
If the source area is stand area, do a trace for both, hull1 with standing height and standing maxspeed, but also hull3 but with standing height and standing maxspeed -- that will simulate the player crouchjumping (or crouching before dropping).
And once again we would bin the airmoves on the target area by what hull they used. If a hull1 bin would have something in int, it would be used. Otherwise hull3 would be used and the airmove would be marked as crouch only.

a* path with airmoves does't do the funneling. The funneling should be attempted and the jump simulated. If the jump doesn't land, use the stored airmove.

the funneling thing that keeps some distance from the walls doesn't work consistently. It depends on portal orientation. Maybe we should move it in a direction that is perpendicular to the breaking point. Are we able to do this without accidently moving the point too far into possible wall in that direction?

stairs and slopes could be merged, as long as the X-Y footprint is convex and Z doesn't intersect with other areas

doors, plats, trains... move them to their other positions (time) and run airmove sampling out of them too, note the time into the move. For reverse jump edges, note the time too.

doors and breakables shouldn't carve the areas, they should split them and stay split, marked, and connected
then for pathfinding, if the doors and breakables are unnamed and are not trigger only, we can pathfind straight through them.
if they are named and closed or unbroken (that is runtime info so in navgen debug we will assume so), navigate to the button/trigger first.

ladders... all floors that touch a ladder should be connected by a ladder connection. That would contain exact mounting and dismounting point, that would be the point in player height (that would depend if area is crouch or stand) on the ladder side facing the centroid. No funneling on ladders.

