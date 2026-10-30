# Depot Preload Collect Return

Select **Depot Preload Collect Return** in PathPlanner or the robot's **Auto Path**
chooser. This is the left/depot-side mirror of **Outpost Preload Collect Return**
across the field's width, using its own two path files.

The sequence is identical: leave immediately while deploying the intake and
spinning up, collect in midfield, return and shoot preloads plus collected balls
for 5 seconds, then make the second pickup, return over the bump and attempt
another 5-second shot until the 20-second deadline. There is no initial preload
shot or startup wait. The modeled first shot begins around 5.95 seconds; the
second return finishes around 17.99 seconds, leaving about 2 seconds for the
final shot. The flywheel remains spun up through both returns.
Transit uses 3 m/s and 8 m/s²; pickup uses 1.48 m/s and
1 m/s². Intake markers and cleanup are unchanged.

Blue starting pose: **(3.6091, 7.4297) m, heading 0°**. Pickup turns face **−90°**.
Normal alliance flipping handles red; leave **Auto/PathFlipped** false.

To visualize it with the [simulation setup](../simulation.md):

```sh
./gradlew simulateJava '-Pauto=Depot Preload Collect Return'
```

Restart simulation so the auto chooser discovers the new auto and paths.
