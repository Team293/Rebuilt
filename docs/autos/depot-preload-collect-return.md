# Depot Preload Collect Return

Select **Depot Preload Collect Return** in PathPlanner or the robot's **Auto Path**
chooser. This is the left/depot-side mirror of **Outpost Preload Collect Return**
across the field's width, using its own two path files.

The sequence is identical: 2.5-second spin-up, 2.5-second preload shooting,
midfield pickup and return, 5-second shooting, then final outward pickup until
the 20-second deadline. Transit uses 3 m/s and 8 m/s²; pickup uses 1.48 m/s and
1 m/s². Intake markers and cleanup are unchanged.

Blue starting pose: **(3.6091, 7.4297) m, heading 0°**. Pickup turns face **−90°**.
Normal alliance flipping handles red; leave **Auto/PathFlipped** false.

To visualize it with the [simulation setup](../simulation.md):

```sh
./gradlew simulateJava '-Pauto=Depot Preload Collect Return'
```

Restart simulation so the auto chooser discovers the new auto and paths.
