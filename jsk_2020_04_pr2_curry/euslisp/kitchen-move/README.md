# kitchen-move

kitchen move test

## move-to-kitchen
move to kitchen-spot (infront of kitchenboard).

```
roscore
rlwrap roseus move-to-kitchen.l
```

TODO! need to change target of *ri*

## Tabletop grasp test
test tabletop recognition and grasp vegs from above

```
rlwrap roseus table-grasp-from-above.l
```

## peel potato test
test peeling potato by peeler

```
# make potato model
(set-potato :w 90 :l 60 :h 60) # this value should be actual value
# grasp peeler
(set-peeler) # during this part you need to pass the peeler to PR2
# attach left gripper
(set-larm)
# peel move
(peel-test)
# if you want finish
(finish)
```
