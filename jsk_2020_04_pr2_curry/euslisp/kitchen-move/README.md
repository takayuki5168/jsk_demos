# kitchen-move

kitchen move test

## move-to-kitchen
move to kitchen-spot (infront of kitchenboard).

```
roscore
rlwrap roseus move-to-kitchen.l

## go to /eng2/7f/room73B2-front-of-kitchenboard spot
(move-to-kitchenboard-ri)

## go to kitchen-spot original spot
(move-to-kitchen-ri)
```


TODO! need to change target of *ri*

## Tabletop grasp test
test tabletop recognition and grasp vegs from above

```
roslaunch jsk_2020_04_pr2_curry tabletop_test.launch
rlwrap roseus table-grasp-from-above.l
```

## peel potato test
test peeling potato by peeler

```
rlwrap roseus peel-potato-code.l

# make potato model
(set-potato :w 90 :l 60 :h 60) # this value should be actual value
# grasp peeler
(set-peeler) # during this part you need to pass the peeler to PR2
# attach left gripper
(set-larm)
# peel move
(peel-test1) # or (peel-test2) for In small increments
# if you want finish
(finish)
```

also exec `(set-peel)` and `(exec-peel)`

## cut potato test
test cutting potato by knife

```
rlwrap roseus cut-potato-code.l

# make potato model
(set-potato :w 90 :l 60 :h 60) # this value should be actual value
# grasp peeler
(set-knife-ver) # during this part you need to pass the knife to PR2
# attach left gripper
(set-larm)
# peel move
(cut-test1) # or (cut-test2) (cut-test3)
# if you want finish
(finish)
```

also exec `(set-cut)` and `(hor-test)` (horizontal grasp)
