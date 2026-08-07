# Left and Right Hand Landmark Topics

## Left hand

Topic:

```text
/teleop_hand_tracking/left/landmarks
```

Message type:

```text
geometry_msgs/msg/PoseArray
```

Meaning: the message contains the position and orientation of all 25 tracked
joints of the Quest user's left hand.

## Right hand

Topic:

```text
/teleop_hand_tracking/right/landmarks
```

Message type:

```text
geometry_msgs/msg/PoseArray
```

Meaning: the message contains the position and orientation of all 25 tracked
joints of the Quest user's right hand.

## Complete message type

Both topics use `geometry_msgs/msg/PoseArray`. Its complete expanded ROS 2
message definition is:

```text
geometry_msgs/msg/PoseArray

std_msgs/msg/Header header
    builtin_interfaces/msg/Time stamp
        int32 sec
        uint32 nanosec
    string frame_id

geometry_msgs/msg/Pose[] poses
    geometry_msgs/msg/Point position
        float64 x
        float64 y
        float64 z
    geometry_msgs/msg/Quaternion orientation
        float64 x
        float64 y
        float64 z
        float64 w
```

For these landmark topics:

- `header.stamp.sec` and `header.stamp.nanosec` form the ROS timestamp.
- `header.frame_id` is `vuer_world`.
- `poses` is an array containing exactly 25 `geometry_msgs/msg/Pose` values.
- `position.x`, `position.y`, and `position.z` are the joint position in metres.
- `orientation.x`, `orientation.y`, `orientation.z`, and `orientation.w` are
  the joint orientation quaternion.

## Full example topic output

This is one complete message captured from
`/teleop_hand_tracking/right/landmarks`. It contains all 25 poses:

```yaml
header:
  stamp:
    sec: 1786096455
    nanosec: 605614596
  frame_id: vuer_world
poses:
- position:
    x: -0.22268617153167725
    y: -0.44024646282196045
    z: 0.9805196523666382
  orientation:
    x: 0.25346807079641837
    y: -0.2502505593625524
    z: 0.04395696590860406
    w: 0.9333790118561599
- position:
    x: -0.18707072734832764
    y: -0.4085875451564789
    z: 0.9949742555618286
  orientation:
    x: -0.5435154322166382
    y: -0.24636968391183256
    z: 0.23203944243345573
    w: 0.768147545037225
- position:
    x: -0.16200588643550873
    y: -0.3882899880409241
    z: 0.9990794062614441
  orientation:
    x: -0.5041411107896902
    y: -0.07540023119914424
    z: 0.08542100429141497
    w: 0.856072308612245
- position:
    x: -0.12909018993377686
    y: -0.38077855110168457
    z: 1.0005314350128174
  orientation:
    x: -0.5789964750192155
    y: 0.08269767752214295
    z: 0.03492299516520669
    w: 0.8103730995390072
- position:
    x: -0.10469749569892883
    y: -0.3809911608695984
    z: 0.9971905946731567
  orientation:
    x: -0.5789964750192155
    y: 0.08269767752214295
    z: 0.03492299516520669
    w: 0.8103730995390072
- position:
    x: -0.18849116563796997
    y: -0.4204659163951874
    z: 0.9981768727302551
  orientation:
    x: 0.25346807079641837
    y: -0.2502505593625524
    z: 0.04395696590860406
    w: 0.9333790118561599
- position:
    x: -0.1407497525215149
    y: -0.4204910397529602
    z: 1.0326693058013916
  orientation:
    x: 0.19503472537160207
    y: -0.08959501610446272
    z: 0.11322685853801319
    w: 0.9701102347125701
- position:
    x: -0.10440385341644287
    y: -0.41348445415496826
    z: 1.0409374237060547
  orientation:
    x: 0.19656415204360084
    y: 0.09394339471423702
    z: 0.14725527259391466
    w: 0.964807264387161
- position:
    x: -0.08158320188522339
    y: -0.40568113327026367
    z: 1.0379388332366943
  orientation:
    x: 0.20328746470373918
    y: 0.14500248754866712
    z: 0.13002182386588504
    w: 0.959553443335812
- position:
    x: -0.06052060425281525
    y: -0.3994048833847046
    z: 1.0336686372756958
  orientation:
    x: 0.20328746470373918
    y: 0.14500248754866712
    z: 0.13002182386588504
    w: 0.959553443335812
- position:
    x: -0.18833160400390625
    y: -0.4359285235404968
    z: 0.9920592308044434
  orientation:
    x: 0.25346807079641837
    y: -0.2502505593625524
    z: 0.04395696590860406
    w: 0.9333790118561599
- position:
    x: -0.13861817121505737
    y: -0.44177505373954773
    z: 1.0262137651443481
  orientation:
    x: 0.2720951480440005
    y: -0.08220442879090396
    z: 0.030831159200816426
    w: 0.9582568037433434
- position:
    x: -0.09635293483734131
    y: -0.44115889072418213
    z: 1.0336970090866089
  orientation:
    x: 0.2559931969953816
    y: 0.26136062480383704
    z: 0.1257243472364082
    w: 0.9221450511748197
- position:
    x: -0.07343809306621552
    y: -0.43108442425727844
    z: 1.0221906900405884
  orientation:
    x: 0.2867666728739922
    y: 0.4247169782995507
    z: 0.15940605035563848
    w: 0.8437831918110811
- position:
    x: -0.05782148241996765
    y: -0.41892606019973755
    z: 1.0069286823272705
  orientation:
    x: 0.2867666728739922
    y: 0.4247169782995507
    z: 0.15940605035563848
    w: 0.8437831918110811
- position:
    x: -0.1865922510623932
    y: -0.4518337547779083
    z: 0.9862868785858154
  orientation:
    x: 0.25346807079641837
    y: -0.2502505593625524
    z: 0.04395696590860406
    w: 0.9333790118561599
- position:
    x: -0.1388905644416809
    y: -0.45614081621170044
    z: 1.0111777782440186
  orientation:
    x: 0.32626291711255934
    y: -0.0334017557328393
    z: 0.015372379696173787
    w: 0.9445636672949281
- position:
    x: -0.09999989718198776
    y: -0.4558582901954651
    z: 1.014029622077942
  orientation:
    x: 0.32554288711054247
    y: 0.3757508099817798
    z: 0.16767574171794702
    w: 0.851303707902829
- position:
    x: -0.08242446184158325
    y: -0.44177091121673584
    z: 0.9999301433563232
  orientation:
    x: 0.2972006023992137
    y: 0.572408445837291
    z: 0.263670356329394
    w: 0.7172853799296428
- position:
    x: -0.07583929598331451
    y: -0.42466944456100464
    z: 0.9838494658470154
  orientation:
    x: 0.2972006023992137
    y: 0.572408445837291
    z: 0.263670356329394
    w: 0.7172853799296428
- position:
    x: -0.18401657044887543
    y: -0.4570632874965668
    z: 0.9797908663749695
  orientation:
    x: 0.47292667183987613
    y: -0.18048799979486063
    z: -0.03198858943398274
    w: 0.861824329627873
- position:
    x: -0.14143356680870056
    y: -0.46737155318260193
    z: 0.9926127791404724
  orientation:
    x: 0.41193017450625175
    y: 0.000779777809186531
    z: -0.004492275528030787
    w: 0.9112040072005816
- position:
    x: -0.11071443557739258
    y: -0.46760332584381104
    z: 0.9924554228782654
  orientation:
    x: 0.4404956690952372
    y: 0.3889583150461694
    z: 0.1470328180580666
    w: 0.7956483803029374
- position:
    x: -0.09742701053619385
    y: -0.455890953540802
    z: 0.9825147390365601
  orientation:
    x: 0.4103494838800949
    y: 0.5435442423851213
    z: 0.2848257248028214
    w: 0.6745719117632096
- position:
    x: -0.09082379937171936
    y: -0.43786078691482544
    z: 0.971863865852356
  orientation:
    x: 0.4103494838800949
    y: 0.5435442423851213
    z: 0.2848257248028214
    w: 0.6745719117632096
---
```

## Pose index meaning

| Index | Joint |
| ---: | --- |
| 0 | Wrist |
| 1 | Thumb metacarpal |
| 2 | Thumb proximal |
| 3 | Thumb distal |
| 4 | Thumb tip |
| 5 | Index metacarpal |
| 6 | Index proximal |
| 7 | Index intermediate |
| 8 | Index distal |
| 9 | Index tip |
| 10 | Middle metacarpal |
| 11 | Middle proximal |
| 12 | Middle intermediate |
| 13 | Middle distal |
| 14 | Middle tip |
| 15 | Ring metacarpal |
| 16 | Ring proximal |
| 17 | Ring intermediate |
| 18 | Ring distal |
| 19 | Ring tip |
| 20 | Pinky metacarpal |
| 21 | Pinky proximal |
| 22 | Pinky intermediate |
| 23 | Pinky distal |
| 24 | Pinky tip |
