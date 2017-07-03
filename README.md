# ez_interactive_marker

[![Build Status](https://travis-ci.org/neka-nat/ez_interactive_marker.svg?branch=master)](https://travis-ci.org/neka-nat/ez_interactive_marker)

Easily create interactive markers from yaml files.

## Quick start

You can create a box with a menu by creating the following setting file.

```yaml:sample_cube.yaml
sample:
  interactive_marker:
    name: 'sample_cube'
    header: {frame_id: 'base_link'}
    pose: {orientation: {w: 1.0}}
    controls:
      - always_visible: True
        interaction_mode: !enum [visualization_msgs/InteractiveMarkerControl, BUTTON]
        markers:
          - type: !enum [visualization_msgs/Marker, CUBE]
            scale: {x: 0.45, y: 0.45, z: 0.45}
            color: {r: 0.0, g: 0.5, b: 0.5, a: 1.0}
            pose: {orientation: {w: 1.0}}
  menu:
    - title: "menu0"
    - title: "menu1"
      children: [{title: "submenu0"}, {title: "submenu1"}]
```

Please execute the following command.

```
rosrun ez_interactive_marker ez_interactive_marker -c simple_cube.yaml
```

![rviz_image](images/rviz_image.png)

## Supported yaml tags

It is possible to use several tags in the configuration file.

### include
This tag includes the written configuration file and expand the contents.

```yaml
!include sub_settings.yaml
```

### enum
This tag expands the enum variable of the specified message module.

```yaml
!enum [visualization_msgs/Marker, CUBE]
```

### euler
This tag converts euler xyz angle to quaternion.

```yaml
!euler [3.14159, 0.0, 0.0] # -> [1.0, 0.0, 0.0, 0.0]
```

### degrees
This tag converts degrees to radians.

```yaml
!degrees 90.0 # -> 1.5708
```

## Menu settings

You can specify several commands to be executed when menu is selected.

### publish topic

In the following example, the string "hello" is published to the topic of "test".
Please also see [this example](example/simple_cube.yaml).

```yaml
menu:
  - title: "menu0"
    command:
      type: 'topic_pub'
      args:
        name: '/test'
        type: 'std_msgs/String'
        msg: {data: 'Hello.'}
```

### service call

In the following example, "add_two_ints" service is called.
Please also see [this example](example/add_two_ints.yaml).

```yaml
menu:
  - title: "menu0"
    command:
      type: 'service_call'
      args:
        name: 'add_two_ints'
        type: 'rospy_tutorials/AddTwoInts'
        request: {a: 1.0, b: 2.0}
```
