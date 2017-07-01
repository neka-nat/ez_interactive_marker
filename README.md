# ez_interactive_marker

Easily create interactive markers from yaml files.

## yaml setting

You can create a box with a menu by creating the following setting file.

```
sample:
  interactive_marker:
    name: 'sample_cube'
    header: {frame_id: 'base_link'}
    pose: {orientation: {w: 1.0}}
    controls:
      - always_visible: True
        interaction_mode: 2
        markers:
          - type: 1
            scale: {x: 0.45, y: 0.45, z: 0.45}
            color: {r: 0.0, g: 0.5, b: 0.5, a: 1.0}
            pose: {orientation: {w: 1.0}}
  menu:
    - title: "menu0"
    - title: "menu1"
      children: [{title: "submenu0"}, {title: "submenu1"}]
```

![rviz_image](images/rviz_image.png)