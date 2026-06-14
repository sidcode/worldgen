"""Emit SDF world files matching the style of the stock assignment worlds."""

import math

from .geometry import WALL_HEIGHT, WALL_THICKNESS

_WORLD_HEADER = """<sdf version='1.6'>
  <!-- {comment} -->
  <world name='default'>
    <model name='ground_plane'>
      <static>1</static>
      <link name='link'>
        <collision name='collision'>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>100</mu>
                <mu2>50</mu2>
              </ode>
              <torsional>
                <ode/>
              </torsional>
            </friction>
            <contact>
              <ode/>
            </contact>
            <bounce/>
          </surface>
          <max_contacts>10</max_contacts>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>200 200</size>
            </plane>
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
      </link>
    </model>
    <light name='sun' type='directional'>
      <cast_shadows>1</cast_shadows>
      <pose frame=''>0 0 10 0 -0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.1 0.1 0.1 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.5 -1</direction>
    </light>
    <physics name='default_physics' default='0' type='ode'>
      <ode>
        <solver>
          <type>quick</type>
          <iters>100</iters>
          <sor>1.3</sor>
          <use_dynamic_moi_rescaling>0</use_dynamic_moi_rescaling>
        </solver>
        <constraints>
          <cfm>0</cfm>
          <erp>0.2</erp>
          <contact_max_correcting_vel>100</contact_max_correcting_vel>
          <contact_surface_layer>0.001</contact_surface_layer>
        </constraints>
      </ode>
      <max_step_size>0.01</max_step_size>
      <real_time_factor>1</real_time_factor>
      <!-- 0 = step as fast as the CPU allows (headless eval runs faster than
           real time; the physics result is unchanged, only wall-clock speed). -->
      <real_time_update_rate>0</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type='adiabatic'/>
    <scene>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <shadows>1</shadows>
    </scene>
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <latitude_deg>0</latitude_deg>
      <longitude_deg>0</longitude_deg>
      <elevation>0</elevation>
      <heading_deg>0</heading_deg>
    </spherical_coordinates>
    <!-- Ground-truth model poses on /gazebo/model_states. Wheel odometry drifts
         under coarse/fast physics; the eval recorder uses this true pose so
         completion stays accurate at any real-time factor. -->
    <plugin name='gazebo_ros_state' filename='libgazebo_ros_state.so'>
      <ros>
        <namespace>/gazebo</namespace>
      </ros>
      <update_rate>50.0</update_rate>
    </plugin>
"""

_WALL_TEMPLATE = """    <model name='{name}'>
      <static>1</static>
      <pose frame=''>{cx:.5f} {cy:.5f} 0 0 -0 {yaw:.5f}</pose>
      <link name='link'>
        <pose frame=''>0 0 {z:.2f} 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            <box>
              <size>{length:.5f} {thickness} {height}</size>
            </box>
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            <box>
              <size>{length:.5f} {thickness} {height}</size>
            </box>
          </geometry>
          <material>
            <script>
              <uri>model://grey_wall/materials/scripts</uri>
              <uri>model://grey_wall/materials/textures</uri>
              <name>vrc/grey_wall</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
      </link>
    </model>
"""

_OBSTACLE_TEMPLATE = """    <model name='{name}'>
      <static>1</static>
      <pose frame=''>{x:.5f} {y:.5f} 0 0 -0 {yaw:.5f}</pose>
      <link name='link'>
        <pose frame=''>0 0 {z:.3f} 0 -0 0</pose>
        <collision name='collision'>
          <geometry>
            {geom}
          </geometry>
          <max_contacts>10</max_contacts>
          <surface>
            <contact>
              <ode/>
            </contact>
            <bounce/>
            <friction>
              <torsional>
                <ode/>
              </torsional>
              <ode/>
            </friction>
          </surface>
        </collision>
        <visual name='visual'>
          <cast_shadows>0</cast_shadows>
          <geometry>
            {geom}
          </geometry>
          <material>
            <script>
              <uri>file://media/materials/scripts/gazebo.material</uri>
              <name>{material}</name>
            </script>
          </material>
        </visual>
        <self_collide>0</self_collide>
        <kinematic>0</kinematic>
      </link>
    </model>
"""

_WORLD_FOOTER = """    <gui fullscreen='0'>
      <camera name='user_camera'>
        <pose frame=''>{camx:.2f} {camy:.2f} {camz:.2f} 0 1.5237 0</pose>
        <view_controller>orbit</view_controller>
        <projection_type>perspective</projection_type>
      </camera>
    </gui>
  </world>
</sdf>
"""


def _obstacle_sdf(i, o):
    """SDF model for a clutter obstacle (see clutter.Obstacle)."""
    if o.kind == "cylinder":
        geom = ("<cylinder>\n"
                f"              <radius>{o.sx / 2.0:.5f}</radius>\n"
                f"              <length>{o.sz:.5f}</length>\n"
                "            </cylinder>")
        material = "Gazebo/Orange"
    else:
        geom = ("<box>\n"
                f"              <size>{o.sx:.5f} {o.sy:.5f} {o.sz:.5f}"
                "</size>\n"
                "            </box>")
        material = "Gazebo/Wood"
    return _OBSTACLE_TEMPLATE.format(
        name=f"gen_obstacle_{i}", x=o.x, y=o.y, yaw=o.yaw,
        z=o.sz / 2.0, geom=geom, material=material)


def world_sdf(walls, comment="generated by worldgen", obstacles=()):
    parts = [_WORLD_HEADER.format(comment=comment)]
    for i, w in enumerate(walls):
        parts.append(_WALL_TEMPLATE.format(
            name=f"gen_wall_{i}", cx=w.cx, cy=w.cy, yaw=w.yaw,
            length=w.length, z=WALL_HEIGHT / 2.0,
            thickness=WALL_THICKNESS, height=WALL_HEIGHT))
    for i, o in enumerate(obstacles):
        parts.append(_obstacle_sdf(i, o))

    xs = [p[k] for w in walls for p in w.endpoints() for k in (0,)]
    ys = [p[1] for w in walls for p in w.endpoints()]
    camx, camy = (min(xs) + max(xs)) / 2.0, (min(ys) + max(ys)) / 2.0
    extent = max(max(xs) - min(xs), max(ys) - min(ys), 10.0)
    parts.append(_WORLD_FOOTER.format(camx=camx, camy=camy,
                                      camz=extent * 1.3))
    return "".join(parts)
