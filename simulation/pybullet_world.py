import time
import pybullet as p
import pybullet_data
from pybullet import connect


#这个类只负责构建仿真世界
class PyBulletWorld:
      def __init__(self,render=True):
          self.render = render
          self.client_id = None
          self.plane_id = None
          self.obstacle_ids = []

      def connect(self):
          if self.render:
              self.client_id = p,connect(p.GUI)
          else:
              self.client_id = p.connect(p.DIRECT)
          p.setAdditionalSearchPath(pybullet_data.getDataPath())
          p.setGravity(0,0,-9.8)

      def reset(self):
          p.resetSimulation()
          p.setAdditionalSearchPath(pybullet_data.getDataPath())
          p.setGravity(0,0,-9.8)
          self.plane_id = None
          self.obstacle_ids = []

      def load_plane(self):
          self.plane_id = p.loadURDF("plane.urdf")
          return self.plane_id

      def load_box(
              self,
              position,
              size,
              orientation=(0,0,0,1),
              color=(0.6,0.6,0.6,1.0),
              fixed_base=True):
          half_extents = [
              size[0]/2.0,
              size[1]/2.0,
              size[2]/2.0,
          ]
          collision_shape = p.createCollisionShape(
              shapeType=p.GEOM_BOX,
              halfExtents=half_extents,
          )
          visual_shape = p.createVisualShape(
              shapeType=p.GEOM_BOX,
              halfExtents=half_extents,
              rgbaColor=color,
          )
          mass = 0 if fixed_base else 1
          body_id = p.createMultiBody(
              baseMass=mass,
              baseCollisionShapeIndex=collision_shape,
              baseVisualShapeIndex=visual_shape,
              basePosition=position,
              baseOrientation=orientation,
          )
          self.obstacle_ids.append(body_id)

          return body_id

      def load_obstacles(self,obstacle_configs):
          for obstacle in obstacle_configs:
              obstacle_type = obstacle.get("type","box")
              if obstacle_type == "box":
                  self.load_box(
                      position=obstacle["position"],
                      size=obstacle["size"],
                      orientation=obstacle.get("orientation",[0,0,0,1]),
                      color=obstacle.get("color",[0.6,0.6,0.6,1.0]),
                      fixed_base=obstacle.get("fixed_base",True),
                  )
              else:
                  raise ValueError(f"Unsupported obstacle type: {obstacle_type}")
          return self.obstacle_ids

      def get_obstacles(self,include_plane=True):
          obstacles = list(self.obstacle_ids)
          if include_plane and self.plane_id is not None:
              obstacles.append(self.plane_id)
          return obstacles

      def make_collision_fn(self,robot_group,self_collisions=True):
          return robot_group.get_collision_fn(
              obstacles=self.get_obstacles(include_plane=True),
              attachments=[],
              self_collisions=self_collisions,
              disabled_collisions=set(),
          )[0]


      def step(self,steps=1,sleep=0.0):
          for _ in range(steps):
              p.stepSimulation()
              if sleep > 0:
                  time.sleep(sleep)

      def disconnect(self):
          if self.client_id is not None:
              p.disconnect(self.client_id)
              self.client_id = None







