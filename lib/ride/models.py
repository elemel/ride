class Model(object):
    def __init__(self, id=None):
        self.id = id

class LevelModel(Model):
    def __init__(self):
        super(LevelModel, self).__init__()
        self.lower_bound = -100, -100
        self.upper_bound = 100, 100
        self.gravity = 0, -10
        self.start = -50, 0
        self.goal = 50, 0
        self.body_models = []
        self.joint_models = []

class BodyModel(Model):
    def __init__(self):
        super(BodyModel, self).__init__()
        self.shape_models = []

class ShapeModel(Model):
    def __init__(self, density=0, friction=0.5, restitution=0.5):
        super(ShapeModel, self).__init__()
        self.density = density
        self.friction = friction
        self.restitution = restitution
        self.group_index = 0

class CircleModel(ShapeModel):
    def __init__(self, center=(0, 0), radius=1, **kwargs):
        super(CircleModel, self).__init__(**kwargs)
        self.center = center
        self.radius = radius

class PolygonModel(ShapeModel):
    def __init__(self, vertices=(), **kwargs):
        super(PolygonModel, self).__init__(**kwargs)
        self.vertices = []
        self.vertices[:] = vertices

class JointModel(Model):
    pass

class RevoluteJointModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor=(0, 0)):
        super(RevoluteJointModel, self).__init__()
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor = anchor

class DistanceJointModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor_1=(0, 0),
                 anchor_2=(0, 0)):
        super(DistanceJointModel, self).__init__()
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2

class PrismaticJointModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor_1=(0, 0),
                 anchor_2=(0, 0)):
        super(PrismaticJointModel, self).__init__()
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2

class MotorModel(JointModel):
    def __init__(self, body_model=None, anchor=(0, 0), torque=1, damping=0,
                 clockwise_key=None, counter_clockwise_key=None):
        super(MotorModel, self).__init__()
        self.body_model = body_model
        self.anchor = anchor
        self.torque = torque
        self.damping = damping
        self.clockwise_key = clockwise_key
        self.counter_clockwise_key = counter_clockwise_key

class SpringModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor_1=(0, 0),
                 anchor_2=(0, 0), spring_constant=1, damping=0):
        super(SpringModel, self).__init__()
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2
        self.spring_constant = spring_constant
        self.damping = damping

class CameraModel(JointModel):
    def __init__(self, body_model=None, anchor=(0, 0)):
        super(CameraModel, self).__init__()
        self.body_model = body_model
        self.anchor = anchor
