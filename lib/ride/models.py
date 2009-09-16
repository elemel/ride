class Model(object):
    pass

class LevelModel(Model):
    def __init__(self):
        self.lower_bound = -100, -100
        self.upper_bound = 100, 100
        self.gravity = 0, -10
        self.start = -50, 0
        self.goal = 50, 0
        self.body_models = set()
        self.joint_models = set()

class BodyModel(Model):
    def __init__(self):
        self.shape_models = set()

class ShapeModel(Model):
    def __init__(self, density=0, friction=0.5, restitution=0.5):
        self.density = density
        self.friction = friction
        self.restitution = restitution

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

class DistanceJointModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor_1=(0, 0),
                 anchor_2=(0, 0)):
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2

class MotorModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, torque=1,
                 damping=0):
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.torque = torque
        self.damping = damping

class SpringModel(JointModel):
    def __init__(self, body_model_1=None, body_model_2=None, anchor_1=(0, 0),
                 anchor_2=(0, 0), spring_constant=1, damping_constant=0):
        self.body_model_1 = body_model_1
        self.body_model_2 = body_model_2
        self.anchor_1 = anchor_1
        self.anchor_2 = anchor_2
        self.spring_constant = spring_constant
        self.damping_constant = damping_constant
