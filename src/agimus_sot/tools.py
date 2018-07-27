from dynamic_graph.sot.core.meta_tasks_kine import MetaTaskKine6d, MetaTaskKineCom
from dynamic_graph.sot.core.meta_tasks_kine_relative import MetaTaskKine6dRel
from dynamic_graph.sot.core.meta_tasks import setGain
from dynamic_graph.sot.core import FeaturePosture, Multiply_of_matrixHomo, Inverse_of_matrixHomo
from dynamic_graph import plug

def getTimerType (type):
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    if type == "double":
        return TimerDouble
    elif type == "matrix":
        return TimerMatrix
    elif type == "matrixhomo":
        return TimerMatrixHomo
    elif type == "vector":
        return TimerVector
    else:
        raise ValueError ("Unknown type of timer.")

def filename_escape(value):
    """
    Normalizes string, converts to lowercase, removes non-alpha characters,
    and converts spaces to hyphens.
    """
    import unicodedata, re
    value = unicodedata.normalize('NFKD', unicode(value)).encode('ascii', 'ignore')
    value = unicode(re.sub('[^\w\s-]', '', value).strip().lower())
    value = unicode(re.sub('[-\s]+', '-', value))
    return str(value)

def insertTimerOnOutput (signal, type):
    """
    Plug the signal sout of the return entity instead of `signal` to
    input signal to enable the timer.
    - param signal an output signal.
    - return an Timer entity.
    """
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    plug(signal, timer.sin)
    return timer

def insertTimer (signal, type):
    """
    - param signal a plugged input signal.
    - return an Timer entity.
    """
    assert signal.isPlugged()
    from dynamic_graph.sot.core.timer import TimerDouble, TimerMatrix, TimerMatrixHomo, TimerVector
    Timer = getTimerType (type)
    timer = Timer ("timer_of_" + signal.name)
    other = signal.getPlugged()
    plug(other, timer.sin)
    plug(timer.sout, signal)
    return timer

def parseHppName (hppjointname):
    if hppjointname == "universe": return "", "universe"
    return hppjointname.split('/', 1)

def transQuatToSE3 (p):
    from pinocchio import SE3, Quaternion
    from numpy import matrix
    return SE3 (Quaternion (p[6],p[3],p[4],p[5]).matrix(), matrix(p[0:3]).transpose())

def se3ToTuple (M):
    from dynamic_graph.sot.core.matrix_util import matrixToTuple
    return matrixToTuple (M.homogeneous)

def computeControlSelection (robot, joint_to_be_removed):
    pinmodel = robot.dynamic.model
    selection = ["1",] * pinmodel.nv
    for j in filter(lambda x: pinmodel.names[x.id] in joint_to_be_removed, pinmodel.joints[1:]):
        selection[j.idx_v:j.idx_v+j.nv] = ["0",] * j.nv
    selection.reverse()
    return "".join(selection)

class Manifold(object):
    sep = "___"

    def __init__ (self, tasks = [], constraints = [], topics = {}, initial_control = []):
        self.tasks = list(tasks)
        self.constraints = list(constraints)
        self.topics = dict(topics)
        self.initial_control = list(initial_control)

    def __add__ (self, other):
        res = Manifold(list(self.tasks), list(self.constraints), dict(self.topics))
        res += other
        return res

    def __iadd__ (self, other):
        self.tasks += other.tasks
        self.constraints += other.constraints
        for k,v in other.topics.items():
            if self.topics.has_key(k):
                a = self.topics[k]
                assert a["type"] == v["type"]
                if a.has_key('topic'): assert a["topic"] == v["topic"]
                else: assert a["handler"] == v["handler"]
                a["signalGetters"] += list(v["signalGetters"])
                # print k, "has", len(a["signalGetters"]), "signals"
            else:
                self.topics[k] = dict(v)
        self.initial_control += other.initial_control
        return self

    def setControlSelection (self, selection):
        for t in self.tasks:
            t.controlSelec.value = selection

    def pushTo (self, sot):
        for t in self.tasks:
            sot.push(t.name)
        if len(self.initial_control)>0:
            from dynamic_graph.sot.core.operator import Mix_of_vector
            ic = Mix_of_vector (sot.name + "_initial_control")
            ic.default.value = tuple ([0,] * sot.getSize())
            for func in self.initial_control:
                func (ic, sot = sot)
            plug (ic.sout, sot.q0)

class Posture(Manifold):
    def __init__ (self, name, sotrobot):
        super(Posture, self).__init__()
        from dynamic_graph.sot.core import Task, FeatureGeneric, GainAdaptive
        from dynamic_graph.sot.core.matrix_util import matrixToTuple
        from numpy import identity

        n = Posture.sep + name
        self.tp = Task ('task' + n)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture('feature_'+n)

        q = list(sotrobot.dynamic.position.value)
        self.tp.feature.state.value = q
        self.tp.feature.posture.value = q

        robotDim = sotrobot.dynamic.getDimension()
        for i in range(6, robotDim):
            self.tp.feature.selectDof (i, True)
        self.tp.gain = GainAdaptive("gain_"+n)
        self.tp.add(self.tp.feature.name)

        # Connects the dynamics to the current feature of the posture task
        plug(sotrobot.dynamic.position, self.tp.feature.state)

        self.tp.setWithDerivative (True)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        self.tasks = [ self.tp ]
        self.topics = {
                    name: {
                        "type": "vector",
                        "topic": "/hpp/target/position",
                        "signalGetters": [ self._signalPositionRef ] },
                    "vel_" + name: {
                        "type": "vector",
                        "topic": "/hpp/target/velocity",
                        "signalGetters": [ self._signalVelocityRef ] },
                }

    def _signalPositionRef (self): return self.tp.feature.posture
    def _signalVelocityRef (self): return self.tp.feature.postureDot

## Represents a gripper or a handle
class OpFrame(object):
    def __init__ (self, srdf, model = None, enabled = True):
        self.robotName = srdf["robot"]
        self.name = srdf["name"]
        self.key = self.robotName + "/" + self.name
        self.link = srdf["link"]
        pose = transQuatToSE3 (srdf["position"])
        if srdf.has_key ("joints"):
            assert model is not None
            ## Only for grippers
            self.joints = srdf["joints"]
            self._setupParentJoint (self.link, pose, model)
            self.enabled = enabled
            if srdf.has_key("torque_constant"):
                self.torque_constant = srdf["torque_constant"]
        else:
            ## Only for handles
            self.pose = pose

    def _setupParentJoint (self, link, pose, model):
        frameid = model.getFrameId (link)
        if frameid < 0 or frameid >= len(model.frames):
            raise ValueError("Link " + self.link + " not found")
        frame = model.frames[frameid]

        self.pose = frame.placement * pose
        self.joint = model.names[frame.parent]

    @property
    def fullLink  (self): return self.robotName + "/" + self.link
    @property
    def fullJoint (self): return self.robotName + "/" + self.joint
    @property
    def fullName  (self): return self.robotName + "/" + self.name

class PreGrasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(PreGrasp, self).__init__()
        self.gripper = gripper
        self.handle = handle
        if otherGraspOnObject is not None:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle  = otherGraspOnObject[1]
        else:
            self.otherGripper = None
            self.otherHandle  = None

    def makeTasks(self, sotrobot, withMeasurementOfObjectPos):
        if self.gripper.enabled:
            if self.otherGripper is not None and self.otherGripper.enabled:
                self._makeRelativeTask (sotrobot, withMeasurementOfObjectPos)
            else:
                self._makeAbsolute (sotrobot, withMeasurementOfObjectPos)
        else:
            if self.otherGripper is not None and self.otherGripper.enabled:
                self._makeAbsoluteBasedOnOther (sotrobot, withMeasurementOfObjectPos)
            else:
                # TODO Both grippers are disabled so nothing can be done...
                # add a warning ?
                pass

    def _makeAbsolute(self, sotrobot, withMeasurementOfObjectPos):
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.name, self.handle.fullName])
        self.graspTask = MetaTaskKine6d (name, sotrobot.dynamic,
                self.gripper.joint, self.gripper.joint)

        setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
        self.graspTask.task.setWithDerivative (False)

        # Current gripper position
        self.graspTask.opmodif = se3ToTuple(self.gripper.pose)

        # Express the velocities in local frame. This is the default.
        # self.graspTask.opPointModif.setEndEffector(True)

        # Desired gripper position:
        # Planned handle pose H_p = object_planned_pose * self.handle.pose
        # Real    handle pose H_r = object_real_pose    * self.handle.pose
        # Displacement        M   = H_p^-1 * H_r
        # planned gripper pose G_p= joint_planned_pose * self.gripper.pose
        # The derised position is
        # G*_r = G_p * M = G_p     * h^-1 * O_p^-1 * O_r * h
        #                = J_p * g * h^-1 * O_p^-1 * O_r * h
        self.gripper_desired_pose = Multiply_of_matrixHomo (name + "_desired")
        if withMeasurementOfObjectPos:
            # TODO Integrate measurement of h_r: position error of O_r^-1 * G_r
            # (for the release phase and computed only at time 0)
            self.gripper_desired_pose.setSignalNumber (5)
            # self.gripper_desired_pose.sin0 -> plug to joint planning pose
            self.gripper_desired_pose.sin1.value = se3ToTuple (self.gripper.pose * self.handle.pose.inverse())
            self._invert_planning_pose = Inverse_of_matrixHomo (self.handle.fullLink + "_invert_planning_pose")
            # self._invert_planning_pose.sin -> plug to object planning pose
            plug (self._invert_planning_pose.sout, self.gripper_desired_pose.sin2)
            # self.gripper_desired_pose.sin3 -> plug to object real pose
            self.gripper_desired_pose.sin4.value = se3ToTuple (self.handle.pose)

            plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)
            self.topics = {
                    self.gripper.fullJoint: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.fullJoint,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin0, ] },
                    self.handle.fullLink: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.handle.fullLink,
                        "signalGetters": [ lambda: self._invert_planning_pose.sin, ] },
                    "measurement_" + self.handle.fullLink: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "tf_listener",
                        "frame0": "world",
                        "frame1": self.handle.fullLink,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin3, ] },
                    }
        else:
            # G*_r = J_p * G
            self.gripper_desired_pose.setSignalNumber (2)
            # self.gripper_desired_pose.sin0 -> plug to joint planning pose
            self.gripper_desired_pose.sin1.value = se3ToTuple (self.gripper.pose)
            self.topics = {
                    self.gripper.fullJoint : {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.fullJoint,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin0, ] },
                    }

        plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)

        self.tasks = [ self.graspTask.task ]
        # TODO Add velocity
    
    def _makeRelativeTask (self, sotrobot, withMeasurementOfObjectPos):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.name, self.handle.fullName,
            "based", self.otherGripper.name, self.handle.fullName])
        print(name)
        self.graspTask = MetaTaskKine6dRel (name, sotrobot.dynamic,
                self.gripper.joint, self.otherGripper.joint,
                self.gripper.joint, self.otherGripper.joint)

        self.graspTask.opmodif = se3ToTuple(self.gripper.pose * self.handle.pose.inverse())
        # Express the velocities in local frame. This is the default.
        # self.graspTask.opPointModif.setEndEffector(True)

        self.graspTask.opmodifBase = se3ToTuple(self.otherGripper.pose * self.otherHandle.pose.inverse())
        # Express the velocities in local frame. This is the default.
        # self.graspTask.opPointModif.setEndEffector(True)

        setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
        self.graspTask.task.setWithDerivative (False)

        self.gripper_desired_pose      = Multiply_of_matrixHomo (name + "_desired1")
        self.otherGripper_desired_pose = Multiply_of_matrixHomo (name + "_desired2")
        if withMeasurementOfObjectPos:
            # TODO Integrate measurement of h1_r and h2_r: position error
            # O_r^-1 * G1_r and O_r^-1 * G2_r
            # (for the release phase and computed only at time 0)
            print("Relative grasp with measurement is NOT IMPLEMENTED")
        # else:
        # G2*_r = H1_p * h1^-1 * h2 = G2_p = J2_p * G
        self.gripper_desired_pose.setSignalNumber (2)
        # self.gripper_desired_pose.sin0 -> plug to joint planning pose
        self.gripper_desired_pose.sin1.value = se3ToTuple (self.gripper.pose * self.handle.pose.inverse())
        self.otherGripper_desired_pose.setSignalNumber (2)
        # self.otherGripper_desired_pose.sin0 -> plug to otherJoint planning pose
        self.otherGripper_desired_pose.sin1.value = se3ToTuple (self.otherGripper.pose * self.otherHandle.pose.inverse())
        plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)
        plug(self.otherGripper_desired_pose.sout, self.graspTask.featureDes.positionRef)
        self.topics = {
                self.gripper.fullJoint : {
                    "velocity": False,
                    "type": "matrixHomo",
                    "handler": "hppjoint",
                    "hppjoint": self.gripper.fullJoint,
                    "signalGetters": [ lambda: self.gripper_desired_pose.sin0, ] },
                self.otherGripper.fullJoint : {
                    "velocity": False,
                    "type": "matrixHomo",
                    "handler": "hppjoint",
                    "hppjoint": self.otherGripper.fullJoint,
                    "signalGetters": [ lambda: self.otherGripper_desired_pose.sin0, ] },
                }

        self.tasks = [ self.graspTask.task ]
        # TODO Add velocity

    def _makeAbsoluteBasedOnOther (self, sotrobot, withMeasurementOfObjectPos):
        assert self.handle.robotName == self.otherHandle.robotName
        assert self.handle.link      == self.otherHandle.link
        name = PreGrasp.sep.join(["", "pregrasp", self.gripper.fullName, self.handle.fullName,
            "based", self.otherGripper.name, self.otherHandle.fullName])
        self.graspTask = MetaTaskKine6d (name, sotrobot.dynamic,
                self.otherGripper.joint, self.otherGripper.joint)

        setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
        self.graspTask.task.setWithDerivative (False)

        # Current gripper position
        self.graspTask.opmodif = se3ToTuple(self.otherGripper.pose)

        # Express the velocities in local frame. This is the default.
        # self.graspTask.opPointModif.setEndEffector(True)

        # Desired gripper position:
        # Displacement gripper1: M = G1_p^-1 * G1_r
        # The derised position of handle1 is
        # H1*_r = H1_p * M = H1_p * G1_p^-1 * G1_r
        #
        # Moreover, the relative position of handle2 wrt to gripper2 may not be perfect so
        # h2_r = O_r^-1 * G2_r
        #
        # G2*_r = O*_r * h2_r = H1_p * G1_p^-1 * G1_r * h1^-1 * h2_r
        #                       O_p * h1 * G1_p^-1 * G1_r * h1^-1 * h2_r
        #                       O_p * h1 * g1^-1 * J1_p^-1 * J1_r * g1 * h1^-1 * h2_r
        # where h2_r can be a constant value of the expression above.
        self.gripper_desired_pose = Multiply_of_matrixHomo (name + "_desired")
        if withMeasurementOfObjectPos:
            # TODO Integrate measurement of h1_r and h2_r: position error
            # O_r^-1 * G1_r and O_r^-1 * G2_r
            # (for the release phase and computed only at time 0)
            self.gripper_desired_pose.setSignalNumber (5)
            ## self.gripper_desired_pose.setSignalNumber (7)
            # self.gripper_desired_pose.sin0 -> plug to object1 planning pose
            # self.gripper_desired_pose.sin1.value = se3ToTuple(self.handle.pose)
            self.gripper_desired_pose.sin1.value = se3ToTuple(self.handle.pose * self.gripper.pose.inverse())
            self._invert_planning_pose = Inverse_of_matrixHomo (self.gripper.fullLink + "_invert_planning_pose")
            # self._invert_planning_pose.sin -> plug to link1 planning pose
            plug (self._invert_planning_pose.sout, self.gripper_desired_pose.sin2)
            # self.gripper_desired_pose.sin3 -> plug to link1 real pose

            # self.gripper_desired_pose.sin4.value = se3ToTuple (self.handle.pose.inverse() * self.otherHandle.pose)
            self.gripper_desired_pose.sin4.value = se3ToTuple (self.gripper.pose * self.handle.pose.inverse() * self.otherHandle.pose)
            ## self.gripper_desired_pose.sin4.value = se3ToTuple (self.gripper.pose * self.handle.pose.inverse())
            ## self.gripper_desired_pose.sin5 -> plug to object real pose
            ## if self.otherGripper.joint not in sotrobot.dyn.signals():
            ##    sotrobot.dyn.createOpPoint (self.otherGripper.joint, self.otherGripper.joint)
            ## plug(sotrobot.dyn.signal(self.otherGripper.joint), self.gripper_desired_pose.sin6)

            plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)
            self.topics = {
                    self.handle.fullLink: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.handle.fullLink,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin0, ] },
                    self.gripper.fullLink: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.gripper.fullLink,
                        "signalGetters": [ lambda: self._invert_planning_pose.sin, ] },
                    "measurement_" + self.gripper.fullLink: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "tf_listener",
                        "frame0": "world",
                        "frame1": self.gripper.fullLink,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin3, ] },
                    ## "measurement_" + self.otherHandle.fullLink: {
                        ## "velocity": False,
                        ## "type": "matrixHomo",
                        ## "handler": "tf_listener",
                        ## "frame0": "world",
                        ## "frame1": self.otherHandle.fullLink,
                        ## "signalGetters": [ self.gripper_desired_pose.sin5 ] },
                    }
        else:
            # G2*_r = H1_p * h1^-1 * h2 = G2_p = J2_p * G
            self.gripper_desired_pose.setSignalNumber (2)
            # self.gripper_desired_pose.sin0 -> plug to joint planning pose
            self.gripper_desired_pose.sin1.value = se3ToTuple (self.otherGripper.pose)
            self.topics = {
                    self.otherGripper.fullJoint : {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": self.otherGripper.fullJoint,
                        "signalGetters": [ lambda: self.gripper_desired_pose.sin0, ] },
                    }

        plug(self.gripper_desired_pose.sout, self.graspTask.featureDes.position)

        self.tasks = [ self.graspTask.task ]
        # TODO Add velocity

class Grasp (Manifold):
    def __init__ (self, gripper, handle, otherGraspOnObject = None):
        super(Grasp, self).__init__()
        self.gripper = gripper
        self.handle = handle

        # self.relative = otherGraspOnObject is not None \
                # and otherGraspOnObject.handle.link == self.handle.link
        # TODO: We should make sure that the relative position is constant
        # but we have no way to do it (HPP does). We assume that the object is
        # not articulated.
        self.relative = (otherGraspOnObject is not None)
        if self.relative:
            self.otherGripper = otherGraspOnObject[0]
            self.otherHandle = otherGraspOnObject[1]

    def makeTasks(self, sotrobot):
        if self.relative:
            self.graspTask = MetaTaskKine6dRel (
                    Grasp.sep + self.gripper.name + Grasp.sep + self.otherGripper.name,
                    sotrobot.dynamic,
                    self.gripper.joint,
                    self.otherGripper.joint,
                    self.gripper.joint,
                    self.otherGripper.joint)

            M = se3ToTuple(self.gripper.pose * self.handle.pose.inverse())
            self.graspTask.opPointModif.activ = True
            self.graspTask.opPointModif.setTransformation (M)
            # Express the velocities in local frame.
            # This is the default.
            # self.graspTask.opPointModif.setEndEffector(True)

            M = se3ToTuple(self.otherGripper.pose * self.otherHandle.pose.inverse())
            self.graspTask.opPointModifBase.activ = True
            self.graspTask.opPointModifBase.setTransformation (M)
            # Express the velocities in local frame.
            # This is the default.
            # self.graspTask.opPointModif.setEndEffector(True)

            # Plug the signals
            plug(self.graspTask.opPointModif    .signal('position'),self.graspTask.feature.position )
            plug(self.graspTask.opPointModif    .signal('jacobian'),self.graspTask.feature.Jq)
            plug(self.graspTask.opPointModifBase.signal('position'),self.graspTask.feature.positionRef )
            plug(self.graspTask.opPointModifBase.signal('jacobian'),self.graspTask.feature.JqRef)
            
            setGain(self.graspTask.gain,(4.9,0.9,0.01,0.9))
            self.graspTask.task.setWithDerivative (False)

            self.tasks = [ self.graspTask.task ]

            # TODO Add velocity
            # self.graspTask.task.setWithDerivative (True)
            # which needs the following signals:
            # - velocity of object attached to gripper.sotjoint, into self.graspTask.feature.dotposition
            # - velocity of object attached to otherGripper.sotjoint, into self.graspTask.feature.dotpositionRef

class EndEffector (Manifold):
    def __init__ (self, sotrobot, gripper, name_suffix):
        from dynamic_graph.sot.core import Task, GainAdaptive

        super(EndEffector, self).__init__()
        self.gripper = gripper
        self.jointNames = gripper.joints
        self.robot = sotrobot
        pinmodel = sotrobot.dynamic.model

        self.name = Posture.sep.join(["ee", gripper.name, name_suffix])

        self.tp = Task ('task' + self.name)
        self.tp.dyn = sotrobot.dynamic
        self.tp.feature = FeaturePosture ('feature_' + self.name)

        plug(sotrobot.dynamic.position, self.tp.feature.state)

        # Select the dofs
        self.tp.feature.posture.value = sotrobot.dynamic.position.value
        # Define the reference and the selected DoF
        self.jointRanks = []
        for name in self.jointNames:
            idJ = pinmodel.getJointId(name)
            assert idJ < pinmodel.njoints
            joint = pinmodel.joints[idJ]
            idx_v = joint.idx_v
            nv = joint.nv
            self.jointRanks.append( (idx_v, nv) )
        for idx_v, nv in self.jointRanks:
            for i in range(idx_v, idx_v + nv):
                self.tp.feature.selectDof (i, True)

        self.tp.gain = GainAdaptive("gain_"+self.name)
        self.tp.add(self.tp.feature.name)

        # Set the gain of the posture task
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))
        plug(self.tp.gain.gain, self.tp.controlGain)
        plug(self.tp.error, self.tp.gain.error)
        if len(self.jointNames) > 0:
            self.tasks = [ self.tp ]

    ### \param type equals "open" or "close"
    ### \param period interval between two integration of SoT
    def makeAdmittanceControl (self, affordance, type, period, simulateTorqueFeedback = False):
        # Make the admittance controller
        from agimus_sot.control.gripper import AdmittanceControl
        # type = "open" or "close"
        desired_torque = affordance.ref["torque"]
        estimated_theta_close = affordance.ref["angle_close"]
        wn, z, alpha, tau, = affordance.getControlParameter ()

        self.ac = AdmittanceControl ("AC_" + self.name + "_" + type,
                estimated_theta_close,
                desired_torque, period,
                alpha, tau,)

        if simulateTorqueFeedback:
            # Get torque from an internal simulation
            M,d,k,x0 = affordance.getSimulationParameters()
            self.ac.setupFeedbackSimulation(M,d,k,x0)
            # Must be done after setupFeedbackSimulation
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
        else:
            self.ac.readPositionsFromRobot(self.robot, self.jointNames)
            # Get torque from robot sensors (or external simulation)
            # TODO allows to switch between current and torque sensors
            self.ac.readCurrentsFromRobot(self.robot, self.jointNames, (self.gripper.torque_constant,))
            # self.ac.readTorquesFromRobot(self.robot, self.jointNames)

        from dynamic_graph.sot.core.operator import Mix_of_vector
        mix_of_vector = Mix_of_vector (self.name + "_control_to_robot_control")
        mix_of_vector.default.value = tuple ([0,] * len(self.tp.feature.posture.value))
        mix_of_vector.setSignalNumber(2)
        for idx_v,nv in self.jointRanks:
            mix_of_vector.addSelec(1, idx_v, nv)

        # Plug the admittance controller to the posture task
        setGain(self.tp.gain,1.)
        plug(self.ac.outputPosition, mix_of_vector.signal("sin1"))
        plug(mix_of_vector.sout, self.tp.feature.posture)
        # TODO plug posture dot ?
        # I do not think it is necessary.
        # TODO should we send to the posture task
        # - the current robot posture
        # - the postureDot from self.ac.outputDerivative
        # This would avoid the last integration in self.ac.
        # This integration does not know the initial point so
        # there might be some drift (removed the position controller at the beginning)

    def makePositionControl (self, position):
        q = list(self.tp.feature.state.value)
        # Define the reference
        ip = 0
        for iq, nv in self.jointRanks:
            q[iq:iq+nv] = position[ip:ip+nv]
            ip += nv
        assert ip == len(position)
        self.tp.feature.posture.value = q
        setGain(self.tp.gain,(4.9,0.9,0.01,0.9))

class Foot (Manifold):
    def __init__ (self, footname, sotrobot, selec='111111'):
        robotname, sotjoint = parseHppName (footname)
        self.taskFoot = MetaTaskKine6d(
                Foot.sep + footname,
                sotrobot.dynamic,sotjoint,sotjoint)
        self.taskFoot.feature.selec.value = selec
        super(Foot, self).__init__(
                tasks = [ self.taskFoot.task, ],
                topics = {
                    footname: {
                        "velocity": False,
                        "type": "matrixHomo",
                        "handler": "hppjoint",
                        "hppjoint": footname,
                        "signalGetters": [ self._signalPositionRef ] },
                    # "vel_" + self.gripper.name: {
                    "vel_" + footname: {
                        "velocity": True,
                        "type": "vector",
                        "handler": "hppjoint",
                        "hppjoint": footname,
                        "signalGetters": [ self._signalVelocityRef ] },
                    })
        self.taskFoot.gain.value = 5

    def _signalPositionRef (self): return self.taskFoot.featureDes.position
    def _signalVelocityRef (self): return self.taskFoot.featureDes.velocity

class COM (Manifold):
    sep = "_com_"
    def __init__ (self, comname, sotrobot):
        self.taskCom = MetaTaskKineCom (sotrobot.dynamic,
                name = COM.sep + comname)
        self.taskCom.task.setWithDerivative (True)
        super(COM, self).__init__(
                tasks = [ self.taskCom.task, ],
                topics = {
                    comname: {
                        "velocity": False,
                        "type": "vector3",
                        "handler": "hppcom",
                        "hppcom": comname,
                        "signalGetters": [ self._signalPositionRef ] },
                    "vel_" + comname: {
                        "velocity": True,
                        "type": "vector3",
                        "handler": "hppcom",
                        "hppcom": comname,
                        "signalGetters": [ self._signalVelocityRef ] },
                    })
        self.taskCom.task.controlGain.value = 5

    def _signalPositionRef (self): return self.taskCom.featureDes.errorIN
    def _signalVelocityRef (self): return self.taskCom.featureDes.errordotIN
