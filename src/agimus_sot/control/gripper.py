from dynamic_graph import plug

class AdmittanceControl:
    """
    The torque controller is then use to maintain a desired force.
    It outputs a velocity command to be sent to entity Device.
    """
    def __init__ (self, name, estimated_theta_closed, desired_torque, period, nums, denoms):
        """
        - estimated_theta_closed: Use for the initial position control. It should correspond to a configuration in collision.
                                  The closer to contact configuration, the least the overshoot.
        - desired_torque: The torque to be applied on the object.
        - period: The SoT integration time.
        - nums, denoms: coefficient of the controller:
           \sum_i denoms[i] * d^i theta / dt^i = \sum_j nums[j] d^j torque / dt^j
        """
        self.name = name
        self.est_theta_closed = estimated_theta_closed
        self.desired_torque = desired_torque
        self.dt = period

        self._makeTorqueControl (nums, denoms)

        self._makeIntegrationOfVelocity ()

    ### Feed-forward - contact phase
    def _makeTorqueControl (self, nums, denoms):
        from agimus_sot.control.controllers import Controller
        self.torque_controller = Controller (self.name + "_torque", nums, denoms, self.dt, [0. for _ in self.est_theta_closed])
        self.torque_controller.addFeedback()
        self.torque_controller.reference.value = self.desired_torque

    ### Internal method
    def _makeIntegrationOfVelocity (self):
        from dynamic_graph.sot.core import Add_of_vector
        self.omega2theta = Add_of_vector (self.name + "_omega2theta")
        self.omega2theta.setCoeff2(self.dt)
        # self.omega2theta.sin1 <- current position
        # self.omega2theta.sin2 <- desired velocity
        plug (self.torque_controller.output, self.omega2theta.sin2)

    ### Setup event to tell when object is grasped
    def setupFeedbackSimulation (self, mass, damping, spring, theta0):
        from agimus_sot.control.controllers import Controller
        from dynamic_graph.sot.core import Add_of_vector
        from agimus_sot import DelayVector

        ## omega -> theta
        # Done in _makeControllerSwich
        # A delay is necessary to avoid loops in SoT

        delayTheta = DelayVector (self.name + "_sim_theta_delay")
        delayTheta.setMemory (tuple([0. for _ in self.est_theta_closed]))
        plug (self.omega2theta.sout, delayTheta.sin)
        self.setCurrentPositionIn(delayTheta.previous)

        ## theta -> phi = theta - theta0
        self.theta2phi = Add_of_vector(self.name + "_sim_theta2phi")
        self.theta2phi.setCoeff1 ( 1)
        self.theta2phi.setCoeff2 (-1)
        plug (delayTheta.current, self.theta2phi.sin1)
        self.theta2phi.sin2.value = theta0

        ## phi -> torque
        from dynamic_graph.sot.core.switch import SwitchVector
        from dynamic_graph.sot.core.operator import CompareVector
        # reverse = self.theta_open[0] > theta0[0]
        reverse = self.desired_torque[0] < 0
        self.sim_contact_condition = CompareVector(self.name + "_sim_contact_condition")
        self.sim_contact_condition.setTrueIfAny(False)

        self.sim_switch = SwitchVector (self.name + "_sim_torque")
        self.sim_switch.setSignalNumber(2)

        plug (self.sim_contact_condition.sout, self.sim_switch.boolSelection)

        # Non contact phase
        if reverse:
            plug (self.theta2phi.sout, self.sim_contact_condition.sin2)
            self.sim_contact_condition.sin1.value = [0. for _ in self.est_theta_closed]
        else:
            plug (self.theta2phi.sout, self.sim_contact_condition.sin1)
            self.sim_contact_condition.sin2.value = [0. for _ in self.est_theta_closed]
        # Contact phase
        self.phi2torque = Controller (self.name + "_sim_phi2torque",
                (spring, damping, mass,), (1.,),
                self.dt, [0. for _ in self.est_theta_closed])
        #TODO if M != 0: phi2torque.pushNumCoef(((M,),))
        plug (self.theta2phi.sout, self.phi2torque.reference)

        # Condition
        # if phi < 0 -> no contact -> torque = 0
        self.sim_switch.sin1.value = [0. for _ in self.est_theta_closed]
        # else       ->    contact -> phi2torque
        plug (self.phi2torque.output, self.sim_switch.sin0)

        delay = DelayVector (self.name + "_sim_torque_delay")
        delay.setMemory (tuple([0. for _ in self.est_theta_closed]))
        # plug (self.phi2torque.output, delay.sin)
        plug (self.sim_switch.sout, delay.sin)
        # self.setCurrentConditionIn (delay.current)
        plug (delay.previous, self.currentTorqueIn)

    def readPositionsFromRobot (self, robot, jointNames):
        # TODO Compare current position to self.est_theta_closed and
        # so as not to overshoot this position.
        # Input formattting
        from dynamic_graph.sot.core.operator import Selec_of_vector
        self. _joint_selec = Selec_of_vector (self.name + "_joint_selec")
        model = robot.dynamic.model
        for jn in jointNames:
            jid = model.getJointId (jn)
            assert jid < len(model.joints)
            jmodel = model.joints[jid]
            self. _joint_selec.addSelec (jmodel.idx_v,jmodel.idx_v + jmodel.nv)
        plug (robot.dynamic.position, self. _joint_selec.sin)
        self.setCurrentPositionIn(self._joint_selec.sout)

    def readCurrentsFromRobot (self, robot, jointNames, torque_constants):
        # Input formattting
        from dynamic_graph.sot.core.operator import Selec_of_vector
        self._current_selec = Selec_of_vector (self.name + "_current_selec")
        model = robot.dynamic.model
        for jn in jointNames:
            jid = model.getJointId (jn)
            assert jid < len(model.joints)
            jmodel = model.joints[jid]
            # TODO there is no value for the 6 first DoF
            assert jmodel.idx_v >= 6
            self._current_selec.addSelec (jmodel.idx_v-6,jmodel.idx_v-6 + jmodel.nv)

        from dynamic_graph.sot.core.operator import Multiply_of_vector
        plug (robot.device.currents, self._current_selec.sin)
        self._multiply_by_torque_constants = Multiply_of_vector (self.name + "_multiply_by_torque_constants")
        self._multiply_by_torque_constants.sin0.value = torque_constants
        plug (self._current_selec.sout, self._multiply_by_torque_constants.sin1)

        plug (self._multiply_by_torque_constants.sout, self.currentTorqueIn)

    def readTorquesFromRobot (self, robot, jointNames):
        # Input formattting
        from dynamic_graph.sot.core.operator import Selec_of_vector
        self._torque_selec = Selec_of_vector (self.name + "_torque_selec")
        model = robot.dynamic.model
        for jn in jointNames:
            jid = model.getJointId (jn)
            assert jid < len(model.joints)
            jmodel = model.joints[jid]
            # TODO there is no value for the 6 first DoF
            assert jmodel.idx_v >= 6
            self._torque_selec.addSelec (jmodel.idx_v-6,jmodel.idx_v-6 + jmodel.nv)
        plug (robot.device.ptorques, self._torque_selec.sin)

        plug (self._torque_selec.sout, self.currentTorqueIn)

    # TODO remove me
    def addOutputTo (self, robot, jointNames, mix_of_vector, sot=None):
        #TODO assert isinstance(mix_of_vector, Mix_of_vector)
        i = mix_of_vector.getSignalNumber()
        mix_of_vector.setSignalNumber(i+1)
        plug (self.outputVelocity, mix_of_vector.signal("sin"+str(i)))
        model = robot.dynamic.model
        for jn in jointNames:
            jid = model.getJointId (jn)
            jmodel = model.joints[jid]
            mix_of_vector.addSelec(i, jmodel.idx_v, jmodel.nv)

    def addTracerRealTime (self, robot):
        from dynamic_graph.tracer_real_time import TracerRealTime
        from agimus_sot.tools import filename_escape
        self._tracer = TracerRealTime (self.name + "_tracer")
        self._tracer.setBufferSize (10 * 1048576) # 10 Mo
        self._tracer.open ("/tmp", filename_escape(self.name), ".txt")

        self._tracer.add (self.omega2theta.name + "sin1",         "_theta_current")
        self._tracer.add (self.omega2theta.name + "sin2",         "_omega")
        self._tracer.add (self.omega2theta.name + "sout",         "_theta_desired")
        self._tracer.add (self.torque_controller.referenceName,   "_reference_torque")
        self._tracer.add (self.torque_controller.measurementName, "_measured_torque")

        robot.device.after.addSignal(self._tracer.name + ".triger")
        return self._tracer

    @property
    def outputPosition (self):
        return self.omega2theta.output

    @property
    def outputVelocity (self):
        return self.torque_controller.output

    @property
    def referenceTorqueIn (self):
        return self.torque_controller.reference

    def setCurrentPositionIn (self, sig):
        plug (sig, self.omega2theta.sin1)

    @property
    def currentTorqueIn (self):
        return self.torque_controller.measurement

    @property
    def torqueConstants (self):
        return self._multiply_by_torque_constants.sin0

class PositionAndAdmittanceControl:
    """
    Encapsulate two controllers: a position controller and a torque controller.
    The position controller is used to create a contact.
    The torque controller is then use to maintain a desired force.
    Both controller outputs a velocity command to be sent to entity Device.
    """
    def __init__ (self, name, theta_open, estimated_theta_closed, desired_torque, period,
            threshold_up, threshold_down,
            wn, z,
            nums_tor, denoms_tor,
            ):
        """
        - theta_open:             Angle for the opened gripper.
        - estimated_theta_closed: Use for the initial position control. It should correspond to a configuration in collision.
                                  The closer to contact configuration, the least the overshoot.
        - desired_torque: The torque to be applied on the object.
        - period: The SoT integration time.
        - threshold_up  : When one component of the torque becomes greater than threshold, switch to torque control
        - threshold_down: When all components of the torque become less    than threshold, switch to position control
        - wn, z: corner frequency and damping of the second order position control.
        - nums_tor, denoms_tor: coefficient of the admittance controller:
           \sum_i denoms[i] * d^i theta / dt^i = \sum_j nums[j] d^j torque / dt^j
        """
        assert desired_torque[0] * (estimated_theta_closed[0]-theta_open[0]) > 0,\
                "Incompatible desired positions and torques."
        super(PositionAndAdmittanceControl, self).__init__(name, estimated_theta_closed,
                desired_torque, period, nums, denoms)

        self.theta_open = theta_open
        self.threshold_up = threshold_up
        self.threshold_down = threshold_down

        self._makePositionControl (wn, z)

        self._makeControllerSwich ()

    def resetToPositionControl (self):
        self.switch.latch.turnOff()

    ### Feed-forward - non-contact phase
    def _makePositionControl (self, wn, z):
        """
        the control reaches a precision of 5% at
        * z = 1: t = - log(0.05) / wn
        * z < 1: t = - log(0.05 * sqrt(1-z**2)) / (z * wn),
        """
        from agimus_sot.control.controllers import secondOrderClosedLoop
        self.position_controller = secondOrderClosedLoop (self.name + "_position", wn, z, self.dt, [0. for _ in self.est_theta_closed])
        self.position_controller.reference.value = self.est_theta_closed

    ### Setup switch between the two control scheme
    def _makeControllerSwich (self):
        from agimus_sot.control.switch import ControllerSwitch
        from agimus_sot.control.controllers import Controller

        self.switch = ControllerSwitch (self.name + "_switch",
                # Outputs a velocity
                (self.position_controller.outputDerivative, self.torque_controller.output),
                # Outputs a position
                # (self.position_controller.output, self.torque_controller.output),
                self.threshold_up, self.threshold_down)

        plug (self.switch.signalOut, self.omega2theta.sin2)

    ### Setup event to tell when object is grasped
    def setupFeedbackSimulation (self, mass, damping, spring, theta0):
        super(PositionAndAdmittanceControl, self).setupFeedbackSimulation(mass, damping, spring, theta0)
        self.setCurrentConditionIn (delay.current)

    def readCurrentsFromRobot (self, robot, jointNames, torque_constants):
        super(PositionAndAdmittanceControl, self).readCurrentsFromRobot (robot, jointNames, torque_constants)
        self.setCurrentConditionIn (self._multiply_by_torque_constants.sout)

    def readTorquesFromRobot (self, robot, jointNames):
        super(PositionAndAdmittanceControl, self).readTorquesFromRobot (robot, jointNames)
        self.setCurrentConditionIn (self._torque_selec.sout)

    def addTracerRealTime (self, robot):
        tracer = super(PositionAndAdmittanceControl, self).addTracerRealTime (robot)
        # self._tracer.add (self.theta2phi.sout,                    "_phi")
        # self._tracer.add (self.currentConditionIn.name,   # Measured torque
                # filename_escape(self.name + "_"))

        # self._tracer.add (self.switch._condition_up.sout)
        # self._tracer.add (self.switch._condition_down.sout)
        return tracer

    @property
    def outputVelocity (self):
        return self.switch.signalOut

    @property
    def referencePositionIn (self):
        return self.position_controller.reference

    def setCurrentPositionIn (self, sig):
        super(PositionAndAdmittanceControl, self).setCurrentPositionIn(sig)
        plug (sig, self.position_controller.measurement)

    def setCurrentConditionIn (self,sig):
        return self.switch.setMeasurement(sig)

    @property
    def switchEventToTorqueCheck (self):
        return self.switch.eventUp.check

    @property
    def switchEventToPositionCheck (self):
        return self.switch.eventDown.check

# vim: set foldmethod=indent
