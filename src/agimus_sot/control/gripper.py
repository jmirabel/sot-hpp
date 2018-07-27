from dynamic_graph import plug

class AdmittanceControl:
    """
    The torque controller is then use to maintain a desired force.
    It outputs a velocity command to be sent to entity Device.
    """
    def __init__ (self, name, estimated_theta_closed, desired_torque, period, alpha = 1., tau = 1.,):
        """
        - estimated_theta_closed: Use for the initial position control. It should correspond to a configuration in collision.
                                  The closer to contact configuration, the least the overshoot.
        - desired_torque: The torque to be applied on the object.
        - period: The SoT integration time.
        - alpha, tau: amplitude and time constant of the first order torque control.
        """
        # assert desired_torque[0] * (estimated_theta_closed[0]-theta_open[0]) > 0,\
                # "Incompatible desired positions and torques."
        self.name = name
        self.est_theta_closed = estimated_theta_closed
        self.desired_torque = desired_torque
        self.dt = period

        self._makeTorqueControl (alpha, tau)

    #TODO I think this is not needed as currently admittance_control is used only
    # to close a gripper.
    def setGripperClosed (self):
        self.latch_gripper_closed.turnOn()

    ### Feed-forward - contact phase
    def _makeTorqueControl (self, alpha, tau):
        from agimus_sot.control.controllers import Controller
        self.torque_controller = Controller (self.name + "_torque", (alpha,), (1., tau), self.dt, [0. for _ in self.est_theta_closed])
        self.torque_controller.addFeedback()
        self.torque_controller.reference.value = self.desired_torque

        self.omega2theta = Controller (self.name + "_sim_omega2theta",
                (1.,), (0., 1.), self.dt, [0. for _ in self.est_theta_closed])
        plug (self.torque_controller.output, self.omega2theta.reference)

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
        plug (self.omega2theta.output, delayTheta.sin)
        plug (delayTheta.previous, self.currentPositionIn)

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
        # from dynamic_graph.sot.core.operator import Selec_of_vector
        # self. _joint_selec = Selec_of_vector (self.name + "_joint_selec")
        # model = robot.dynamic.model
        # for jn in jointNames:
            # jid = model.getJointId (jn)
            # assert jid < len(model.joints)
            # jmodel = model.joints[jid]
            # self. _joint_selec.addSelec (jmodel.idx_v,jmodel.idx_v + jmodel.nv)
        # plug (robot.dynamic.position, self. _joint_selec.sin)
        # plug (self. _joint_selec.sout, self.currentPositionIn)
        pass

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

        # self._tracer.add (self.switch.latch.name + ".out",        "_torque_control_activated")
        self._tracer.add (self.omega2theta.referenceName,         "_omega")
        self._tracer.add (self.omega2theta.outputName,            "_theta")
        # self._tracer.add (self.theta2phi.sout,                    "_phi")
        self._tracer.add (self.torque_controller.referenceName,   "_reference_torque")
        self._tracer.add (self.torque_controller.measurementName, "_measured_torque")
        # self._tracer.add (self.currentConditionIn.name,   # Measured torque
                # filename_escape(self.name + "_"))

        # self._tracer.add (self.switch._condition_up.sout)
        # self._tracer.add (self.switch._condition_down.sout)
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

    # TODO
    # @property
    # def currentPositionIn (self):
        # return self.position_controller.measurement

    @property
    def currentTorqueIn (self):
        return self.torque_controller.measurement

    @property
    def torqueConstants (self):
        return self._multiply_by_torque_constants.sin0

class AdmittanceControl2:
    """
    Encapsulate two controllers: a position controller and a torque controller.
    The position controller is used to create a contact.
    The torque controller is then use to maintain a desired force.
    Both controller outputs a velocity command to be sent to entity Device.
    """
    def __init__ (self, name, theta_open, estimated_theta_closed, desired_torque, period,
            threshold_up, threshold_down,
            wn = 10., z = 1., alpha = 1., tau = 1.,
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
        - alpha, tau: amplitude and time constant of the first order torque control.
        """
        assert desired_torque[0] * (estimated_theta_closed[0]-theta_open[0]) > 0,\
                "Incompatible desired positions and torques."
        self.name = name
        self.theta_open = theta_open
        self.est_theta_closed = estimated_theta_closed
        self.desired_torque = desired_torque
        self.threshold_up = threshold_up
        self.threshold_down = threshold_down
        self.dt = period

        self._makePositionControl (wn, z)

        self._makeTorqueControl (alpha, tau)

        self._makeControllerSwich ()

    def resetToPositionControl (self):
        self.switch.latch.turnOff()

    #TODO I think this is not needed as currently admittance_control is used only
    # to close a gripper.
    def setGripperOpen (self):
        self.latch_gripper_closed.turnOff()

    #TODO I think this is not needed as currently admittance_control is used only
    # to close a gripper.
    def setGripperClosed (self):
        self.latch_gripper_closed.turnOn()

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

    ### Feed-forward - contact phase
    def _makeTorqueControl (self, alpha, tau):
        from agimus_sot.control.controllers import Controller
        self.torque_controller = Controller (self.name + "_torque", (alpha,), (1., tau), self.dt, [0. for _ in self.est_theta_closed])
        self.torque_controller.addFeedback()
        self.torque_controller.reference.value = self.desired_torque

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

        # TODO this assumes that the current position is 0
        # Should we use self.theta_open instead ? But what if
        # the same gripper is involved in different grasp with different
        # theta_open ?
        self.omega2theta = Controller (self.name + "_sim_omega2theta",
                (1.,), (0., 1.), self.dt, [0. for _ in self.est_theta_closed])
        plug (self.switch.signalOut, self.omega2theta.reference)

        #TODO I think this is not needed as currently admittance_control is used only
        # to close a gripper.
        from dynamic_graph.sot.core.switch import SwitchVector
        self.switch_input_position = SwitchVector (self.name + "_switch_input_position")
        self.switch_input_torque   = SwitchVector (self.name + "_switch_input_torque")
        self.switch_input_position.setSignalNumber(2)
        self.switch_input_torque  .setSignalNumber(2)
        # References for open gripper
        self.switch_input_position.sin0.value = self.theta_open
        self.switch_input_torque  .sin0.value = tuple([ 0., ] * len(self.desired_torque))
        # References for closed gripper
        self.switch_input_position.sin1.value = self.est_theta_closed
        self.switch_input_torque  .sin1.value = self.desired_torque

        plug (self.switch_input_position.sout, self.position_controller.reference)
        plug (self.switch_input_torque  .sout, self.torque_controller  .reference)

        from dynamic_graph.sot.core.latch import Latch
        self.latch_gripper_closed = Latch (self.name + "_latch_gripper_closed")
        self.latch_gripper_closed.turnOff()
        plug (self.latch_gripper_closed.out, self.switch_input_position.boolSelection)
        plug (self.latch_gripper_closed.out, self.switch_input_torque  .boolSelection)

    ### Setup event to tell when object is grasped
    def _makeSteadyControlEvent (self):
        #TODO
        # Either check:
        #  - the torque error (and its derivatives ?)
        #  - the output velocity command (and its derivatives ?)
        # Should we use a time delay to check whether it is steady ?
        pass

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
        plug (self.omega2theta.output, delayTheta.sin)
        plug (delayTheta.previous, self.currentPositionIn)

        ## theta -> phi = theta - theta0
        self.theta2phi = Add_of_vector(self.name + "_sim_theta2phi")
        self.theta2phi.setCoeff1 ( 1)
        self.theta2phi.setCoeff2 (-1)
        plug (delayTheta.current, self.theta2phi.sin1)
        self.theta2phi.sin2.value = theta0

        ## phi -> torque
        from dynamic_graph.sot.core.switch import SwitchVector
        from dynamic_graph.sot.core.operator import CompareVector
        reverse = self.theta_open[0] > theta0[0]
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
        self.setCurrentConditionIn (delay.current)
        plug (delay.previous, self.currentTorqueIn)

    def readPositionsFromRobot (self, robot, jointNames):
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
        plug (self. _joint_selec.sout, self.currentPositionIn)

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

        self.setCurrentConditionIn (self._multiply_by_torque_constants.sout)
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

        self.setCurrentConditionIn (self._torque_selec.sout)
        plug (self._torque_selec.sout, self.currentTorqueIn)

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

        self._tracer.add (self.switch.latch.name + ".out",        "_torque_control_activated")
        self._tracer.add (self.switch._switch.name + ".sout",     "_omega")
        self._tracer.add (self.omega2theta.outputName,            "_theta")
        # self._tracer.add (self.theta2phi.sout,                    "_phi")
        self._tracer.add (self.torque_controller.referenceName,   "_reference_torque")
        self._tracer.add (self.torque_controller.measurementName, "_measured_torque")
        # self._tracer.add (self.currentConditionIn.name,   # Measured torque
                # filename_escape(self.name + "_"))

        # self._tracer.add (self.switch._condition_up.sout)
        # self._tracer.add (self.switch._condition_down.sout)
        robot.device.after.addSignal(self._tracer.name + ".triger")
        return self._tracer

    @property
    def outputPosition (self):
        return self.omega2theta.output

    @property
    def outputVelocity (self):
        return self.switch.signalOut

    @property
    def referencePositionIn (self):
        return self.position_controller.reference

    @property
    def referenceTorqueIn (self):
        return self.torque_controller.reference

    @property
    def currentPositionIn (self):
        return self.position_controller.measurement

    @property
    def currentTorqueIn (self):
        return self.torque_controller.measurement

    def setCurrentConditionIn (self,sig):
        return self.switch.setMeasurement(sig)

    @property
    def switchEventToTorqueCheck (self):
        return self.switch.eventUp.check

    @property
    def switchEventToPositionCheck (self):
        return self.switch.eventDown.check

    @property
    def torqueConstants (self):
        return self._multiply_by_torque_constants.sin0

    @property
    def robotVelocityOut (self):
        return self._to_robot_velocity.sout

# vim: set foldmethod=indent
