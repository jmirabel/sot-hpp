from hpp.corbaserver.manipulation.constraint_graph_factory import ConstraintFactoryAbstract, GraphFactoryAbstract
from tools import Manifold, Grasp, PreGrasp, OpFrame, EndEffector
from dynamic_graph.sot.core import SOT

class Affordance(object):
    def __init__ (self, gripper, handle, openControlType, closeControlType,
            refs, controlParams = {}, simuParams = {}):
        self.gripper = gripper
        self.handle  = handle
        self.controlType = { "open": openControlType, "close": closeControlType, }
        self.ref = refs
        self.controlParams = controlParams
        self.simuParams = simuParams

    def setControl (self, refOpen, refClose, openType = "position", closeType="position"):
        from warnings import warn
        warn("Method Affordance.getControl will be deleted soon!")
        if openType != "position" or closeType != "position":
            raise NotImplementedError ("Only position control is implemented for gripper opening/closure.")
        self.controlType = {
                "open": openType,
                "close": closeType,
                }
        self.ref = {
                "angle_open": refOpen,
                "angle_close": refClose,
                }

    def getControlParameter (self):
        wn    = self.controlParams.get("wn",    10.)
        z     = self.controlParams.get("z",     1. )
        nums_tor   = self.controlParams.get("torque_num", (1.,)),
        denoms_tor = self.controlParams.get("torque_denom", (1.,))
        return wn, z, nums_tor, denoms_tor

    def getSimulationParameters (self):
        mass    = self.simuParams.get("mass"   , 0.)
        damping = self.simuParams.get("damping", 5. )
        spring  = self.simuParams.get("spring" , 100. )
        refPos  = self.simuParams.get("refPos" , self.ref["angle_close"] )
        return mass,damping,spring,refPos

class TaskFactory(ConstraintFactoryAbstract):
    gfields = ('grasp', 'pregrasp', 'gripper_open', 'gripper_close')
    pfields = ()

    def __init__ (self, graphfactory):
        super(TaskFactory, self).__init__ (graphfactory)
        self._grippers = dict()

    def _buildGripper (self, type, gripper, handle):
        key = (type, gripper, handle)
        if self._grippers.has_key (key):
            return self._grippers[key]
        try:
            aff = self.graphfactory.affordances[(gripper, handle)]
        except KeyError:
            # If there are no affordance, do not add a task.
            self._grippers[key] = Manifold ()
            return self._grippers[key]
        gf = self.graphfactory
        gripperFrame = gf.gripperFrames[gripper]
        if not gripperFrame.enabled:
            self._grippers[key] = Manifold ()
            return self._grippers[key]
        robot = gf.sotrobot
        if aff.controlType[type] == "position":
            ee = EndEffector (robot, gripperFrame, "p" + type + ("_" + handle if handle is not None else ""))
            ee.makePositionControl (aff.ref["angle_"+type])
            self._grippers[key] = ee
        elif aff.controlType[type] == "torque" or aff.controlType[type] == "position_torque":
            ee = EndEffector (robot, gripperFrame, "pt_" + type + ("_" + handle if handle is not None else ""))
            ee.makeAdmittanceControl (aff, type,
                    period = gf.parameters["period"],
                    simulateTorqueFeedback = gf.parameters.get("simulateTorqueFeedback",False))
            if gf.parameters["addTracerToAdmittanceController"]:
                tracer = ee.ac.addTracerRealTime (robot)
                gf.tracers[tracer.name] = tracer
            self._grippers[key] = ee
        else:
            raise NotImplementedError ("Control type " + type + " is not implemented for gripper.")
        return self._grippers[key]

    def buildGrasp (self, g, h, otherGrasp=None):
        gf = self.graphfactory
        if h is None:
            gripper_open = self._buildGripper ("open", g, h)
            return { 'gripper_open': gripper_open }
        gripper = gf.gripperFrames[g]
        handle  = gf.handleFrames [h]

        gripper_close = self._buildGripper ("close", g, h)
        pregrasp = PreGrasp (gripper, handle, otherGrasp)
        pregrasp.makeTasks (gf.sotrobot, gf.parameters["useMeasurementOfObjectsPose"])

        if not gripper.enabled:
            # TODO If otherGrasp is not None,
            # we should include the grasp function of otherGrasp, not pregrasp function...
            grasp = Manifold()
        else:
            grasp = Grasp (gripper, handle, otherGrasp)
            grasp.makeTasks (gf.sotrobot)
        return { 'grasp': grasp,
                 'pregrasp': pregrasp,
                 'gripper_close': gripper_close }

    ## \name Accessors to the different elementary constraints
    # \{
    def getGrasp(self, gripper, handle, otherGrasp=None):
        if isinstance(gripper, str): ig = self.graphfactory.grippers.index(gripper)
        else: ig = gripper
        if isinstance(handle, str): ih = self.graphfactory.handles.index(handle)
        else: ih = handle
        if otherGrasp is not None:
            otherIg = self.graphfactory.grippers.index(otherGrasp[0].key)
            otherIh = self.graphfactory.handles.index(otherGrasp[1].key)
            k = (ig, ih, otherIg, otherIh)
        else:
            k = (ig, ih)
        if not self._grasp.has_key(k):
            self._grasp[k] = self.buildGrasp(self.graphfactory.grippers[ig], None if ih is None else self.graphfactory.handles[ih], otherGrasp)
            assert isinstance (self._grasp[k], dict)
        return self._grasp[k]

    def g (self, gripper, handle, what, otherGrasp=None):
        return self.getGrasp(gripper, handle, otherGrasp)[what]

    def buildPlacement (self, o):
        # Nothing to do
        return dict()

class Factory(GraphFactoryAbstract):
    class State:
        def __init__ (self, tasks, grasps, factory):
            self.name = factory._stateName (grasps)
            self.grasps = grasps
            self.manifold = Manifold()

            self.objectsAlreadyGrasped = {}
            
            for ig, ih in enumerate(grasps):
                if ih is not None:
                    # Add task gripper_close
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'gripper_close')
                    otherGrasp = self.objectsAlreadyGrasped.get(factory.objectFromHandle[ih])
                    self.manifold += tasks.g (factory.grippers[ig], factory.handles[ih], 'grasp', otherGrasp)
                    self.objectsAlreadyGrasped[factory.objectFromHandle[ih]] = (
                            factory.gripperFrames[factory.grippers[ig]],
                            factory. handleFrames[factory.handles[ih]],)
                else:
                    # Add task gripper_open
                    self.manifold += tasks.g (factory.grippers[ig], None, 'gripper_open')

    def __init__ (self, supervisor):
        super(Factory, self).__init__ ()
        self.tasks = TaskFactory (self)
        self.hpTasks = supervisor.hpTasks
        self.lpTasks = supervisor.lpTasks
        self.affordances = dict()
        self.sots = dict()
        ## A dictionnary
        # - key: name of the transition after which an action must be done
        # - value: dictionnary:
        #          - key: the reached state (at most two values, depending on whether the dst state was reached)
        #          - value: sot representing the post-action
        self.postActions = dict()
        ## A dictionnary
        # - key: name of the transition before which an action must be done
        # - value: sot representing the pre-action
        self.preActions = dict()

        self.timers = {}
        self.tracers = {}
        self.controllers = {}
        self.supervisor = supervisor

        ## Accepted parameters:
        ## - period: [double, no default]
        ##           Time interval between two iterations of the graph.
        ## - simulateTorqueFeedback: [boolean, False]
        ##                           do not use torque feedback from the robot
        ##                           but simulate it instead.
        ## - useMeasurementOfObjectsPose: [boolean, False]
        ##                                whether SoT should use tf_listener to get the real position of objects.
        self.parameters = {
                "addTracerToAdmittanceController": False,
                "addTimerToSotControl": False,
                "simulateTorqueFeedback": False,
                "useMeasurementOfObjectsPose": False,
                }

    def _newSoT (self, name):
        sot = SOT (name)
        sot.setSize(self.sotrobot.dynamic.getDimension())
        sot.damping.value = 0.001

        if self.parameters["addTimerToSotControl"]:
            from .tools import insertTimerOnOutput
            self.timers[name] = insertTimerOnOutput (sot.control, "vector")
            self.SoTtracer.add (self.timers[name].name + ".timer", str(len(self.timerIds)) + ".timer")
            self.timerIds.append(name)
        return sot

    def addAffordance (self, aff, simulateTorqueFeedback = False):
        assert isinstance(aff, Affordance)
        self.affordances [(aff.gripper, aff.handle)] = aff

    def generate (self):
        if self.parameters["addTimerToSotControl"]:
            # init tracer
            from dynamic_graph.tracer_real_time import TracerRealTime
            SoTtracer = TracerRealTime ("tracer_of_timers")
            self.tracers[SoTtracer.name] = SoTtracer
            self.timerIds = []
            SoTtracer.setBufferSize (10 * 1048576) # 10 Mo
            SoTtracer.open ("/tmp", "sot-control-trace", ".txt")
            self.sotrobot.device.after.addSignal("tracer_of_timers.triger")
            self.supervisor.SoTtracer = SoTtracer
            self.supervisor.SoTtimerIds = self.timerIds
        super(Factory, self).generate ()

        self.supervisor.sots = self.sots
        self.supervisor.grasps = { (gh, w): t for gh, ts in self.tasks._grasp.items() for w, t in ts.items() }
        self.supervisor.hpTasks = self.hpTasks
        self.supervisor.lpTasks = self.lpTasks
        self.supervisor.postActions = self.postActions
        self.supervisor.preActions  = self.preActions
        self.supervisor.SoTtimers = self.timers
        self.supervisor.tracers = self.tracers
        self.supervisor.controllers = self.controllers

        from dynamic_graph import plug
        self.supervisor.sots_indexes = dict()
        for tn,sot in self.sots.iteritems():
            # Pre action
            if self.preActions.has_key(tn):
                pa_sot = self.preActions[tn]
                self.supervisor.addSignalToSotSwitch (pa_sot.name, pa_sot.control)
            # Action
            if self.timers.has_key (sot.name):
                self.supervisor.addSignalToSotSwitch (sot.name, self.timers[sot.name].sout)
            else:
                self.supervisor.addSignalToSotSwitch (sot.name, sot.control)
            # Post action
            if self.postActions.has_key(tn):
                d = self.postActions[tn]
                for targetState, pa_sot in d.iteritems():
                    self.supervisor.addSignalToSotSwitch (pa_sot.name, pa_sot.control)

    def setupFrames (self, srdfGrippers, srdfHandles, sotrobot, disabledGrippers = ()):
        self.sotrobot = sotrobot

        self.grippersIdx = { g: i for i,g in enumerate(self.grippers) }
        self.handlesIdx  = { h: i for i,h in enumerate(self.handles) }

        self.gripperFrames = { g: OpFrame(srdfGrippers[g], sotrobot.dynamic.model, g not in disabledGrippers) for g in self.grippers }
        self.handleFrames  = { h: OpFrame(srdfHandles [h]                                                   ) for h in self.handles  }

        # Compute the DoF which should not be affected by the
        # task not related to end-effectors.
        gripper_joints = []
        for g,gripper in self.gripperFrames.iteritems():
            try:
                gripper_joints += gripper.joints
            except:
                print "Gripper", gripper.name, "has not joints. Cannot be controlled."
        from .tools import computeControlSelection
        # print "Remove gripper_joints from solvers", gripper_joints
        self.dof_selection = computeControlSelection (sotrobot,gripper_joints)
        self.hpTasks.setControlSelection (self.dof_selection)
        self.lpTasks.setControlSelection (self.dof_selection)

    def makeState (self, grasps, priority):
        # Nothing to do here
        return Factory.State(self.tasks, grasps, self)

    def makeLoopTransition (self, state):
        n = self._loopTransitionName(state.grasps)
        sot = self._newSoT ('sot_'+n)

        self.hpTasks.pushTo(sot)
        state.manifold.pushTo(sot)
        self.lpTasks.pushTo(sot)

        self.sots[n] = sot

    def makeTransition (self, stateFrom, stateTo, ig):
        sf = stateFrom
        st = stateTo
        names = self._transitionNames(sf, st, ig)

        iobj = self.objectFromHandle [st.grasps[ig]]
        obj = self.objects[iobj]
        noPlace = self._isObjectGrasped (sf.grasps, iobj)
        #TODO compute other grasp on iobj
        # it must be a grasp or pregrasp task
        otherGrasp = sf.objectsAlreadyGrasped.get(iobj ,None)

        # The different cases:
        pregrasp = True
        intersec = not noPlace
        preplace = not noPlace

        # Start here
        nWaypoints = pregrasp + intersec + preplace
        nTransitions = 1 + nWaypoints

        # Link waypoints
        transitions = names[:]
        assert nWaypoints > 0
        M = 1 + pregrasp
        sots = [ ]
        for i in range(nTransitions):
            ns = ("{0}_{1}{2}".format(names[0], i, i+1),
                  "{0}_{2}{1}".format(names[1], i, i+1))

            for n in ns:
                s = self._newSoT('sot_'+n)
                self.hpTasks.pushTo(s)

                if pregrasp and i == 1:
                    # Add pregrasp task
                    pregrasp = self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp = otherGrasp)
                    pregrasp.pushTo (s)
                if i < M: sf.manifold.pushTo(s)
                else:     st.manifold.pushTo(s)

                self.lpTasks.pushTo(s)
                self.sots[n] = s
                sots.append (n)

        ## Post-actions for transitions from
        # 1. pregrasp to intersec, intersec (st) reached:
        #   - "pregrasp"
        #   - "gripper_close"
        key = sots[2*(M-1)]
        sot = self._newSoT ("postAction_" + key)
        self.hpTasks.pushTo (sot)
        # "gripper_close" is in st.manifold
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_close').pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        st.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        if not self.postActions.has_key(key):
            self.postActions[ key ] = dict()
        self.postActions[ key ] [ st.name ] = sot

        # 2. intersec to pregrasp, pregrasp (sf) reached:
        # TODO Should this post-action be done ?
        # Force the re-alignment with planning.
        key = sots[2*(M-1)+1]
        sot = self._newSoT ("postAction_" + key)
        self.hpTasks.pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        if not self.postActions.has_key(key):
            self.postActions[ key ] = dict()
        self.postActions[ key ] [ sf.name ] = sot


        ## Pre-actions for transitions from

        # 1.intersec to pregrasp:
        #   - "pregrasp": the motion must be relative to the object
        #   - "gripper_open"
        key = sots[2*(M-1) + 1]
        sot = self._newSoT ("preAction_" + key)
        self.hpTasks.pushTo (sot)
        # "gripper_open" is in sf.manifold
        # self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'gripper_open').pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        self.preActions[ key ] = sot

        # 2. pregrasp to intersec:
        #   - "pregrasp": the motion must be relative to the object
        # Required to force the alignment gripper / handle before the actual grasp.
        key = sots[2*(M-1)]
        sot = self._newSoT ("preAction_" + key)
        self.hpTasks.pushTo (sot)
        self.tasks.g (self.grippers[ig], self.handles[st.grasps[ig]], 'pregrasp', otherGrasp).pushTo (sot)
        sf.manifold.pushTo (sot)
        self.lpTasks.pushTo (sot)
        self.preActions[ key ] = sot
