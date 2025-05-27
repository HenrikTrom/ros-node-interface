import rospy
from std_srvs.srv import Trigger, TriggerResponse
import threading

class BaseInterfaceModule():
    """
    Base Class for all for instances of BaseRosInterface to inherit from. 
    """

    def __init__(self, type) -> None:
        self.type=type
        
    def start(self):
        raise NotImplementedError()
    
    def stop(self):
        raise NotImplementedError()
    
    def shutdown(self):
        raise NotImplementedError()
    
    def loop_step(self):
        raise NotImplementedError()

class TestModule(BaseInterfaceModule):
    def __init__(self) -> None:
        super().__init__(type="test")
        self.i = 0

    def start(self):
        print(f'Starting {self.type}')

    def stop(self):
        print(f'Stopping {self.type}')

    def shutdown(self):
        print(f'Shutdown {self.type}')

    def loop_step(self):
        print(self.i)
        self.i+=1

class BaseRosInterface():
    """
    Wrapper class with start, stop and shutdown services. 
    """
    def __init__(self, module: BaseInterfaceModule, name: str, rate: float) -> None:
        rospy.init_node(name)
        self.module = module
        self.name = f'{self.module.type}'
        self.should_run = False
        start_srv = rospy.Service(f'{self.name}/start', Trigger, self.start_cb)
        stop_srv = rospy.Service(f'{self.name}/stop', Trigger, self.stop_cb)
        shutdown_srv = rospy.Service(f'{self.name}/shutdown', Trigger, self.shutdown_cb)
        self.rate = rospy.Rate(rate)
        rospy.spin()

    def threadfunction(self):
        while self.should_run and not rospy.is_shutdown():
            self.module.loop_step()
            self.rate.sleep()
        rospy.loginfo(f'Stopped {self.name}')

    def start_cb(self, msg: Trigger):
        res = TriggerResponse()
        if self.should_run:
            res.message = "Process is already running"
            res.success = False
        else:
            rospy.loginfo(f'Starting {self.name}')
            res.message = "Starting"
            self.should_run = True
            self.module.start()
            self.mainthread = threading.Thread(target=self.threadfunction, args=())
            self.mainthread.start()
            res.success = True
        return res

    def stop(self):
        res = TriggerResponse()
        if self.should_run:
            rospy.loginfo(f'Stopping {self.name}')
            res.message = "Stopping"
            self.should_run = False
            self.mainthread.join()
            self.module.stop()
            res.success = True
        else:
            res.message = "Process is not running"
            res.success = False
        return res

    def stop_cb(self, msg: Trigger):
        return self.stop()

    def shutdown_cb(self, msg: Trigger):
        res = self.stop()
        rospy.loginfo(f'Shutting down {self.name}');
        rospy.signal_shutdown('Shutdown callback')
        return res

if __name__ == "__main__":
    module = TestModule()
    interface = BaseRosInterface(module=module, name=f'{module.type}_interface')
