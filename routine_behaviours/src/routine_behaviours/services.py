import rospy
import re
from std_srvs.srv import Empty, EmptyResponse

from strands_executive_msgs import task_utils
from strands_executive_msgs.msg import Task
from strands_executive_msgs.srv import DemandTask, SetExecutionStatus


class TaskDemander(object):
    """ Create a services that will propose a the given task on demand """
    def __init__(self):
        super(TaskDemander, self).__init__()

        demand_task_service = '/task_executor/demand_task'
        set_exe_stat_srv_name = '/task_executor/set_execution_status'
        rospy.loginfo("Waiting for task_executor service...")
        rospy.wait_for_service(demand_task_service)
        rospy.wait_for_service(set_exe_stat_srv_name)
        rospy.loginfo("Done")        
        self.demand_task_srv = rospy.ServiceProxy(demand_task_service, DemandTask)
        set_execution_status = rospy.ServiceProxy(set_exe_stat_srv_name, SetExecutionStatus)
        set_execution_status(True)
        self.services = []


    def demand_task(self, task):
        """ Demand a task and catch any exceptions """
    	try:
    		self.demand_task_srv(task)
    	except Exception, e:
    		rospy.logwarn(e)

    def offer_demand_service(self, task):

        def demand(req):
            rospy.loginfo('Demanding task of type %s' % task.action)
            self.demand_task(task)
            return EmptyResponse()

        service_name = 'demand_' + str(len(self.services)) + '_' + re.sub(r'\W', '_', task.action)
        service = rospy.Service(service_name, Empty, demand)
        self.services.append(service)
        return service_name



