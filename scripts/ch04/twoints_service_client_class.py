#!/usr/bin/env python

import rospy
from deu_ros.srv import TwoInts, TwoIntsRequest
import sys


class TwoInstsServiceClient:
    def __init__(self):
        rospy.init_node('twoints_service_client_class')
        rospy.wait_for_service('twoints_service')
        self.service = rospy.ServiceProxy('twoints_service', TwoInts)

    def get_result(self, a, b):
        request = TwoIntsRequest(a, b)
        resp = self.service(request)
        return resp.result_add, resp.result_mult


if __name__=="__main__":
    client = TwoInstsServiceClient()
    if len(sys.argv) < 3:
        print "Two arguments of int type is required."
        sys.exit(-1)

    a, b = int(sys.argv[1]), int(sys.argv[2])

    for i in range(3):
        result_add, result_mult = client.get_result(a, b)
        print 'result_add =', result_add, ', result_mult =', result_mult
        rospy.sleep(1)