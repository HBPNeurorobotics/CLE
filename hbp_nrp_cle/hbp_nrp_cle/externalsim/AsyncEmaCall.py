from __future__ import print_function

import rospy
from concurrent.futures import ThreadPoolExecutor

class AsyncServiceProxy(object):

    def __init__(self, service_name, service_type, persistent=True,
                 headers=None, callback=None):
        """Create an asynchronous service proxy."""

        self.executor = ThreadPoolExecutor(max_workers=1)
        self.service_proxy = rospy.ServiceProxy(
            service_name,
            service_type,
            persistent,
            headers)
        self.callback = callback

    def __call__(self, *args, **kwargs):
        """Get a Future corresponding to a call of this service."""

        fut = self.executor.submit(self.service_proxy.call, *args, **kwargs)
        if self.callback is not None:
            fut.add_done_callback(self.callback)

        return fut