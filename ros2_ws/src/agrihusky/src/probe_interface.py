#!/usr/bin/env python3

import rclpy, serial, time

from rclpy.node import Node
from  agrihusky.srv import ProbeRequest

###

class ProbeInterface(Node):
    
    def __init__(self, rate=10, port='/dev/ttyACM1'):
        super().__init__('probe_interface')

        self.probe = serial.Serial(port, 9600)  # Serial interface to probe  while self.probe.read() : pass
        self.rate = rate

        self.srv = self.create_service(ProbeRequest, 'probe_request', self.probeCallback)


    def probeCallback(self, request, response):
        self.probe.write(request.start)  # Write PROBE_START in serial port

        while True:  # Wait until reading is obtained from the probe
            if self.probe.in_waiting > 0:
                status = self.probe.read()
                
                if status == b'\xFF':  result = self.probe.read()
                else:                  result = None
                break  # If probe gave succesful reading return data and status otherwise only status

            time.sleep(0.01)

        response.status = status;  response.data = result
        self.get_logger().info(f'Request: start={request.start} -> status={response.status}, data={response.data}')

        return response

###

def main(args=None):
    try:
        rclpy.init(args=args)
        probeInterface = ProbeInterface(rate=25, port='/dev/ttyACM1')    
        rclpy.spin(probeInterface)
    except KeyboardInterrupt:
        pass
    finally:
        probeInterface.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()