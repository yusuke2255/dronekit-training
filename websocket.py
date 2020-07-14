# -*- coding: utf-8 -*-

import json
import time

from gevent.pywsgi import WSGIServer
from geventwebsocket.handler import WebSocketHandler

from flask import Flask, request, render_template

import command

app = Flask(__name__)
app.config.from_object(__name__)

import vehicle_wrapper_ws
import argparse

parser = argparse.ArgumentParser(
            prog='runner.py',
            description='dronekit training',
            epilog='end',
            add_help=True,
            )

parser.add_argument('-i', '--ip', required=True)

args = parser.parse_args()

wrapper = vehicle_wrapper_ws.WsVehicleWrapper(args.ip)

# websocket.py

# @app.route('/')
# def index():
#     return render_template('index.html')

class Subscriber:
    ws = None
    location_callback = None
    is_arm_callback = None

    def __init__(self, ws):
        self.ws = ws

    def send(self, attr_name, value):
        if attr_name == 'location.global_frame':
            data = {
                'event': 'change_attr',
                'attr': attr_name,
                'value': {
                    'lat': value.lat,
                    'lon': value.lon,
                    'alt': value.alt
                }
            }
            self.ws.send(json.dumps(data))
        elif attr_name == 'armed':
            print "change arm"
            data = {
                'event': 'change_attr',
                'attr': attr_name,
                'value': value
            }
            self.ws.send(json.dumps(data))

    def subscribe(self):
        location_callback = lambda s, attr_name, value: self.send(attr_name, value)
        self.location_callback = location_callback
        wrapper.vehicle.add_attribute_listener('location.global_frame', location_callback)
        wrapper.vehicle.add_attribute_listener('armed', location_callback)

    def unsubscribe(self):
        wrapper.vehicle.remove_attribute_listener('location.global_frame', self.location_callback)
        wrapper.vehicle.remove_attribute_listener('armed', self.location_callback)

def create_message_event(msg):
    return json.dumps({
        'event': 'message',
        'value': msg
    })

@app.route('/pipe')
def pipe():
    if request.environ.get('wsgi.websocket'):
        ws = request.environ['wsgi.websocket']
        subscriber = Subscriber(ws)
        subscriber.subscribe()
        while True:
            time.sleep(1)

            message = ws.receive()
            if message is None:
                break
            event = command.resolve(message)

            if isinstance(event, command.ArmCommand):
                wrapper.arm_and_wait(ws)
            elif isinstance(event, command.TakeOffCommand):
                wrapper.takeoff_and_wait(event.altitude, ws)
            elif isinstance(event, command.GoToCommand):
                wrapper.go_and_wait(event.distance, event.azimuth, wrapper.retrieve_current_altitude(), ws)
            elif isinstance(event, command.RtlCommand):
                wrapper.rtl_and_close()
                subscriber.unsubscribe()
            elif isinstance(event, command.UnknownCommand):
                wrapper.rtl_and_close()
                subscriber.unsubscribe()
                msg = create_message_event("Unknown command.Return to launch [message=%s]" %event.message)
                ws.send(msg)
            else:
                wrapper.rtl_and_close()
                subscriber.unsubscribe()
                msg = create_message_event("Error.Return to launch")
                ws.send(msg)

    return

if __name__ == '__main__':
   app.debug = True

   host = 'localhost'
   port = 8080

   host_port = (host, port)
   server = WSGIServer(
       host_port,
       app,
       handler_class=WebSocketHandler
   )
   server.serve_forever()