#!/usr/bin/env python
import subprocess
import os

def get_index_of_mic(output, search_str):
    splits = (str(output)).split('index:')[1:]
    for sink in splits:
        term = sink.strip()
        if search_str in term:
            terms = term.split(' ')
            t = int(terms[0].replace('\n\tname:', ''))
            return t

if __name__ == "__main__":
    command = ['pacmd', 'list-sinks']
    p = subprocess.Popen(command)

    p = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)
    output = p.stdout.read()
    description = 'Misty_Virtual_Mic'

    if description not in str(output):
        # Add sink
        c2 = ['pactl', 'load-module', 'module-null-sink', 'sink_name=MistyMicOutput', 'sink_properties=device.description="Misty_Virtual_Mic"']
        p2 = subprocess.Popen(c2)

        # Find index
        p3 = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, close_fds=True)
        output = p3.stdout.read()
        index = get_index_of_mic(output, description)
    else:
        index = get_index_of_mic(output, description)
    # index = get_index_of_mic(output, description)
    # print(index)

    c4 = ['pacmd', 'set-default-sink', str(index)]
    p4 = subprocess.Popen(c4)
